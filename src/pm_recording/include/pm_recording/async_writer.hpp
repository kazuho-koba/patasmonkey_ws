#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#ifdef __linux__
#include <pthread.h>
#endif
#include "rosbag2_cpp/writer_interfaces/base_writer_interface.hpp"

namespace pm_recording
{
// Foxyのwriter境界だけを非同期化する。受信時刻・CDRの所有権・順序は変更せず、
// topic登録も同じFIFOへ入れるため、discoveryとmessage保存がstorageへ並行アクセスしない。
class AsyncWriter : public rosbag2_cpp::writer_interfaces::BaseWriterInterface
{
public:
  struct Stats {
    uint64_t accepted = 0, written = 0, peak_bytes = 0, peak_messages = 0;
    // 圧縮前CDRの累積量。差分/経過秒で流入・保存帯域を直接比較できる。
    // ファイル容量や件数だけでは画像サイズと小さいIMUの偏りを判別できない。
    uint64_t accepted_bytes = 0, written_bytes = 0;
    uint64_t pending_bytes = 0, pending_items = 0;
    double max_write_seconds = 0;
    bool opened = false, closed = false;
    std::string error;
  };

  AsyncWriter(std::unique_ptr<BaseWriterInterface> inner, uint64_t byte_limit,
              uint64_t message_limit, std::function<void()> on_failure)
  : inner_(std::move(inner)), byte_limit_(byte_limit), message_limit_(message_limit),
    on_failure_(std::move(on_failure)) {}

  ~AsyncWriter() override {reset();}

  void open(const rosbag2_cpp::StorageOptions & storage,
            const rosbag2_cpp::ConverterOptions & converter) override
  {
    if (storage.max_cache_size != 0) {
      throw std::runtime_error("標準cacheは0のみ: 非同期FIFOとは別機構です");
    }
    inner_->open(storage, converter);
    {
      // monitorがopen中にもstats()を読むため、公開状態は同じmutexで更新する。
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.opened = true;
    }
    worker_ = std::thread([this] {run();});
  }

  void create_topic(const rosbag2_storage::TopicMetadata & topic) override
  {enqueue({Kind::Create, topic, nullptr});}

  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override
  {enqueue({Kind::Remove, topic, nullptr});}

  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override
  {enqueue({Kind::Message, {}, std::move(message)});}

  void reset() override
  {
    // transportの購読停止後に呼ぶ。新規受付を閉じ、受理済みFIFOを保存してから
    // metadataを確定する。時間切れで切り捨てず、呼出元がjoin完了まで待つ。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closing_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) {worker_.join();}
    std::lock_guard<std::mutex> lock(mutex_);
    if (stats_.closed || !stats_.opened) {return;}
    try {inner_->reset();} catch (const std::exception & e) {
      if (stats_.error.empty()) {stats_.error = e.what();}
    }
    stats_.closed = true;
  }

  Stats stats() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = stats_;
    result.pending_bytes = pending_bytes_;
    result.pending_items = pending_items_;
    return result;
  }

private:
  enum class Kind {Create, Remove, Message};
  struct Item {
    Kind kind;
    rosbag2_storage::TopicMetadata topic;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
    uint64_t bytes() const {return message ? message->serialized_data->buffer_length : 0;}
  };

  void enqueue(Item item)
  {
    bool failed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stats_.opened || closing_ || !stats_.error.empty()) {
        throw std::runtime_error("非同期writerは受付停止中です");
      }
      // 書込み中の1件も使用量へ含む。容量待ちで購読threadを停止させず、
      // 上限超過を明示的な失敗とし、既に受理したmessageだけは停止時にflushする。
      if (item.bytes() > byte_limit_ - pending_bytes_ || pending_items_ >= message_limit_) {
        stats_.error = "非同期FIFOの上限を超過しました。保存完了とは扱えません";
        failed = true;
      } else {
        pending_bytes_ += item.bytes();
        ++pending_items_;
        stats_.peak_bytes = std::max(stats_.peak_bytes, pending_bytes_);
        stats_.peak_messages = std::max(stats_.peak_messages, pending_items_);
        if (item.message) {
          ++stats_.accepted;
          stats_.accepted_bytes += item.bytes();
        }
        queue_.push_back(std::move(item));
      }
    }
    if (failed) {
      on_failure_();
      throw std::runtime_error("非同期FIFOの上限超過");
    }
    ready_.notify_one();
  }

  void run()
  {
#ifdef __linux__
    // /procの待機診断でDDSとstorage workerを区別できる短いthread名。
    pthread_setname_np(pthread_self(), "pm_bag_writer");
#endif
    while (true) {
      Item item;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] {return closing_ || !queue_.empty();});
        if (queue_.empty()) {return;}
        item = std::move(queue_.front());
        queue_.pop_front();
      }
      bool failed = false;
      const auto begin = std::chrono::steady_clock::now();
      try {
        if (item.kind == Kind::Create) {inner_->create_topic(item.topic);}
        else if (item.kind == Kind::Remove) {inner_->remove_topic(item.topic);}
        else {inner_->write(item.message);}
      } catch (const std::exception & e) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stats_.error.empty()) {stats_.error = e.what();}
        failed = true;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_bytes_ -= item.bytes();
        --pending_items_;
        if (item.message && !failed) {
          ++stats_.written;
          stats_.written_bytes += item.bytes();
          stats_.max_write_seconds = std::max(stats_.max_write_seconds,
            std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count());
        }
        if (failed) {
          // storage故障後に別messageを書いて成功を装わない。未保存数は
          // accepted-writtenに残し、親の整合検査と非zero終了で必ず異常扱いする。
          queue_.clear();
          closing_ = true;
        }
      }
      if (failed) {on_failure_(); return;}
    }
  }

  std::unique_ptr<BaseWriterInterface> inner_;
  uint64_t byte_limit_, message_limit_, pending_bytes_ = 0, pending_items_ = 0;
  std::function<void()> on_failure_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Item> queue_;
  std::thread worker_;
  bool closing_ = false;
  Stats stats_;
};
}  // namespace pm_recording
