#pragma once

#include <fstream>
#include <vector>
#include "pm_recording/async_writer.hpp"

namespace pm_recording
{
// 標準の同期writeをそのまま転送する診断用decorator。FIFOや追加購読は作らない。
// time_stampはrecorderが採番した時刻であり、DDS到着時刻ではない。
// この入口より前の遅延をDDS、executor、Writerのmutexへ個別分解はできない。
class TimingWriter : public rosbag2_cpp::writer_interfaces::BaseWriterInterface
{
public:
  using Stats = AsyncWriter::Stats;
  explicit TimingWriter(
    std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> target,
    bool enabled) : target_(std::move(target)), enabled_(enabled)
  {
    if (enabled_) {rows_.reserve(30000);}
  }
  void open(const rosbag2_cpp::StorageOptions & storage,
    const rosbag2_cpp::ConverterOptions & converter) override
  {
    path_ = storage.uri;
    target_->open(storage, converter);
    std::lock_guard<std::mutex> lock(mutex_);
    state_.opened = true;
  }
  void create_topic(const rosbag2_storage::TopicMetadata & topic) override
  {target_->create_topic(topic);}
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override
  {target_->remove_topic(topic);}
  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override
  {
    // Writerがserializeした呼び出しに加え、monitorのstats参照との競合も防ぐ。
    // 測定OFF時はclock取得・trace蓄積を一切行わない。
    std::lock_guard<std::mutex> lock(mutex_);
    ++state_.accepted;
    state_.accepted_bytes += message->serialized_data->buffer_length;
    const auto enter = enabled_ ? std::chrono::steady_clock::now() :
      std::chrono::steady_clock::time_point{};
    const int64_t wall = enabled_ ? std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count() : 0;
    target_->write(message);
    ++state_.written;
    state_.written_bytes += message->serialized_data->buffer_length;
    if (!enabled_) {return;}
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - enter).count();
    state_.max_write_seconds = std::max(state_.max_write_seconds, duration / 1e9);
    // 全IMUと10ms以上のwriteだけを上限付きで保持。記録中にCSVをflushしない。
    if (message->topic_name == "/oak/imu/data" || duration >= 10000000) {
      if (rows_.size() < 30000) {
        rows_.push_back({message->topic_name, message->time_stamp, wall, duration});
      } else {++omitted_;}
    }
  }
  Stats stats() const {std::lock_guard<std::mutex> lock(mutex_); return state_;}
  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.closed) {return;}
    target_->reset();
    state_.closed = true;
    // 購読終了・metadata close後にのみディスクへ診断を保存する。
    std::ofstream out(path_ + "/write_timing.csv");
    out << "topic,record_ns,write_enter_ns,write_duration_ns\n";
    for (const auto & row : rows_) {
      out << row.topic << ',' << row.record << ',' << row.enter << ',' << row.duration << '\n';
    }
    if (!out) {throw std::runtime_error("write_timing.csvの保存に失敗");}
    std::ofstream info(path_ + "/write_timing_info.txt");
    info << "enabled=" << enabled_ << "\nrows=" << rows_.size()
      << "\nomitted=" << omitted_ << "\n";
    if (!info) {throw std::runtime_error("診断情報の保存に失敗");}
  }
private:
  struct Row {std::string topic; int64_t record, enter, duration;};
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> target_;
  bool enabled_;
  std::string path_;
  mutable std::mutex mutex_;
  Stats state_;
  std::vector<Row> rows_;
  size_t omitted_ = 0;
};
}
