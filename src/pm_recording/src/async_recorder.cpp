#include <fstream>
#include <atomic>
#include <iostream>
#include <limits>
#include "pm_recording/async_writer.hpp"
#ifdef PM_TIMING_RECORDER
#include "pm_recording/timing_writer.hpp"
#endif
#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_transport/rosbag2_transport.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
uint64_t positive_number(const std::string & value)
{
  if (value.empty() || value[0] == '-') {throw std::runtime_error("正の整数が必要です");}
  size_t used = 0;
  const auto result = std::stoull(value, &used);
  if (used != value.size() || result == 0) {throw std::runtime_error("正の整数が必要です");}
  return result;
}

// Foxy transportへ渡すQoSを既存YAMLから読む。未知値を自動補完せず拒否し、
// RELIABLE / TRANSIENT_LOCAL等を通常recorderと同じ指定に保つ。
std::unordered_map<std::string, rclcpp::QoS> load_qos(const std::string & path)
{
  std::unordered_map<std::string, rclcpp::QoS> result;
  if (path.empty()) {return result;}
  for (const auto & entry : YAML::LoadFile(path)) {
    const auto config = entry.second;
    auto qos = rclcpp::QoS(positive_number(config["depth"].as<std::string>()));
    const auto history = config["history"].as<std::string>();
    if (history == "keep_all") {qos.keep_all();}
    else if (history != "keep_last") {throw std::runtime_error("未知のhistory");}
    const auto reliability = config["reliability"].as<std::string>();
    if (reliability == "reliable") {qos.reliable();}
    else if (reliability == "best_effort") {qos.best_effort();}
    else {throw std::runtime_error("未知のreliability");}
    const auto durability = config["durability"].as<std::string>();
    if (durability == "transient_local") {qos.transient_local();}
    else if (durability == "volatile") {qos.durability_volatile();}
    else {throw std::runtime_error("未知のdurability");}
    for (const auto & field : config) {
      const auto key = field.first.as<std::string>();
      if (key != "history" && key != "depth" && key != "reliability" && key != "durability") {
        throw std::runtime_error("未対応QoS項目: " + key);
      }
    }
    result.emplace(entry.first.as<std::string>(), qos);
  }
  return result;
}
}

int main(int argc, char ** argv)
{
  try {
    rosbag2_transport::StorageOptions storage;
    storage.storage_id = "mcap";
    rosbag2_transport::RecordOptions record{};
    record.rmw_serialization_format = "cdr";
    record.topic_polling_interval = std::chrono::milliseconds(100);
    record.node_prefix = "pm_async";
    uint64_t bytes = 64ULL*1024*1024, messages = 8192;
    std::string qos_path;
#ifdef PM_TIMING_RECORDER
    bool timing_enabled = false;
#endif
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key[0] != '-') {record.topics.push_back(key); continue;}
      if (++i >= argc) {throw std::runtime_error("optionの値がありません: " + key);}
      const std::string value = argv[i];
      if (key == "-o") {storage.uri = value;}
      else if (key == "-s") {storage.storage_id = value;}
      else if (key == "--max-bag-size") {
        storage.max_bagfile_size = value == "0" ? 0 : positive_number(value);
      }
      else if (key == "--max-cache-size") {
        if (value != "0") {throw std::runtime_error("標準cacheは0のみ使用できます");}
      }
      else if (key == "--polling-interval") {
        record.topic_polling_interval = std::chrono::milliseconds(positive_number(value));
      }
      else if (key == "--qos-profile-overrides-path") {qos_path = value;}
      else if (key == "--queue-bytes") {bytes = positive_number(value);}
      else if (key == "--queue-messages") {messages = positive_number(value);}
#ifdef PM_TIMING_RECORDER
      else if (key == "--trace") {
        if (value != "on" && value != "off") {throw std::runtime_error("traceはon/off");}
        timing_enabled = value == "on";
      }
#endif
      else {throw std::runtime_error("未知のoption: " + key);}
    }
    if (storage.uri.empty() || record.topics.empty()) {
      throw std::runtime_error("出力先 -o と明示topicが必要です");
    }
    record.topic_qos_profile_overrides = load_qos(qos_path);
#ifdef PM_TIMING_RECORDER
    // 同じtransport/QoS/CLIを使うが、writeは標準の同期処理。queue設定は使わない。
    auto async = std::make_unique<pm_recording::TimingWriter>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>(), timing_enabled);
#else
    auto async = std::make_unique<pm_recording::AsyncWriter>(
      std::make_unique<rosbag2_cpp::writers::SequentialWriter>(), bytes, messages,
      [] {rclcpp::shutdown();});
#endif
    auto * state = async.get();
    auto writer = std::make_shared<rosbag2_cpp::Writer>(std::move(async));
    rosbag2_transport::Rosbag2Transport transport(
      std::make_shared<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>()),
      writer, std::make_shared<rosbag2_cpp::Info>());
    transport.init();
    // 有界の1 Hz診断履歴。購読callbackではI/Oせず、停止後にCSVへ保存する。
    // バッファが一過性burstで増えたのか、持続的に増え続けるのかを確認する。
    struct Sample {double elapsed; pm_recording::AsyncWriter::Stats state;};
    std::deque<Sample> samples;
    size_t evicted = 0;
    std::atomic<bool> monitoring{true};
    const auto origin = std::chrono::steady_clock::now();
    std::thread monitor([&] {
      while (monitoring.load()) {
        if (samples.size() == 6000) {samples.pop_front(); ++evicted;}
        samples.push_back({std::chrono::duration<double>(
          std::chrono::steady_clock::now()-origin).count(), state->stats()});
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
    std::string transport_error;
    try {transport.record(storage, record);} catch (const std::exception & e) {
      transport_error = e.what();
      rclcpp::shutdown();
    }
    const bool stopped = !rclcpp::ok();
    // 標準transportが戻った時点で購読は停止済み。FIFOの全flushとmetadata
    // closeを明示的に待つ。transport内部の例外catchによる成功終了も検査する。
    state->reset();
    monitoring.store(false);
    monitor.join();
    const auto stats = state->stats();
    // 1 Hzの最終sampleより後にflushが完了し得る。join後の確定状態を末尾に
    // 追加し、CSVでも滞留解消を確認できるようにする。履歴上限は維持する。
    if (samples.size() == 6000) {samples.pop_front(); ++evicted;}
    samples.push_back({std::chrono::duration<double>(
      std::chrono::steady_clock::now()-origin).count(), stats});
    const bool complete = stopped && transport_error.empty()
      && stats.opened && stats.closed && stats.error.empty()
      && stats.accepted == stats.written && stats.accepted_bytes == stats.written_bytes;
    YAML::Emitter report;
    report << YAML::BeginMap << YAML::Key << "verified" << YAML::Value << complete
      << YAML::Key << "accepted" << YAML::Value << stats.accepted
      << YAML::Key << "written" << YAML::Value << stats.written
      << YAML::Key << "accepted_bytes" << YAML::Value << stats.accepted_bytes
      << YAML::Key << "written_bytes" << YAML::Value << stats.written_bytes
      << YAML::Key << "queue_byte_limit" << YAML::Value << bytes
      << YAML::Key << "queue_message_limit" << YAML::Value << messages
      << YAML::Key << "peak_bytes" << YAML::Value << stats.peak_bytes
      << YAML::Key << "peak_messages" << YAML::Value << stats.peak_messages
      << YAML::Key << "max_write_seconds" << YAML::Value << stats.max_write_seconds
      << YAML::Key << "trace_evicted" << YAML::Value << evicted
      << YAML::Key << "error" << YAML::Value << stats.error
      << YAML::Key << "transport_error" << YAML::Value << transport_error << YAML::EndMap;
    std::ofstream output(storage.uri + "/async_writer.yaml");
    output << report.c_str() << '\n';
    output.close();
    std::ofstream trace(storage.uri + "/async_writer.csv");
    // 既存列を保持して末尾へ追加する。byte差分は圧縮率に依存しないFIFO帯域診断。
    trace << "elapsed_s,accepted,written,pending_bytes,pending_items,max_write_seconds,"
          << "accepted_bytes,written_bytes\n";
    for (const auto & row : samples) {
      trace << row.elapsed << ',' << row.state.accepted << ',' << row.state.written << ','
        << row.state.pending_bytes << ',' << row.state.pending_items << ','
        << row.state.max_write_seconds << ',' << row.state.accepted_bytes << ','
        << row.state.written_bytes << '\n';
    }
    trace.close();
    transport.shutdown();
    return complete && !output.fail() ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr << "async_recorder: " << e.what() << std::endl;
    return 1;
  }
}
