#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <vector>
#include "pm_recording/async_writer.hpp"
#include "pm_recording/timing_writer.hpp"

namespace
{
// storageを遅延・故障させる偽物で、センサなしでも所有権・FIFO・停止を検証する。
struct FakeStorage : rosbag2_cpp::writer_interfaces::BaseWriterInterface {
  std::vector<std::string> events;
  bool fail = false;
  void open(const rosbag2_cpp::StorageOptions &, const rosbag2_cpp::ConverterOptions &) override {}
  void reset() override {events.push_back("close");}
  void create_topic(const rosbag2_storage::TopicMetadata & t) override {events.push_back(t.name);}
  void remove_topic(const rosbag2_storage::TopicMetadata &) override {events.push_back("remove");}
  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) override {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (fail) {throw std::runtime_error("simulated disk error");}
    events.push_back(std::to_string(m->time_stamp));
  }
};
auto message(int64_t stamp, size_t bytes)
{
  auto result = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  result->time_stamp = stamp;
  result->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  result->serialized_data->buffer_length = bytes;
  return result;
}
}

TEST(AsyncWriter, PreservesOrderAndDrainsOnStop)
{
  auto inner = std::make_unique<FakeStorage>();
  auto * storage = inner.get();
  pm_recording::AsyncWriter writer(std::move(inner), 1024, 100, [] {});
  writer.open({}, {});
  writer.create_topic({"/test", "std_msgs/msg/String", "cdr", ""});
  for (int i = 0; i < 10; ++i) {writer.write(message(i, 16));}
  writer.reset();
  EXPECT_EQ(storage->events.size(), 12U);
  EXPECT_EQ(storage->events.front(), "/test");
  EXPECT_EQ(storage->events[10], "9");
  EXPECT_EQ(storage->events.back(), "close");
  EXPECT_EQ(writer.stats().accepted, 10U);
  EXPECT_EQ(writer.stats().written, 10U);
  EXPECT_EQ(writer.stats().accepted_bytes, 160U);
  EXPECT_EQ(writer.stats().written_bytes, 160U);
  EXPECT_TRUE(writer.stats().error.empty());
  writer.reset();
  EXPECT_EQ(storage->events.size(), 12U);
}

TEST(AsyncWriter, RejectsOverflowInsteadOfSilentDrop)
{
  std::atomic<int> failures{0};
  pm_recording::AsyncWriter writer(std::make_unique<FakeStorage>(), 8, 10,
    [&] {++failures;});
  writer.open({}, {});
  EXPECT_THROW(writer.write(message(0, 9)), std::runtime_error);
  writer.reset();
  EXPECT_EQ(failures.load(), 1);
  EXPECT_FALSE(writer.stats().error.empty());
  EXPECT_EQ(writer.stats().accepted, 0U);
  EXPECT_EQ(writer.stats().accepted_bytes, 0U);
}

TEST(AsyncWriter, CapturesStorageFailure)
{
  auto inner = std::make_unique<FakeStorage>();
  inner->fail = true;
  std::atomic<int> failures{0};
  pm_recording::AsyncWriter writer(std::move(inner), 1024, 100, [&] {++failures;});
  writer.open({}, {});
  writer.write(message(0, 16));
  writer.reset();
  EXPECT_EQ(failures.load(), 1);
  EXPECT_EQ(writer.stats().accepted, 1U);
  EXPECT_EQ(writer.stats().written, 0U);
  EXPECT_EQ(writer.stats().accepted_bytes, 16U);
  EXPECT_EQ(writer.stats().written_bytes, 0U);
  EXPECT_EQ(writer.stats().error, "simulated disk error");
}

TEST(AsyncWriter, RejectsStandardCacheAndWriteAfterClose)
{
  pm_recording::AsyncWriter writer(std::make_unique<FakeStorage>(), 1024, 100, [] {});
  rosbag2_cpp::StorageOptions options;
  options.max_cache_size = 1;
  EXPECT_THROW(writer.open(options, {}), std::runtime_error);
  options.max_cache_size = 0;
  writer.open(options, {});
  writer.reset();
  EXPECT_THROW(writer.write(message(0, 16)), std::runtime_error);
}

TEST(AsyncWriter, CountsInflightAndDrainsAcceptedAfterOverflow)
{
  struct BlockingStorage : FakeStorage {
    std::promise<void> entered, release;
    std::shared_future<void> gate = release.get_future().share();
    void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> m) override {
      entered.set_value();
      gate.wait();
      events.push_back(std::to_string(m->time_stamp));
    }
  };
  auto inner = std::make_unique<BlockingStorage>();
  auto * storage = inner.get();
  auto entered = storage->entered.get_future();
  pm_recording::AsyncWriter writer(std::move(inner), 16, 1, [] {});
  writer.open({}, {});
  writer.write(message(0, 16));
  entered.wait();
  // queueから取り出した書込み中1件も、byte/item上限から除外しない。
  EXPECT_THROW(writer.write(message(1, 1)), std::runtime_error);
  storage->release.set_value();
  writer.reset();
  EXPECT_EQ(writer.stats().accepted, 1U);
  EXPECT_EQ(writer.stats().written, 1U);
  EXPECT_FALSE(writer.stats().error.empty());
  EXPECT_EQ(writer.stats().peak_bytes, 16U);
  EXPECT_EQ(writer.stats().accepted_bytes, 16U);
  EXPECT_EQ(writer.stats().written_bytes, 16U);
}

TEST(TimingWriter, ForwardsSynchronouslyWithoutChangingStamp)
{
  auto inner = std::make_unique<FakeStorage>();
  auto * storage = inner.get();
  pm_recording::TimingWriter writer(std::move(inner), true);
  writer.write(message(123456789, 16));
  // writeから戻る時点で下流書込み済み。非同期queueへの投入ではない。
  ASSERT_EQ(storage->events.size(), 1U);
  EXPECT_EQ(storage->events.front(), "123456789");
  EXPECT_EQ(writer.stats().written, 1U);
  EXPECT_GE(writer.stats().max_write_seconds, .009);
}

TEST(TimingWriter, DisabledAndFailurePropagation)
{
  auto inner = std::make_unique<FakeStorage>();
  auto * storage = inner.get();
  pm_recording::TimingWriter writer(std::move(inner), false);
  writer.write(message(7, 16));
  EXPECT_EQ(writer.stats().max_write_seconds, 0);
  storage->fail = true;
  EXPECT_THROW(writer.write(message(8, 16)), std::runtime_error);
  EXPECT_EQ(writer.stats().accepted, 2U);
  EXPECT_EQ(writer.stats().written, 1U);
}
