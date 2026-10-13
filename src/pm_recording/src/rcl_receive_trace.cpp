// recorder専用の受信境界診断。LD_PRELOADを指定した試験processだけを観測し、
// rclの引数・戻り値・待機条件・QoSを変更しない。追加購読も作らない。
#include <rcl/wait.h>
#include <rcl/subscription.h>
#include <fastdds/rtps/reader/StatefulReader.h>
#include <rmw_fastrtps_shared_cpp/custom_subscriber_info.hpp>
#include <fastrtps/subscriber/Subscriber.h>
#include <dlfcn.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
// 最大300000件で固定。DDS入口とrcl境界の両方を60秒記録するため、
// rcl単独の200000件から増やす。計測中のファイルI/O・可変長確保を避け、
// 上限を超えた件数は終了時に明記する。時間は同一ホストのsystem/steady ns。
struct Row {
  int64_t wall, begin, end, source, received;
  uint64_t bytes;
  long tid;
  int kind, topic, result, ready, imu_ready;
  uint64_t gid_hi, gid_lo, sequence, sequence_last;
  uint32_t fragment;
};
constexpr size_t limit = 300000;
Row rows[limit];
std::atomic<size_t> used{0};
// Foxy/FastDDS 2.1.4では1つのDATAを対象外readerにも渡す場合がある。
// 実際の2購読のreader GUIDをrcl側で取得し、対象外は時計採番・履歴追加しない。
struct ReaderId {std::atomic<uint64_t> hi{0}, lo{0};};
ReaderId imu_reader, color_reader;
ReaderId imu_writer, color_writer;
int64_t steady() {return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::steady_clock::now().time_since_epoch()).count();}
int64_t wall() {return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::system_clock::now().time_since_epoch()).count();}
int topic_id(const rcl_subscription_t * subscription) {
  const char * name = rcl_subscription_get_topic_name(subscription);
  if (!name) {return 0;}
  if (std::strcmp(name, "/oak/imu/data") == 0) {return 1;}
  if (std::strcmp(name, "/oak/color/image_raw") == 0) {return 2;}
  return 0;
}
void gid(Row & row, const uint8_t * bytes) {
  // rmw_fastrtpsのGID先頭16 byteはGUID prefix(12)+entity(4)。
  // endianに依存しない2つの整数へ変換し、rcl takeとDDS受信を照合する。
  for (int i=0; i<8; ++i) {
    row.gid_hi=(row.gid_hi<<8)|bytes[i];
    row.gid_lo=(row.gid_lo<<8)|bytes[i+8];
  }
}
void gid(Row & row, const eprosima::fastrtps::rtps::GUID_t & value) {
  uint8_t bytes[16];
  std::memcpy(bytes, value.guidPrefix.value, 12);
  std::memcpy(bytes+12, value.entityId.value, 4);
  gid(row, bytes);
}
uint64_t sequence(const eprosima::fastrtps::rtps::SequenceNumber_t & value) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(value.high))<<32)|value.low;
}
bool dds_enabled() {
  static const bool enabled = [] {
    const char * value=std::getenv("PM_DDS_TRACE");
    return value && std::strcmp(value, "1")==0;
  }();
  return enabled;
}
void identify_reader(const rcl_subscription_t * subscription, int topic) {
  ReaderId * target=topic==1 ? &imu_reader : topic==2 ? &color_reader : nullptr;
  if (!target || target->hi.load(std::memory_order_acquire)!=0 || !dds_enabled()) {return;}
  const auto * handle=rcl_subscription_get_rmw_handle(subscription);
  // 実装固有の公開headerに従うため、別RMWの場合はcastせずDDS診断を行わない。
  if (!handle || !handle->implementation_identifier ||
    std::strcmp(handle->implementation_identifier,"rmw_fastrtps_cpp")!=0) {return;}
  const auto * info=static_cast<const CustomSubscriberInfo *>(handle->data);
  if (!info || !info->subscriber_) {return;}
  Row row{}; gid(row, info->subscriber_->getGuid());
  target->lo.store(row.gid_lo,std::memory_order_relaxed);
  target->hi.store(row.gid_hi,std::memory_order_release);
}
bool matches(const ReaderId & id, const Row & row) {
  const auto hi=id.hi.load(std::memory_order_acquire);
  return hi && hi==row.gid_hi && id.lo.load()==row.gid_lo;
}
bool target_reader(const eprosima::fastrtps::rtps::GUID_t & value,
                   const eprosima::fastrtps::rtps::GUID_t & writer) {
  if (!dds_enabled()) {return false;}
  Row reader_row{}, writer_row{}; gid(reader_row,value); gid(writer_row,writer);
  // readerを絞るだけでは対象外writerの配送試行も入る。購読とpublisherの
  // 両GUIDを照合し、画像fragmentが他topicの履歴を埋めることを避ける。
  return (matches(imu_reader,reader_row) && matches(imu_writer,writer_row)) ||
         (matches(color_reader,reader_row) && matches(color_writer,writer_row));
}
Row dds_row(int kind, const eprosima::fastrtps::rtps::CacheChange_t * change) {
  Row row{}; row.kind=kind; row.begin=steady(); row.wall=wall();
  row.tid=syscall(SYS_gettid);
  if (change) {
    gid(row, change->writerGUID); row.sequence=sequence(change->sequenceNumber);
    row.bytes=change->serializedPayload.length;
    row.source=change->sourceTimestamp.to_ns();
  }
  return row;
}
void append(Row row) {
  const size_t index = used.fetch_add(1, std::memory_order_relaxed);
  if (index < limit) {rows[index] = row;}
}
template<typename Function> Function original(const char * name) {
  // 名前解決は各hookの初回だけ。解決不能時に偽の成功を返すことはしない。
  auto result = reinterpret_cast<Function>(dlsym(RTLD_NEXT, name));
  if (!result) {_exit(126);}
  return result;
}
__attribute__((destructor)) void save() {
  const char * path = std::getenv("PM_RCL_TRACE_PATH");
  if (!path) {return;}
  // ros2 runの親Pythonにもpreloadは継承されるが、親がrclを呼ばなければ
  // 出力しない。子recorderの診断CSVを空ファイルで上書きさせない。
  const size_t total = used.load();
  if (total == 0) {return;}
  FILE * output = std::fopen(path, "w");
  if (!output) {std::perror("PM_RCL_TRACE_PATH"); return;}
  std::fprintf(output, "# total=%zu,omitted=%zu\n", total,
    total > limit ? total-limit : 0);
  std::fprintf(output, "wall_ns,begin_ns,end_ns,source_ns,received_ns,bytes,tid,kind,topic,result,ready,imu_ready,gid_hi,gid_lo,sequence,sequence_last,fragment\n");
  for (size_t i=0; i < total && i < limit; ++i) {
    const auto & r = rows[i];
    std::fprintf(output, "%lld,%lld,%lld,%lld,%lld,%llu,%ld,%d,%d,%d,%d,%d,%llu,%llu,%llu,%llu,%u\n",
      (long long)r.wall, (long long)r.begin, (long long)r.end,
      (long long)r.source, (long long)r.received, (unsigned long long)r.bytes,
      r.tid, r.kind, r.topic, r.result, r.ready, r.imu_ready,
      (unsigned long long)r.gid_hi, (unsigned long long)r.gid_lo,
      (unsigned long long)r.sequence, (unsigned long long)r.sequence_last, r.fragment);
  }
  std::fclose(output);
}
}

extern "C" rcl_ret_t rcl_wait(rcl_wait_set_t * set, int64_t timeout) {
  static auto call = original<decltype(&rcl_wait)>("rcl_wait");
  Row row{}; row.kind=1; row.begin=steady();
  const auto result = call(set, timeout);
  row.end=steady(); row.wall=wall(); row.result=result;
  row.tid=syscall(SYS_gettid);
  // rcl_waitの成功後、NULLでないsubscriptionは処理可能と通知されたもの。
  // これはDDSへの到着時刻そのものではなく、rclが返した可用状態である。
  if (result == RCL_RET_OK && set) {
    for (size_t i=0; i < set->size_of_subscriptions; ++i) {
      if (set->subscriptions[i]) {
        ++row.ready;
        if (topic_id(set->subscriptions[i]) == 1) {row.imu_ready=1;}
      }
    }
  }
  append(row);
  return result;
}

extern "C" rcl_ret_t rcl_take_serialized_message(
  const rcl_subscription_t * subscription, rcl_serialized_message_t * message,
  rmw_message_info_t * info, rmw_subscription_allocation_t * allocation) {
  static auto call = original<decltype(&rcl_take_serialized_message)>(
    "rcl_take_serialized_message");
  Row row{}; row.kind=2; row.topic=topic_id(subscription); row.begin=steady();
  identify_reader(subscription,row.topic);
  const auto result=call(subscription, message, info, allocation);
  row.end=steady(); row.wall=wall(); row.result=result;
  row.tid=syscall(SYS_gettid);
  // 未成功時のmessage/infoは読まない。RMWがtimestampを未提供なら0のまま
  // 出力し、解析側で「到着時刻が測れた」と誤認しないようにする。
  if (result == RCL_RET_OK) {
    row.bytes=message ? message->buffer_length : 0;
    row.source=info ? info->source_timestamp : 0;
    row.received=info ? info->received_timestamp : 0;
    if (info) {gid(row, info->publisher_gid.data);}
    ReaderId * writer=row.topic==1 ? &imu_writer : row.topic==2 ? &color_writer : nullptr;
    if (writer && writer->hi.load()==0) {
      writer->lo.store(row.gid_lo,std::memory_order_relaxed);
      writer->hi.store(row.gid_hi,std::memory_order_release);
    }
  }
  append(row);
  return result;
}

namespace eprosima { namespace fastrtps { namespace rtps {
// FastDDS 2.1.4の公開class/headerと照合した試験用hook。
// DDS reader入口はtransport搬送後・reader内部処理前であり、network packetの
// 最初の到着時刻ではない。payload内容やhistoryを変更せず、元関数を1回だけ呼ぶ。
bool StatefulReader::processDataMsg(CacheChange_t * change) {
  using Function=bool(*)(StatefulReader *, CacheChange_t *);
  static auto call=original<Function>(
    "_ZN8eprosima8fastrtps4rtps14StatefulReader14processDataMsgEPNS1_13CacheChange_tE");
  if (!change || !target_reader(getGuid(),change->writerGUID)) {return call(this, change);}
  auto row=dds_row(3, change);
  const bool result=call(this, change);
  row.end=steady(); row.result=result; append(row);
  // changeは元関数内で回収され得る。戻った後には再参照しない。
  return result;
}
bool StatefulReader::processDataFragMsg(CacheChange_t * change, uint32_t size,
  uint32_t first, uint16_t count) {
  using Function=bool(*)(StatefulReader *, CacheChange_t *, uint32_t, uint32_t, uint16_t);
  static auto call=original<Function>(
    "_ZN8eprosima8fastrtps4rtps14StatefulReader18processDataFragMsgEPNS1_13CacheChange_tEjjt");
  if (!change || !target_reader(getGuid(),change->writerGUID)) {return call(this, change, size, first, count);}
  auto row=dds_row(4, change); row.bytes=size; row.fragment=first;
  const bool result=call(this, change, size, first, count);
  row.end=steady(); row.result=result; append(row);
  return result;
}
bool StatefulReader::processHeartbeatMsg(const GUID_t & writer, uint32_t count,
  const SequenceNumber_t & first, const SequenceNumber_t & last, bool final, bool liveliness) {
  using Function=bool(*)(StatefulReader *, const GUID_t &, uint32_t,
    const SequenceNumber_t &, const SequenceNumber_t &, bool, bool);
  static auto call=original<Function>(
    "_ZN8eprosima8fastrtps4rtps14StatefulReader19processHeartbeatMsgERKNS1_6GUID_tEjRKNS1_16SequenceNumber_tES8_bb");
  if (!target_reader(getGuid(),writer)) {return call(this, writer, count, first, last, final, liveliness);}
  auto row=dds_row(5, nullptr); gid(row, writer);
  row.sequence=sequence(first); row.sequence_last=sequence(last);
  const bool result=call(this, writer, count, first, last, final, liveliness);
  row.end=steady(); row.result=result; append(row);
  return result;
}
}}}
