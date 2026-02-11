#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace tb_bridge_cpp
{
enum class MsgType : uint8_t
{
  CMD_VEL = 0x01,
  SET_ID = 0x03,
  SET_TELEMETRY = 0x02,
  HEARTBEAT = 0x10,
  ACK = 0x11,
  TIME_SYNC = 0x12,
  METRICS_CFG = 0x13,
  CMD_VEL_ACK = 0x14,
  TEXT_LOG = 0x20,
  IMU = 0x30,
  IR = 0x31,
  TOF = 0x32,
  IMU_TS = 0x40,
  IR_TS = 0x41,
  TOF_TS = 0x42,
};

struct Frame
{
  MsgType type{MsgType::TEXT_LOG};
  uint8_t flags{0};
  uint16_t len{0};
  uint32_t seq{0};
  std::vector<uint8_t> payload;
};

// Encode a frame to bytes (no CRC yet)
std::vector<uint8_t> encode_frame(MsgType type, uint8_t flags, uint32_t seq,
                                  const uint8_t* payload, uint16_t len);

// Incremental decoder: feed bytes, returns any complete frames extracted.
class FrameDecoder
{
public:
  void feed(const uint8_t* data, size_t n);
  bool pop(Frame& out);

private:
  std::vector<uint8_t> buf_;
  size_t find_magic_() const;
};

}  // namespace tb_bridge_cpp
