#include "tb_bridge_cpp/framing.hpp"
#include <cstring>

namespace tb_bridge_cpp
{
static inline void le16(uint8_t* p, uint16_t v) { p[0]=uint8_t(v); p[1]=uint8_t(v>>8); }
static inline void le32(uint8_t* p, uint32_t v) { p[0]=uint8_t(v); p[1]=uint8_t(v>>8); p[2]=uint8_t(v>>16); p[3]=uint8_t(v>>24); }
static inline uint16_t rd16(const uint8_t* p){ return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
static inline uint32_t rd32(const uint8_t* p){ return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24); }

std::vector<uint8_t> encode_frame(MsgType type, uint8_t flags, uint32_t seq,
                                  const uint8_t* payload, uint16_t len)
{
  // header = 2(magic)+1(type)+1(flags)+2(len)+4(seq) = 10 bytes
  std::vector<uint8_t> out;
  out.resize(10 + len);
  out[0] = 0x54; // 'T'
  out[1] = 0x42; // 'B'
  out[2] = static_cast<uint8_t>(type);
  out[3] = flags;
  le16(&out[4], len);
  le32(&out[6], seq);
  if (len && payload) {
    std::memcpy(&out[10], payload, len);
  }
  return out;
}

size_t FrameDecoder::find_magic_() const
{
  for (size_t i = 0; i + 1 < buf_.size(); ++i) {
    if (buf_[i] == 0x54 && buf_[i+1] == 0x42) return i;
  }
  return std::string::npos;
}

void FrameDecoder::feed(const uint8_t* data, size_t n)
{
  if (!data || n == 0) return;
  buf_.insert(buf_.end(), data, data + n);

  // Keep buffer from growing forever if noise occurs
  if (buf_.size() > 1024 * 1024) {
    buf_.erase(buf_.begin(), buf_.begin() + (buf_.size() - 64 * 1024));
  }
}

bool FrameDecoder::pop(Frame& out)
{
  // Need at least header
  while (true) {
    if (buf_.size() < 10) return false;

    size_t m = find_magic_();
    if (m == std::string::npos) {
      // keep last byte just in case it's 'T'
      buf_.erase(buf_.begin(), buf_.end() - 1);
      return false;
    }
    if (m > 0) {
      buf_.erase(buf_.begin(), buf_.begin() + m);
      if (buf_.size() < 10) return false;
    }

    uint8_t type = buf_[2];
    uint8_t flags = buf_[3];
    uint16_t len = rd16(&buf_[4]);
    uint32_t seq = rd32(&buf_[6]);

    size_t total = 10 + size_t(len);
    if (buf_.size() < total) return false;

    out.type = static_cast<MsgType>(type);
    out.flags = flags;
    out.len = len;
    out.seq = seq;
    out.payload.assign(buf_.begin() + 10, buf_.begin() + total);

    buf_.erase(buf_.begin(), buf_.begin() + total);
    return true;
  }
}

}  // namespace tb_bridge_cpp
