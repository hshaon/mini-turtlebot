#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

#include "tb_bridge_cpp/framing.hpp"

namespace tb_bridge_cpp
{

class TcpTransport
{
public:
  using FrameHandler = std::function<void(const Frame&)>;

  TcpTransport() = default;
  ~TcpTransport();

  bool start(const std::string& host, int port, FrameHandler cb);
  void stop();

  bool send_bytes(const uint8_t* data, size_t n);

  bool is_connected() const { return connected_.load(); }

private:
  void io_thread_(std::string host, int port);

  int sock_{-1};
  std::thread th_;
  std::atomic<bool> run_{false};
  std::atomic<bool> connected_{false};

  std::mutex send_mtx_;
  FrameHandler cb_;
  FrameDecoder decoder_;
};

}  // namespace tb_bridge_cpp
