#include "tb_bridge_cpp/transport_tcp.hpp"

#include <cstring>
#include <chrono>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>

namespace tb_bridge_cpp
{

TcpTransport::~TcpTransport()
{
  stop();
}

static int connect_tcp(const std::string& host, int port)
{
  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* res = nullptr;
  const std::string port_str = std::to_string(port);
  int rc = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res);
  if (rc != 0 || !res) return -1;

  int s = -1;
  for (auto p = res; p; p = p->ai_next) {
    s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (s < 0) continue;
    if (::connect(s, p->ai_addr, p->ai_addrlen) == 0) {
      int one = 1;
      (void)::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      break; // success
    }
    ::close(s);
    s = -1;
  }

  freeaddrinfo(res);
  return s;
}

bool TcpTransport::start(const std::string& host, int port, FrameHandler cb)
{
  if (run_.load()) return false;
  cb_ = std::move(cb);
  run_.store(true);
  th_ = std::thread([this, host, port]() { io_thread_(host, port); });
  return true;
}

void TcpTransport::stop()
{
  run_.store(false);

  if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    sock_ = -1;
  }

  if (th_.joinable()) th_.join();
  connected_.store(false);
}

bool TcpTransport::send_bytes(const uint8_t* data, size_t n)
{
  if (!connected_.load() || sock_ < 0) return false;
  if (!data || n == 0) return true;

  std::lock_guard<std::mutex> lk(send_mtx_);
  size_t sent = 0;
  while (sent < n) {
    ssize_t rc = ::send(sock_, data + sent, n - sent, 0);
    if (rc <= 0) return false;
    sent += size_t(rc);
  }
  return true;
}

void TcpTransport::io_thread_(std::string host, int port)
{
  using namespace std::chrono_literals;

  while (run_.load()) {
    sock_ = connect_tcp(host, port);
    if (sock_ < 0) {
      connected_.store(false);
      std::this_thread::sleep_for(500ms);
      continue;
    }

    connected_.store(true);

    uint8_t buf[4096];
    while (run_.load()) {
      ssize_t rc = ::recv(sock_, buf, sizeof(buf), 0);
      if (rc <= 0) break;

      decoder_.feed(buf, size_t(rc));
      Frame f;
      while (decoder_.pop(f)) {
        if (cb_) cb_(f);
      }
    }

    connected_.store(false);
    if (sock_ >= 0) {
      ::shutdown(sock_, SHUT_RDWR);
      ::close(sock_);
      sock_ = -1;
    }

    // retry
    std::this_thread::sleep_for(250ms);
  }
}

}  // namespace tb_bridge_cpp
