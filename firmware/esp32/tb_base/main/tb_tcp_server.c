#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

static const char *TAG = "tb_tcp";

#define TB_MAGIC0 0x54  // 'T'
#define TB_MAGIC1 0x42  // 'B'
#define TB_HDR_LEN 10

typedef enum {
  TB_MSG_CMD_VEL       = 0x01,
  TB_MSG_SET_TELEMETRY = 0x02,

  //For Heartbeat
  TB_MSG_HEARTBEAT = 0x10,
  TB_MSG_ACK       = 0x11,
} tb_msg_type_t;

static int send_all(int sock, const uint8_t *data, size_t n) {
  size_t sent = 0;
  while (sent < n) {
    int r = send(sock, data + sent, n - sent, 0);
    if (r <= 0) return -1;
    sent += (size_t)r;
  }
  return 0;
}

static void wr16_le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32_le(uint8_t *p, uint32_t v) {
  p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}

static int send_frame(int sock, uint8_t type, uint8_t flags, uint32_t seq,
                      const uint8_t *payload, uint16_t len)
{
  uint8_t hdr[TB_HDR_LEN];
  hdr[0] = TB_MAGIC0;
  hdr[1] = TB_MAGIC1;
  hdr[2] = type;
  hdr[3] = flags;
  wr16_le(&hdr[4], len);
  wr32_le(&hdr[6], seq);

  if (send_all(sock, hdr, sizeof(hdr)) < 0) return -1;
  if (len && payload) {
    if (send_all(sock, payload, len) < 0) return -1;
  }
  return 0;
}


static inline uint16_t rd16_le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd32_le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool find_magic(const uint8_t *buf, size_t n, size_t *idx_out) {
  for (size_t i = 0; i + 1 < n; i++) {
    if (buf[i] == TB_MAGIC0 && buf[i+1] == TB_MAGIC1) {
      *idx_out = i;
      return true;
    }
  }
  return false;
}

typedef struct __attribute__((packed)) {
  uint8_t enable_imu;
  float   imu_rate_hz;
  uint8_t enable_ir;
  float   ir_rate_hz;
  uint8_t enable_motor;
  float   motor_rate_hz;
  uint8_t enable_batt;
  float   batt_rate_hz;
  uint8_t enable_lidar;
  float   lidar_rate_hz;
} tb_cfg_t;

static void handle_frame(int sock, uint8_t type, uint32_t seq, const uint8_t *payload, uint16_t len)
{
  if (type == TB_MSG_HEARTBEAT) {
    // ACK payload: echoed seq (4 bytes)
    uint8_t ack_payload[4];
    wr32_le(ack_payload, seq);
    (void)send_frame(sock, TB_MSG_ACK, 0, seq, ack_payload, sizeof(ack_payload));
    return;
  }

  if (type == TB_MSG_CMD_VEL) {
    if (len != 24) {
      ESP_LOGW(TAG, "CMD_VEL len=%u (expected 24)", (unsigned)len);
      return;
    }
    float v[6];
    memcpy(v, payload, sizeof(v));
    float vx = v[0];
    float wz = v[5];
    ESP_LOGI(TAG, "CMD_VEL seq=%lu vx=%.3f wz=%.3f", (unsigned long)seq, (double)vx, (double)wz);
    return;
  }

  if (type == TB_MSG_SET_TELEMETRY) {
    if (len != sizeof(tb_cfg_t)) {
      ESP_LOGW(TAG, "SET_TELEMETRY len=%u (expected %u)", (unsigned)len, (unsigned)sizeof(tb_cfg_t));
      return;
    }
    tb_cfg_t cfg;
    memcpy(&cfg, payload, sizeof(cfg));
    ESP_LOGI(TAG,
      "CFG seq=%lu imu=%u@%.1f ir=%u@%.1f motor=%u@%.1f batt=%u@%.1f lidar=%u@%.1f",
      (unsigned long)seq,
      (unsigned)cfg.enable_imu, (double)cfg.imu_rate_hz,
      (unsigned)cfg.enable_ir, (double)cfg.ir_rate_hz,
      (unsigned)cfg.enable_motor, (double)cfg.motor_rate_hz,
      (unsigned)cfg.enable_batt, (double)cfg.batt_rate_hz,
      (unsigned)cfg.enable_lidar, (double)cfg.lidar_rate_hz
    );
    return;
  }

  ESP_LOGI(TAG, "frame type=0x%02x seq=%lu len=%u", (unsigned)type, (unsigned long)seq, (unsigned)len);
}

void tcp_server_task(void *arg)
{
  (void)arg;

  const int port = CONFIG_TB_TCP_PORT;

  int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed");
    vTaskDelete(NULL);
    return;
  }

  int yes = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "bind() failed");
    close(listen_fd);
    vTaskDelete(NULL);
    return;
  }

  if (listen(listen_fd, 1) < 0) {
    ESP_LOGE(TAG, "listen() failed");
    close(listen_fd);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "TCP server listening on port %d", port);

  static uint8_t stream[16 * 1024];
  size_t stream_len = 0;

  while (1) {
    struct sockaddr_in6 client_addr;
    socklen_t client_len = sizeof(client_addr);
    int sock = accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "accept() failed");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    ESP_LOGI(TAG, "client connected");
    stream_len = 0;

    while (1) {
      uint8_t rx[2048];
      int r = recv(sock, rx, sizeof(rx), 0);
      if (r <= 0) {
        ESP_LOGW(TAG, "client disconnected");
        break;
      }

      if (stream_len + (size_t)r > sizeof(stream)) {
        // keep last 1KB
        size_t keep = 1024;
        if (stream_len > keep) {
          memmove(stream, stream + (stream_len - keep), keep);
          stream_len = keep;
        } else {
          stream_len = 0;
        }
      }

      memcpy(stream + stream_len, rx, (size_t)r);
      stream_len += (size_t)r;

      while (1) {
        if (stream_len < TB_HDR_LEN) break;

        size_t m = 0;
        if (!find_magic(stream, stream_len, &m)) {
          if (stream_len > 1) {
            stream[0] = stream[stream_len - 1];
            stream_len = 1;
          }
          break;
        }
        if (m > 0) {
          memmove(stream, stream + m, stream_len - m);
          stream_len -= m;
          if (stream_len < TB_HDR_LEN) break;
        }

        uint8_t type = stream[2];
        uint16_t len = rd16_le(&stream[4]);
        uint32_t seq = rd32_le(&stream[6]);

        size_t total = TB_HDR_LEN + (size_t)len;
        if (stream_len < total) break;

        handle_frame(sock, type, seq, &stream[TB_HDR_LEN], len);

        memmove(stream, stream + total, stream_len - total);
        stream_len -= total;
      }
    }

    shutdown(sock, SHUT_RDWR);
    close(sock);
  }
}
