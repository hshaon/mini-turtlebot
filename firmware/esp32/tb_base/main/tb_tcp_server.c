#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "lwip/tcp.h"

#include "tb_motors.h"
#include "tb_oled.h"
#include "tb_wifi.h"
#include "tb_imu.h"
#include "tb_ir.h"
#include "tb_tof.h"
#include "tb_lidar.h"
static const char *TAG = "tb_tcp";

#define TB_MAGIC0 0x54  // 'T'
#define TB_MAGIC1 0x42  // 'B'
#define TB_HDR_LEN 10

// Simple bring-up scaling for CMD_VEL (tune for your robot).
#define TB_MAX_VX_MPS  0.25f
#define TB_MAX_WZ_RPS  1.0f

typedef enum {
  TB_MSG_CMD_VEL       = 0x01,
  TB_MSG_SET_TELEMETRY = 0x02,
  TB_MSG_SET_ID        = 0x03,

  //For Heartbeat
  TB_MSG_HEARTBEAT = 0x10,
  TB_MSG_ACK       = 0x11,

  TB_MSG_IMU        = 0x30,
  TB_MSG_IR         = 0x31,
  TB_MSG_TOF        = 0x32,
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

static char s_robot_id[16] = "---";

static bool find_magic(const uint8_t *buf, size_t n, size_t *idx_out) {
  for (size_t i = 0; i + 1 < n; i++) {
    if (buf[i] == TB_MAGIC0 && buf[i+1] == TB_MAGIC1) {
      *idx_out = i;
      return true;
    }
  }
  return false;
}

static float clamp_rate(float hz, float min_hz)
{
  if (hz < min_hz) return min_hz;
  return hz;
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
  uint8_t enable_tof;
  float   tof_rate_hz;
  uint8_t enable_lidar;
  float   lidar_rate_hz;
  uint16_t lidar_udp_port;
} tb_cfg_t;

#define TB_MIN_IMU_HZ 50.0f
#define TB_MIN_IR_HZ  10.0f
#define TB_MIN_TOF_HZ 10.0f
#define TB_MIN_LIDAR_HZ 1.0f
#define TB_LIDAR_SWEEP_HZ 10.0f
#define TB_LIDAR_RANGE_MIN_MM 50
#define TB_LIDAR_RANGE_MAX_MM 12000
#define TB_LIDAR_MAGIC 0x4C44

static tb_cfg_t g_cfg = {
  .enable_imu = 1,
  .imu_rate_hz = TB_MIN_IMU_HZ,
  .enable_ir = 1,
  .ir_rate_hz = TB_MIN_IR_HZ,
  .enable_motor = 0,
  .motor_rate_hz = 0.0f,
  .enable_batt = 0,
  .batt_rate_hz = 0.0f,
  .enable_tof = 1,
  .tof_rate_hz = TB_MIN_TOF_HZ,
  .enable_lidar = 0,
  .lidar_rate_hz = TB_MIN_LIDAR_HZ,
  .lidar_udp_port = 5601,
};
static TickType_t g_last_imu = 0;
static TickType_t g_last_ir = 0;
static TickType_t g_last_tof = 0;
static uint32_t g_lidar_sweep_counter = 0;

static int lidar_udp_sock = -1;
static struct sockaddr_in lidar_peer;
static bool lidar_peer_valid = false;

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t seq;
  uint16_t count;
  uint16_t range_min_mm;
  uint16_t range_max_mm;
} tb_lidar_udp_hdr_t;

static void lidar_peer_set_port(uint16_t port)
{
  if (!lidar_peer_valid) return;
  lidar_peer.sin_port = htons(port);
}

static bool lidar_udp_send_sweep(uint16_t seq, const uint16_t *ranges_mm, size_t count)
{
  if (!ranges_mm || count == 0) return false;
  if (!lidar_peer_valid || lidar_udp_sock < 0) return true;

  uint8_t buf[10 + TB_LIDAR_POINTS * 2];
  size_t expected = 10 + count * 2;
  if (expected > sizeof(buf)) return false;

  tb_lidar_udp_hdr_t hdr;
  hdr.magic = TB_LIDAR_MAGIC;
  hdr.seq = seq;
  hdr.count = (uint16_t)count;
  hdr.range_min_mm = TB_LIDAR_RANGE_MIN_MM;
  hdr.range_max_mm = TB_LIDAR_RANGE_MAX_MM;

  memcpy(buf, &hdr, sizeof(hdr));
  for (size_t i = 0; i < count; ++i) {
    uint16_t v = ranges_mm[i];
    buf[10 + i * 2] = (uint8_t)(v & 0xFF);
    buf[10 + i * 2 + 1] = (uint8_t)(v >> 8);
  }

  int r = sendto(lidar_udp_sock, buf, expected, 0, (struct sockaddr*)&lidar_peer, sizeof(lidar_peer));
  return r == (int)expected;
}

static void handle_frame(int sock, uint8_t type, uint32_t seq, const uint8_t *payload, uint16_t len)
{
  if (type == TB_MSG_HEARTBEAT) {
    // ACK payload is empty (seq is already in header).
    (void)send_frame(sock, TB_MSG_ACK, 0, seq, NULL, 0);
    return;
  }

  if (type == TB_MSG_SET_ID) {
    size_t n = (len < (sizeof(s_robot_id) - 1)) ? len : (sizeof(s_robot_id) - 1);
    memcpy(s_robot_id, payload, n);
    s_robot_id[n] = '\0';
    ESP_LOGI(TAG, "SET_ID: %s", s_robot_id);
    tb_oled_show_status(s_robot_id, CONFIG_TB_WIFI_SSID, tb_wifi_get_ip_str());
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

    float vx_n = (TB_MAX_VX_MPS > 0.0f) ? (vx / TB_MAX_VX_MPS) : 0.0f;
    float wz_n = (TB_MAX_WZ_RPS > 0.0f) ? (wz / TB_MAX_WZ_RPS) : 0.0f;

    float speed_l = vx_n - wz_n;
    float speed_r = vx_n + wz_n;

    tb_motors_set(speed_l, speed_r);
    return;
  }

  if (type == TB_MSG_SET_TELEMETRY) {
    if (len != sizeof(tb_cfg_t)) {
      ESP_LOGW(TAG, "SET_TELEMETRY len=%u (expected %u)", (unsigned)len, (unsigned)sizeof(tb_cfg_t));
      return;
    }
    tb_cfg_t cfg;
    memcpy(&cfg, payload, sizeof(cfg));
    g_cfg = cfg;
    if (g_cfg.enable_imu) g_cfg.imu_rate_hz = clamp_rate(g_cfg.imu_rate_hz, TB_MIN_IMU_HZ);
    if (g_cfg.enable_ir) g_cfg.ir_rate_hz = clamp_rate(g_cfg.ir_rate_hz, TB_MIN_IR_HZ);
    if (g_cfg.enable_tof) g_cfg.tof_rate_hz = clamp_rate(g_cfg.tof_rate_hz, TB_MIN_TOF_HZ);
    if (g_cfg.enable_lidar) g_cfg.lidar_rate_hz = clamp_rate(g_cfg.lidar_rate_hz, TB_MIN_LIDAR_HZ);
    if (g_cfg.lidar_udp_port == 0) g_cfg.lidar_udp_port = 5601;
    lidar_peer_set_port(g_cfg.lidar_udp_port);

    ESP_LOGI(TAG,
      "CFG seq=%lu imu=%u@%.1f ir=%u@%.1f motor=%u@%.1f batt=%u@%.1f tof=%u@%.1f lidar=%u@%.1f udp=%u",
      (unsigned long)seq,
      (unsigned)g_cfg.enable_imu, (double)g_cfg.imu_rate_hz,
      (unsigned)g_cfg.enable_ir, (double)g_cfg.ir_rate_hz,
      (unsigned)g_cfg.enable_motor, (double)g_cfg.motor_rate_hz,
      (unsigned)g_cfg.enable_batt, (double)g_cfg.batt_rate_hz,
      (unsigned)g_cfg.enable_tof, (double)g_cfg.tof_rate_hz,
      (unsigned)g_cfg.enable_lidar, (double)g_cfg.lidar_rate_hz,
      (unsigned)g_cfg.lidar_udp_port
    );
    return;
  }

  ESP_LOGI(TAG, "frame type=0x%02x seq=%lu len=%u", (unsigned)type, (unsigned long)seq, (unsigned)len);
}

static bool telemetry_tick(int sock, uint32_t *seq)
{
  if (!seq) return true;
  const TickType_t now = xTaskGetTickCount();

  if (g_cfg.enable_imu && g_cfg.imu_rate_hz > 0.0f && tb_imu_is_ready()) {
    TickType_t interval = pdMS_TO_TICKS((int)(1000.0f / g_cfg.imu_rate_hz));
    if (interval == 0) interval = 1;
    if ((now - g_last_imu) >= interval) {
      tb_imu_raw_t imu;
      if (tb_imu_read(&imu) == ESP_OK) {
        float v[6] = {imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz};
        if (send_frame(sock, TB_MSG_IMU, 0, (*seq)++, (const uint8_t*)v, sizeof(v)) < 0) return false;
      }
      g_last_imu = now;
    }
  }

  if (g_cfg.enable_ir && g_cfg.ir_rate_hz > 0.0f) {
    TickType_t interval = pdMS_TO_TICKS((int)(1000.0f / g_cfg.ir_rate_hz));
    if (interval == 0) interval = 1;
    if ((now - g_last_ir) >= interval) {
      float l = 0.0f;
      float r = 0.0f;
      if (tb_ir_read(&l, &r) == ESP_OK) {
        float v[2] = {l, r};
        if (send_frame(sock, TB_MSG_IR, 0, (*seq)++, (const uint8_t*)v, sizeof(v)) < 0) return false;
      }
      g_last_ir = now;
    }
  }

  if (g_cfg.enable_tof && g_cfg.tof_rate_hz > 0.0f && tb_tof_is_ready()) {
    TickType_t interval = pdMS_TO_TICKS((int)(1000.0f / g_cfg.tof_rate_hz));
    if (interval == 0) interval = 1;
    if ((now - g_last_tof) >= interval) {
      float range_m = 0.0f;
      if (tb_tof_read(&range_m) == ESP_OK) {
        if (send_frame(sock, TB_MSG_TOF, 0, (*seq)++, (const uint8_t*)&range_m, sizeof(range_m)) < 0) return false;
      }
      g_last_tof = now;
    }
  }

  if (g_cfg.enable_lidar && g_cfg.lidar_rate_hz > 0.0f) {
    uint16_t ranges[TB_LIDAR_POINTS];
    uint32_t sweep_seq = 0;
    if (tb_lidar_pop_sweep(ranges, TB_LIDAR_POINTS, &sweep_seq)) {
      g_lidar_sweep_counter++;
      float hz = g_cfg.lidar_rate_hz;
      if (hz > TB_LIDAR_SWEEP_HZ) hz = TB_LIDAR_SWEEP_HZ;
      int send_every = (hz <= 0.0f) ? 0 : (int)lroundf(TB_LIDAR_SWEEP_HZ / hz);
      if (send_every < 1) send_every = 1;
      if ((g_lidar_sweep_counter % (uint32_t)send_every) == 0) {
        if (!lidar_udp_send_sweep((uint16_t)sweep_seq, ranges, TB_LIDAR_POINTS)) return false;
      }
    }
  }

  return true;
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
    struct sockaddr_storage client_addr;
    socklen_t client_len = sizeof(client_addr);
    int sock = accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "accept() failed");
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "client connected");
    if (client_addr.ss_family == AF_INET) {
      struct sockaddr_in *in = (struct sockaddr_in*)&client_addr;
      lidar_peer = *in;
      lidar_peer.sin_port = htons(g_cfg.lidar_udp_port);
      lidar_peer_valid = true;
      if (lidar_udp_sock < 0) {
        lidar_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
      }
    } else {
      lidar_peer_valid = false;
    }

    stream_len = 0;
    uint32_t tx_seq = 1;

    while (1) {
      uint8_t rx[2048];
      int r = recv(sock, rx, sizeof(rx), 0);
      if (r == 0) {
        ESP_LOGW(TAG, "client disconnected");
        strcpy(s_robot_id, "---");
        tb_oled_show_status(s_robot_id, CONFIG_TB_WIFI_SSID, tb_wifi_get_ip_str());
        break;
      }
      if (r < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          ESP_LOGW(TAG, "client recv error");
          strcpy(s_robot_id, "---");
          tb_oled_show_status(s_robot_id, CONFIG_TB_WIFI_SSID, tb_wifi_get_ip_str());
          break;
        }
        r = 0;
      }

      if (r > 0 && stream_len + (size_t)r > sizeof(stream)) {
        // keep last 1KB
        size_t keep = 1024;
        if (stream_len > keep) {
          memmove(stream, stream + (stream_len - keep), keep);
          stream_len = keep;
        } else {
          stream_len = 0;
        }
      }

      if (r > 0) {
        memcpy(stream + stream_len, rx, (size_t)r);
        stream_len += (size_t)r;
      }

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

      if (!telemetry_tick(sock, &tx_seq)) {
        ESP_LOGW(TAG, "telemetry send failed");
        strcpy(s_robot_id, "---");
        tb_oled_show_status(s_robot_id, CONFIG_TB_WIFI_SSID, tb_wifi_get_ip_str());
        break;
      }
    }

    shutdown(sock, SHUT_RDWR);
    close(sock);
    if (lidar_udp_sock >= 0) {
      close(lidar_udp_sock);
      lidar_udp_sock = -1;
    }
    lidar_peer_valid = false;
  }
}
