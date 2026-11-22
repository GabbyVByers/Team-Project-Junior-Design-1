
/*
 *   Source [main.c]
 */

#define IMU_DEMO
#ifdef IMU_DEMO

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "imu.h"

#define PIN_LED_CALIBRATING GPIO_NUM_4
#define PIN_LED_READY       GPIO_NUM_2

#define SETTLE_S      1.0f   // let the sensor fusion converge before calibrating
#define CALIBRATE_S   2.0f   // device at rest; averaged into the baseline
#define PRINT_EVERY   10     // samples (10 Hz at the IMU's 100 Hz)

// Stationary detection (zero-velocity update)
#define STILL_ACCEL   0.2f   // m/s^2
#define STILL_RATE    0.1f   // rad/s
#define STILL_TIME    0.2f   // s both must hold before the device counts as stationary
#define BIAS_TRACKING 0.02f  // per-sample weight for re-estimating accel bias while stationary

typedef struct { float w, x, y, z; } quat_t;
typedef struct { float x, y, z; } vec3_t;

static quat_t qmul(quat_t a, quat_t b)
{
  return (quat_t){
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

static quat_t qconj(quat_t q) { return (quat_t){ q.w, -q.x, -q.y, -q.z }; }

static vec3_t vadd(vec3_t a, vec3_t b)    { return (vec3_t){ a.x + b.x, a.y + b.y, a.z + b.z }; }
static vec3_t vsub(vec3_t a, vec3_t b)    { return (vec3_t){ a.x - b.x, a.y - b.y, a.z - b.z }; }
static vec3_t vscale(vec3_t a, float s)   { return (vec3_t){ a.x * s, a.y * s, a.z * s }; }
static float  vlen(vec3_t a)              { return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z); }

// q * v * q^-1
static vec3_t rotate(quat_t q, vec3_t v)
{
  quat_t r = qmul(qmul(q, (quat_t){ 0, v.x, v.y, v.z }), qconj(q));
  return (vec3_t){ r.x, r.y, r.z };
}

static imu_sample_t next_sample(void)
{
  imu_sample_t s;
  while (!imu_read(&s)) {
    vTaskDelay(1);
  }
  return s;
}

void app_main(void)
{
  gpio_config_t led_cfg = {
    .pin_bit_mask = (1ULL << PIN_LED_CALIBRATING) | (1ULL << PIN_LED_READY),
    .mode = GPIO_MODE_OUTPUT,
  };
  gpio_config(&led_cfg);
  gpio_set_level(PIN_LED_CALIBRATING, 1);
  gpio_set_level(PIN_LED_READY, 0);
  
  imu_init();
  
  for (float t = 0; t < SETTLE_S; ) {
    t += next_sample().dt;
  }
  
  // Calibration: average orientation and residual acceleration (the sensor's bias) at rest
  quat_t q_sum = { 0 };
  vec3_t a_sum = { 0 };
  int n = 0;
  
  for (float t = 0; t < CALIBRATE_S; n++) {
    imu_sample_t s = next_sample();
    t += s.dt;
    
    // q and -q are the same rotation; flip to agree with the running sum before adding
    float sign = (q_sum.w * s.w + q_sum.x * s.x + q_sum.y * s.y + q_sum.z * s.z < 0) ? -1 : 1;
    q_sum = (quat_t){ q_sum.w + sign * s.w, q_sum.x + sign * s.x, q_sum.y + sign * s.y, q_sum.z + sign * s.z };
    a_sum = vadd(a_sum, (vec3_t){ s.ax, s.ay, s.az });
  }
  
  float norm = sqrtf(q_sum.w * q_sum.w + q_sum.x * q_sum.x + q_sum.y * q_sum.y + q_sum.z * q_sum.z);
  quat_t q_base = { q_sum.w / norm, q_sum.x / norm, q_sum.y / norm, q_sum.z / norm };
  quat_t q_base_inv = qconj(q_base);
  vec3_t bias = vscale(a_sum, 1.0f / n);
  
  gpio_set_level(PIN_LED_CALIBRATING, 0);
  gpio_set_level(PIN_LED_READY, 1);
  
  // Tracking: position is expressed in the device's frame as it was during calibration
  vec3_t pos = { 0 }, vel = { 0 }, acc_prev = { 0 };
  quat_t q_prev = q_base;
  float still_time = 0, motion_time = 0;
  bool moving = false;
  
  for (int i = 1; ; i++) {
    imu_sample_t s = next_sample();
    quat_t q = { s.w, s.x, s.y, s.z };
    vec3_t a = vsub((vec3_t){ s.ax, s.ay, s.az }, bias);
    
    // Angular rate from the change in orientation since the last sample
    quat_t dq = qmul(qconj(q_prev), q);
    float rate = 2.0f * sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z) / s.dt;
    q_prev = q;
    
    still_time = (vlen(a) < STILL_ACCEL && rate < STILL_RATE) ? still_time + s.dt : 0;
    
    // Orientation change since calibration (current device frame -> baseline frame)
    quat_t q_rel = qmul(q_base_inv, q);
    
    if (still_time >= STILL_TIME) {
      if (moving) {
        // The true velocity is zero now, so whatever is left is integration error. Bias
        // error grows velocity error linearly, so the position picked up half of it
        // times the length of the motion; take that back out.
        pos = vsub(pos, vscale(vel, 0.5f * motion_time));
        moving = false;
      }
      vel = acc_prev = (vec3_t){ 0 };
      
      // At rest the measured linear acceleration is pure bias; track it as it drifts
      bias = vadd(bias, vscale(a, BIAS_TRACKING));
    } else {
      if (!moving) {
        moving = true;
        motion_time = 0;
      }
      motion_time += s.dt;
      
      // Trapezoidal integration in the baseline frame
      vec3_t acc = rotate(q_rel, a);
      vec3_t vel_next = vadd(vel, vscale(vadd(acc_prev, acc), 0.5f * s.dt));
      pos = vadd(pos, vscale(vadd(vel, vel_next), 0.5f * s.dt));
      vel = vel_next;
      acc_prev = acc;
    }
    
    if (i % PRINT_EVERY == 0) {
      printf("Pos: x=%.3f y=%.3f z=%.3f m | dQuat: w=%.4f x=%.4f y=%.4f z=%.4f\n",
        pos.x, pos.y, pos.z, q_rel.w, q_rel.x, q_rel.y, q_rel.z);
    }
  }
}

#endif /* IMU_DEMO */



// #define LIDAR_DEMO
#ifdef LIDAR_DEMO

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lidar.h"

void app_main(void)
{
  lidar_init();
  
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("Distance: %d cm\n", lidar_read().distance_cm);
  }
}

#endif /* LIDAR_DEMO */



//#define WEB_DEMO
#ifdef WEB_DEMO

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "secrets.h"

#define MAX_CLIENTS 7

static httpd_handle_t server;

static void on_network_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
  if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
    esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = data;
    printf("WiFi connected - IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
  }
}

static void wifi_init(void)
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_network_event, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_network_event, NULL));

  wifi_config_t wifi_cfg = {
    .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());
}

// Called for every frame the browser sends (the server handles the handshake itself)
static esp_err_t ws_handler(httpd_req_t *req)
{
  // The page never sends anything, but drain any frame so the socket stays in sync
  uint8_t buf[32];
  httpd_ws_frame_t frame = { .payload = buf };
  return httpd_ws_recv_frame(req, &frame, sizeof(buf));
}

// Runs inside the server task (queued with httpd_queue_work); sends to every open WebSocket
static void send_counter(void *arg)
{
  char text[16];
  int len = snprintf(text, sizeof(text), "%d", (int)(intptr_t)arg);

  httpd_ws_frame_t frame = {
    .type    = HTTPD_WS_TYPE_TEXT,
    .payload = (uint8_t *)text,
    .len     = len,
  };

  int fds[MAX_CLIENTS];
  size_t count = MAX_CLIENTS;
  if (httpd_get_client_list(server, &count, fds) != ESP_OK) { return; }

  for (size_t i = 0; i < count; i++) {
    if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      httpd_ws_send_frame_async(server, fds[i], &frame);
    }
  }
}

void app_main(void)
{
  wifi_init();

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_open_sockets = MAX_CLIENTS;
  ESP_ERROR_CHECK(httpd_start(&server, &cfg));

  httpd_uri_t ws_uri = {
    .uri          = "/ws",
    .method       = HTTP_GET,
    .handler      = ws_handler,
    .is_websocket = true,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));

  int counter = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));
    httpd_queue_work(server, send_counter, (void *)(intptr_t)counter++);
  }
}

#endif /* WEB_DEMO */

