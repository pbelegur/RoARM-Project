#include <WiFi.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "pin_config.h"
#include "esp_wifi.h"  // WiFi tuning

// ---------- WiFi ----------
#define WIFI_SSID      "LGMesaroof"
#define WIFI_PASSWORD  "12345678"

// ---------- Stream constants ----------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---------- Server handles ----------
httpd_handle_t httpd_main = nullptr;

// ---------- HTML Root ----------
static esp_err_t handle_root(httpd_req_t *req) {
  const char html[] =
      "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'/>"
      "<h3>T-CameraPlus-S3</h3>"
      "<p><a href='/ping'>/ping</a></p>"
      "<p><a href='/status'>/status</a></p>"
      "<p><a href='/capture'>/capture</a></p>"
      "<p>Stream: <a href='/stream'>/stream</a></p>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, sizeof(html) - 1);
}

// ---------- /ping ----------
static esp_err_t handle_ping(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "pong", 4);
}

// ---------- /status ----------
static esp_err_t handle_status(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  char buf[128];
  int n = snprintf(buf, sizeof(buf),
                   "{\"ok\":true,\"pid\":%d,\"framesize\":%d,\"quality\":%d}",
                   s ? s->id.PID : -1,
                   s ? s->status.framesize : -1,
                   s ? s->status.quality : -1);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

// ---------- /capture ----------
static esp_err_t handle_capture(httpd_req_t *req) {
  httpd_resp_set_type(req, "image/jpeg");
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_set_status(req, "500");
    return httpd_resp_send(req, "fb null", 7);
  }
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ---------- /stream ----------
static esp_err_t handle_stream(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  static unsigned long frameCounter = 0;

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK)
      break;

    char part_buf[64];
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)fb->len);
    if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK)
      break;

    if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK)
      break;

    esp_camera_fb_return(fb);
    Serial.printf("[Stream] Frame %lu sent\n", ++frameCounter);

    // Smooth pacing (~30 fps cap)
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
  return ESP_OK;
}

// ---------- Start HTTP server ----------
static void start_servers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 10;

  static const httpd_uri_t uri_root    = { .uri = "/",        .method = HTTP_GET, .handler = handle_root,    .user_ctx = NULL };
  static const httpd_uri_t uri_ping    = { .uri = "/ping",    .method = HTTP_GET, .handler = handle_ping,    .user_ctx = NULL };
  static const httpd_uri_t uri_status  = { .uri = "/status",  .method = HTTP_GET, .handler = handle_status,  .user_ctx = NULL };
  static const httpd_uri_t uri_capture = { .uri = "/capture", .method = HTTP_GET, .handler = handle_capture, .user_ctx = NULL };
  static const httpd_uri_t uri_stream  = { .uri = "/stream",  .method = HTTP_GET, .handler = handle_stream,  .user_ctx = NULL };

  if (httpd_start(&httpd_main, &cfg) == ESP_OK) {
    httpd_register_uri_handler(httpd_main, &uri_root);
    httpd_register_uri_handler(httpd_main, &uri_ping);
    httpd_register_uri_handler(httpd_main, &uri_status);
    httpd_register_uri_handler(httpd_main, &uri_capture);
    httpd_register_uri_handler(httpd_main, &uri_stream);
  }
}

// ---------- Camera init ----------
static bool init_camera() {
  pinMode(AP1511B_FBC, OUTPUT);
  digitalWrite(AP1511B_FBC, HIGH);
  delay(200);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0   = 12;
  c.pin_d1   = 14;
  c.pin_d2   = 15;
  c.pin_d3   = 13;
  c.pin_d4   = 11;
  c.pin_d5   = 9;
  c.pin_d6   = 8;
  c.pin_d7   = 6;
  c.pin_xclk = 7;
  c.pin_pclk = 10;
  c.pin_vsync= 4;
  c.pin_href = 5;
  c.pin_sccb_sda = 1;
  c.pin_sccb_scl = 2;
  c.pin_pwdn  = -1;
  c.pin_reset = 3;

  c.xclk_freq_hz = 10000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QQVGA;   // smoother (160x120)
  c.jpeg_quality = 30;                // lighter compression, decent quality
  c.fb_count     = 3;                 // triple buffer for stable streaming
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init fail 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("[CAM] PID=0x%x (should be 0x2640)\n", s->id.PID);
    s->set_framesize(s, FRAMESIZE_QQVGA);
    s->set_quality(s, 25);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
  }

  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(60);
  }
  return true;
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBooting…");
  Serial.printf("[MEM] Free heap: %u, PSRAM: %u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());

  if (!init_camera()) {
    Serial.println("[CAM] init failed; rebooting in 5s");
    delay(5000);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);   // boost signal
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP: %s (RSSI: %d dBm)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  start_servers();
  Serial.println("HTTP server ready");
  Serial.printf("Open: http://%s\n", WiFi.localIP().toString().c_str());
}

// ---------- loop ----------
void loop() {
  static uint32_t t0 = millis();
  if (millis() - t0 > 5000) {
    digitalWrite(AP1511B_FBC, HIGH);
    t0 = millis();
    Serial.printf("[MEM] Free heap: %u, PSRAM: %u\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
  }
  delay(10);
}
