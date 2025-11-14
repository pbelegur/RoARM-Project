#include <WiFi.h>
#include <WebSocketsClient.h>
#include "esp_camera.h"
#include "pin_config.h"
#define SERVER_IP "192.168.181.150"  // your PC IP
#define SERVER_PORT 8888
#define WIFI_SSID      "PSA35"
#define WIFI_PASSWORD  "Pavan@07"


WebSocketsClient ws;

bool initCamera() {
  pinMode(AP1511B_FBC, OUTPUT);
  digitalWrite(AP1511B_FBC, HIGH);
  delay(200);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0   = 12;  c.pin_d1 = 14;  c.pin_d2 = 15;  c.pin_d3 = 13;
  c.pin_d4   = 11;  c.pin_d5 = 9;   c.pin_d6 = 8;   c.pin_d7 = 6;
  c.pin_xclk = 7;   c.pin_pclk = 10; c.pin_vsync = 4; c.pin_href = 5;
  c.pin_sccb_sda = 1; c.pin_sccb_scl = 2;
  c.pin_pwdn = -1;  c.pin_reset = 3;

  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;   // FRAMESIZE_XGA for sharper image
  c.jpeg_quality = 10;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) return false;
  return true;
}

void sendFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  if (ws.isConnected()) {
    ws.sendBIN(fb->buf, fb->len);
  //  delay(50); // add a small delay to let server breathe
  }
  esp_camera_fb_return(fb);

  // ws.sendBIN(fb->buf, fb->len);
  // esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected to WiFi! IP: %s\n", WiFi.localIP().toString().c_str());

  if (!initCamera()) {
    Serial.println("Camera init failed!");
    while (true);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_sharpness(s, 2);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_denoise(s, 1);
  s->set_saturation(s, 1);
  s->set_gainceiling(s, GAINCEILING_64X);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 5);
  s->set_lenc(s, 1);

  ws.begin(SERVER_IP, SERVER_PORT, "/jpgstream_server");
  ws.onEvent([](WStype_t type, uint8_t *payload, size_t length){
    if (type == WStype_CONNECTED)
      Serial.println("✅ Connected to Python WebSocket server!");
    if (type == WStype_DISCONNECTED)
      Serial.println("❌ Disconnected from server.");
  });
  ws.setReconnectInterval(5000);
}

void loop() {
  ws.loop();
  static unsigned long lastSend = 0;

  if (WiFi.status() == WL_CONNECTED && ws.isConnected() && millis() - lastSend > 100) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      ws.sendBIN(fb->buf, fb->len);   // send JPEG to Python
      esp_camera_fb_return(fb);
      lastSend = millis();
      Serial.printf("Sent frame: %d bytes\n", fb->len);
    }
  }
}
