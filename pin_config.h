#pragma once
// --------------------------------------------------------------------
//   LilyGO T-Camera Plus S3  (V1.1 board revision, OV2640 camera)
// --------------------------------------------------------------------

// Select your board revision
#define T_CAMERA_PLUS_S3_V1_1

#ifdef T_CAMERA_PLUS_S3_V1_1

// ====================== LCD (for reference) =========================
#define LCD_MOSI      35
#define LCD_SCLK      36
#define LCD_MISO      37
#define LCD_CS        34
#define LCD_RST       33
#define LCD_DC        45
#define LCD_BL        46
#define LCD_WIDTH     240
#define LCD_HEIGHT    240

// ====================== OV2640 Camera Pins ==========================
#define OV2640_PWDN    -1     // Power-down not used
#define OV2640_RESET     3     // Camera reset
#define OV2640_XCLK      7     // External clock

#define OV2640_SDA       1     // SCCB (I2C) SDA
#define OV2640_SCL       2     // SCCB (I2C) SCL

#define OV2640_D7        6
#define OV2640_D6        8
#define OV2640_D5        9
#define OV2640_D4       11
#define OV2640_D3       13
#define OV2640_D2       15
#define OV2640_D1       14
#define OV2640_D0       12

#define OV2640_VSYNC     4
#define OV2640_HREF      5
#define OV2640_PCLK     10

// ====================== Power & Utility Pins ========================
#define AP1511B_FBC     16     // Power enable for camera rail
#define KEY1             0     // Optional user button

#endif  // T_CAMERA_PLUS_S3_V1_1

// =================== Map to esp_camera driver ======================
#define PWDN_GPIO_NUM   OV2640_PWDN
#define RESET_GPIO_NUM  OV2640_RESET
#define XCLK_GPIO_NUM   OV2640_XCLK
#define SIOD_GPIO_NUM   OV2640_SDA
#define SIOC_GPIO_NUM   OV2640_SCL

#define Y9_GPIO_NUM     OV2640_D7
#define Y8_GPIO_NUM     OV2640_D6
#define Y7_GPIO_NUM     OV2640_D5
#define Y6_GPIO_NUM     OV2640_D4
#define Y5_GPIO_NUM     OV2640_D3
#define Y4_GPIO_NUM     OV2640_D2
#define Y3_GPIO_NUM     OV2640_D1
#define Y2_GPIO_NUM     OV2640_D0

#define VSYNC_GPIO_NUM  OV2640_VSYNC
#define HREF_GPIO_NUM   OV2640_HREF
#define PCLK_GPIO_NUM   OV2640_PCLK
