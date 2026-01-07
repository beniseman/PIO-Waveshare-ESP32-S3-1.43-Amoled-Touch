#include <Arduino.h>
#include "lvgl.h"

#include "lcd_bsp.h"   // lcd_lvgl_Init()
#include "ui/ui.h"     // ui_init()

// Touch driver (FT3168)
#include "FT3168.h"    // Touch_Init()

void setup()
{
  Serial.begin(115200);
  delay(50);

  // Touch first (prevents "i2c driver not installed" spam)
  Touch_Init();

  // Panel + LVGL registration (flush cb, buffers, rotation, etc.)
  lcd_lvgl_Init();

  // SquareLine UI creation
  ui_init();
}

void loop()
{
  lv_timer_handler();
  delay(5);
}