# Waveshare ESP32-S3 AMOLED 1.43" – PlatformIO

This project provides a **minimal, working PlatformIO setup** for the **Waveshare ESP32-S3 1.43" AMOLED Touch Display**.

The goal of this repository is **not** to demonstrate features or applications, but simply to:

- Bring the board up cleanly in **PlatformIO**
- Initialize the AMOLED display (SH8601 / CO5300)
- Enable **LVGL (v8.x)** rendering
- Enable **FT3168 capacitive touch**
- Support **SquareLine Studio–generated UI projects**
- Serve as a stable foundation for future projects

---

## Hardware

- **Board:** Waveshare ESP32-S3 Touch AMOLED 1.43"
- **Display:** AMOLED (SH8601 / CO5300)
- **Touch Controller:** FT3168
- **Interface:** QSPI (display), I²C (touch)

---

## Software Stack

- **PlatformIO**
- **Arduino framework (ESP32-S3)**
- **LVGL 8.x**
- **Waveshare-provided BSP + drivers**
- **SquareLine Studio compatibility**

This project intentionally uses the **Waveshare driver and LVGL files directly**, rather than relying on external LVGL display libraries, to avoid artifacts and timing issues.

---
