# LIS3DH Accelerometer Driver (ESP-IDF)

This driver provides basic initialization, identification, and acceleration reading support for the LIS3DH accelerometer over I²C.

## Functions
- `lis3dh_init()` – Sets up 100Hz ODR, enables XYZ.
- `lis3dh_who_am_i()` – Reads WHO_AM_I (0x33 expected).
- `lis3dh_read_accel()` – Reads raw X/Y/Z 12-bit acceleration data.
- `lis3dh_convert_to_g()` – Converts raw readings to g-units.

## Example Wiring
| LIS3DH Pin | ESP32-S3 |
|-------------|----------|
| SCL         | GPIO 9   |
| SDA         | GPIO 8   |
| VCC         | 3.3V     |
| GND         | GND      |

Use I²C address `0x18` (SA0=GND) or `0x19` (SA0=VCC).
