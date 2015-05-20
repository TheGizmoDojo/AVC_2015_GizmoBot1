# autonomous_vehicle

Arduino source code for an autonomous vehicle using the following components:

1. Arduino Uno or Mega
2. Adafruit Ultimate GPS
3. Sparkfun HMC6352 magenetometer breakout
4. 5 x HC-SR04 ultra-sonic sensors
5. Pololu Qik 2s12v10 motor controller board

## Arduino Mega Notes

Need to replace standard SD library with this one that uses soft SPI on pins 10,11,12,13 instead of 50,51,52

https://github.com/adafruit/SD

| Arduino | Component   | Wire color |
|---------|-------------|------------|
| 10-13   | SD shield   | N/A        |
| 18      | GPS RX      | Gray       |
| 19      | GPS TX      | White      |
| 20      | Compass SDA | Gray       |
| 21      | Compass SCL | White      |
| 50      | Qik TX      | Gray       |
| 51      | Qik RX      | Purple     |
| 52      | Qik RESET   | White      |
| A8-A12  | HC-SR04     | Various    |

## Arduino Uno Notes (Untested)

Note that the GPS is connected to the hardware serial UART so cannot use serial monitor at the same time.

| Arduino | Component   | Wire color |
|---------|-------------|------------|
| 0       | GPS RX      | Gray       |
| 1       | GPS TX      | White      |
| 2       | Qik TX      | Gray       |
| 3       | Qik RX      | Purple     |
| 4       | Qik RESET   | White      |
| 10-13   | SD shield   | N/A        |
| A0-A3,7 | HC-SR04     | Various    |
| A4      | Compass SDA | Gray       |
| A5      | Compass SCL | White      |


