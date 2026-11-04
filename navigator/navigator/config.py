from math import pi

## 50Hz
DUTY_MIN = 0.025  # equivalent to 1000µs for a 50Hz PWM (20000µs period)
DUTY_MID = 0.075  # equivalent to 1500µs for a 50Hz PWM (20000µs period)
DUTY_MAX = 0.125  # equivalent to 2000µs for a 50Hz PWM (20000µs period)

## 50Hz
PWM_FREQ = 50

RX_PORT = "/dev/ttyS0"

FREQ_IMU = 100