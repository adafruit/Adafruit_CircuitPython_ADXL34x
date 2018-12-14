import time
import board
import busio
import adafruit_adxl34x

i2c = busio.I2C(board.SCL, board.SDA)

accelerometer = adafruit_adxl34x.ADXL345(i2c)
accelerometer.enable_tap_detection()
# you can also configure the tap detection parameters when you enable tap detection:
# accelerometer.enable_tap_detection(tap_count=2,threshold=20, duration=50)
while True:
    print("%f %f %f"%accelerometer.acceleration)

    print("Tapped: %s"%accelerometer.events['tap'])
    time.sleep(0.5)
