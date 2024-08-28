import os
import RPi.GPIO as GPIO
from time import time, sleep
import ADS1x15

VoltageChangePin = 26 # Broadcom pin 18 PWM1 (P1 pin 33),  PWM 只有12, 13
HeatingPin = 20 # Broadcom pin 17 (P1 pin 11)
ReservePin = 21 # Broadcom pin 27 (P1 pin 13)

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(HeatingPin, GPIO.OUT) # LED pin set as output
GPIO.setup(ReservePin, GPIO.OUT) # LED pin set as output
GPIO.setup(VoltageChangePin, GPIO.OUT) # PWM pin set as output

# ADS = ADS1x15.ADS1115(1, 0x48)
#
# print(os.path.basename(__file__))
# print("ADS1X15_LIB_VERSION: {}".format(ADS1x15.__version__))
#
# # set gain to 4.096V max
# ADS.setGain(ADS.PGA_4_096V)
#
# # set comparator to traditional mode, active high, latch, and trigger alert after 1 conversion
# ADS.setComparatorMode(ADS.COMP_MODE_TRADITIONAL)
#
# ADS.setComparatorPolarity(ADS.COMP_POL_ACTIV_HIGH)
# ADS.setComparatorLatch(ADS.COMP_LATCH)
# ADS.setComparatorQueue(ADS.COMP_QUE_1_CONV)
# # set threshold
# f = ADS.toVoltage()
# ADS.setComparatorThresholdLow(int(1.5 / f))    # 1.5V
# ADS.setComparatorThresholdHigh(int(2.5 / f))   # 2.5V

while True :
    # val_0 = ADS.readADC(0)
    # volt_0 = val_0 * f
    # print("Analog0: {0:d}\t{1:.3f}V".format(val_0, volt_0))
    GPIO.output(VoltageChangePin, GPIO.LOW)
    sleep(1)
    GPIO.output(HeatingPin, GPIO.LOW)
    sleep(1)
    GPIO.output(ReservePin, GPIO.LOW)
    sleep(1)
    # val_0 = ADS.readADC(0)
    # volt_0 = val_0 * f
    # print("Analog0: {0:d}\t{1:.3f}V".format(val_0, volt_0))
    GPIO.output(VoltageChangePin, GPIO.HIGH)
    sleep(1)
    GPIO.output(HeatingPin, GPIO.HIGH)
    sleep(1)
    GPIO.output(ReservePin, GPIO.HIGH)
    sleep(2.5)