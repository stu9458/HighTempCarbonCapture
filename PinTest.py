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

Relay1_delay_sec_list = [20, 15] # 低壓加溫20秒、高壓加溫10秒
Relay2_delay_sec_list = [35, 70] # High不加熱、Low加熱
Relay1_sec_count = 0
Relay2_sec_count = 0
Relay1_status = 0
Relay2_status = 0
Unactivated = GPIO.HIGH
Activated   = GPIO.LOW

while True :
    # val_0 = ADS.readADC(0)
    # volt_0 = val_0 * f
    # print("Analog0: {0:d}\t{1:.3f}V".format(val_0, volt_0))
    if (Relay1_status == 0): #Voltage Low
        GPIO.output(VoltageChangePin, Unactivated)
    elif (Relay1_status == 1): #Voltage High
        GPIO.output(VoltageChangePin, Activated)

    if (Relay2_status == 0): #Heating Off
        GPIO.output(HeatingPin, Unactivated)
    elif (Relay2_status == 1): #Heating Enable
        GPIO.output(HeatingPin, Activated)
    elif (Relay2_status == 2): #Heating Off
        GPIO.output(HeatingPin, Unactivated)

    if (Relay1_sec_count >= Relay1_delay_sec_list[Relay1_status]):
        Relay1_sec_count = 0
        Relay1_status = Relay1_status+1

    if (Relay2_sec_count >= Relay2_delay_sec_list[Relay2_status]):
        Relay2_sec_count = 0
        Relay2_status = Relay2_status+1

    if (Relay1_status >= len(Relay1_delay_sec_list)):
        Relay1_status = 0
    if (Relay2_status >= len(Relay2_delay_sec_list)):
        Relay2_status = 0

    print(f'１號Relay當前狀態：{Relay1_status}. 當前狀態秒數：{Relay1_sec_count}. ２號Relay當前狀態：{Relay2_status}. 當前狀態秒數：{Relay2_sec_count}')

    Relay1_sec_count = Relay1_sec_count + 1
    Relay2_sec_count = Relay2_sec_count + 1
    sleep(1)