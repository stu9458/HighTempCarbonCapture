import tkinter as tk
import threading
import RPi.GPIO as GPIO
from time import time, sleep
from datetime import datetime
import spidev

def read_max6675(spi_channel, spi_device):
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device)
    # spi.mode = 0
    spi.max_speed_hz = 50000  # You may need to adjust this value based on your hardware

    try:
        while True:
            # Read the raw ADC value from the MAX6675
            adc_data = spi.xfer2([0, 0])
            print(f"ADC1:{adc_data[0]}. ADC2:{adc_data[1]}")
            # Combine the two bytes into a 12-bit value
            output = ((adc_data[0] & 0x7F) << 5) | (adc_data[1] >> 3)

            # Convert the raw value to Celsius
            temperature_celsius = output * 0.25
            temperature_celsius = (temperature_celsius - 2) * 97 / 100

            # Convert to Fahrenheit
            temperature_fahrenheit = (temperature_celsius * 9 / 5) + 32

            print(f"Temperature: {temperature_celsius:.2f} °C, {temperature_fahrenheit:.2f} °F")
            sleep(1)
            # return temperature_celsius

    except KeyboardInterrupt:
        print("Temperature reading stopped.")
    finally:
        spi.close()

while True:
    read_max6675(0, 0)
    sleep(1)

