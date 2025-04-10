import threading
import serial
import math
from time import time, sleep
from datetime import datetime
# import spidev

ser = serial.Serial()
ser2 = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser2.port = "/dev/ttyUSB1"
# ser.port = "COM3"

# 38400,N,8,1
ser.baudrate = 38400
ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser.parity = serial.PARITY_NONE  # set parity check
ser.stopbits = serial.STOPBITS_ONE  # number of stop bits

ser.timeout = 0.5  # non-block read 0.5s
ser.writeTimeout = 0.5  # timeout for write 0.5s
ser.xonxoff = False  # disable software flow control
ser.rtscts  = False  # disable hardware (RTS/CTS) flow control
ser.dsrdtr  = False  # disable hardware (DSR/DTR) flow control

ser2.baudrate = 38400
ser2.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser2.parity = serial.PARITY_NONE  # set parity check
ser2.stopbits = serial.STOPBITS_ONE  # number of stop bits

ser2.timeout = 0.5  # non-block read 0.5s
ser2.writeTimeout = 0.5  # timeout for write 0.5s
ser2.xonxoff = False  # disable software flow control
ser2.rtscts  = False  # disable hardware (RTS/CTS) flow control
ser2.dsrdtr  = False  # disable hardware (DSR/DTR) flow control

auchCRCHi = [ 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
 0x40]

auchCRCLo = [ 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
 0x40]

def crc16(data, ifrom, ito) :
    uchCRCHi = 0xff
    uchCRCLo = 0xff
    for i in range(ifrom, ito):
        uindex = uchCRCHi ^ data[i]
        uchCRCHi = uchCRCLo ^ auchCRCHi[uindex]
        uchCRCLo = auchCRCLo[uindex]
    return (uchCRCHi << 8) | uchCRCLo

def read_max6675(spi_channel, spi_device):
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device)
    # spi.mode = 0
    spi.max_speed_hz = 50000  # You may need to adjust this value based on your hardware

    try:
        # Read the raw ADC value from the MAX6675
        adc_data = spi.xfer2([0, 0])
        # print(f"ADC1:{adc_data[0]}. ADC2:{adc_data[1]}")
        # Combine the two bytes into a 12-bit value
        output = ((adc_data[0] & 0x7F) << 5) | (adc_data[1] >> 3)

        # Convert the raw value to Celsius
        temperature_celsius = output * 0.25
        temperature_celsius = (temperature_celsius - 2) * 97 / 100

        # Convert to Fahrenheit
        temperature_fahrenheit = (temperature_celsius * 9 / 5) + 32

        print(f"攝氏溫度: {temperature_celsius:.2f} °C, 華氏溫度：{temperature_fahrenheit:.2f} °F")
        # return temperature_celsius

    except KeyboardInterrupt:
        print("Temperature reading stopped.")
    finally:
        spi.close()

def Get_Current_Power():
    if ser.isOpen():
        ser.flushInput()  # flush input buffer
        ser.flushOutput() # flush output buffer

        # cmd_flush_AT2P = [0xAA, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        cmd_read_AT2P = [0xAA, 0x37, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        hi_c = crc16(cmd_read_AT2P, 0, 9) >> 8
        lo_c = crc16(cmd_read_AT2P, 0, 9)  & 0x00ff
        cmd_read_AT2P.append(lo_c)
        cmd_read_AT2P.append(hi_c)
        cmd_read_AT2P.append(0xFF)
        try:
            ser.write(cmd_read_AT2P)
            sleep(0.1)  # wait 0.1s
            response = ser.read(42)
            # 電壓
            read_vol = ((response[3] << 24) + (response[4] << 16) + (response[5] << 8) + (response[6] << 0)) / 100
            # 電流
            read_amp = ((response[7] << 24) + (response[8] << 16) + (response[9] << 8) + (response[10] << 0)) / 1000
            # 功率
            read_power = ((response[11] << 24) + (response[12] << 16) + (response[13] << 8) + (response[14] << 0)) / 100

            print(f"量測功率: {read_power}W. 量測電壓:{read_vol}V. 量測電流:{read_amp}A")
            return read_power

        except Exception as e1:
            print("communicating error " + str(e1))
            # ser.close()
def Get_Temperature():
    global pre_temp
    if ser.isOpen():
        ser.flushInput()  # flush input buffer
        ser.flushOutput() # flush output buffer

        cmd_read_temp = [0x01, 0x03, 0x00, 0x00, 0x00, 0x02]
        hi_c = crc16(cmd_read_temp, 0, 6) >> 8
        lo_c = crc16(cmd_read_temp, 0, 6) & 0x00ff
        cmd_read_temp.append(hi_c)
        cmd_read_temp.append(lo_c)
        print("Send data:", cmd_read_temp)
        try:
            ser.write(cmd_read_temp)
            sleep(0.1)  # wait 0.1s
            response = ser.read(9)
            print("read byte data:", response.hex())
            read_temp = 0
            if (response[2] == 4):
                read_temp = (response[3] << 8) + (response[4] << 0)
                print(f"熱電偶量測溫度: {read_temp}.")

            return read_temp

        except Exception as e1:
            print("溫度讀取失敗，請檢查溫度量測器 " + str(e1))

def Get_MDK():
    global pre_temp
    if ser2.isOpen():
        ser2.flushInput()  # flush input buffer
        ser2.flushOutput() # flush output buffer

        cmd_read_MDK = [0x05, 0x03, 0x00, 0x06, 0x00, 0x01]
        hi_c = crc16(cmd_read_MDK, 0, 6) >> 8
        lo_c = crc16(cmd_read_MDK, 0, 6) & 0x00ff
        cmd_read_MDK.append(hi_c)
        cmd_read_MDK.append(lo_c)
        # print("Send data:", cmd_read_MDK)
        try:
            ser2.write(cmd_read_MDK)
            sleep(0.1)  # wait 0.1s
            response = ser2.read(9)
            # print("read byte data:", response.hex())
            read_MDK = 0
            if (response[2] == 2):
                read_MDK = (response[3] << 8) + (response[4] << 0)
                print(f"讀取MDK濃度: {read_MDK/10}.")

            return read_MDK

        except Exception as e1:
            print("濃度讀取失敗，請檢查MDK濃度偵測器 " + str(e1))

# try:
ser.open()
ser2.open()
while True:
    Get_Temperature()
    Get_Current_Power()
    Get_MDK()
    sleep(1)
# except Exception as ex:
#     print("溫度感測模組或功率錶頭未插入(open serial port error) " + str(ex))
#     exit()


