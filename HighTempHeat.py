#!/usr/bin/python3

# External module imports
import serial
import tkinter as tk
import threading
import RPi.GPIO as GPIO
from time import time, sleep
from datetime import datetime
from tkinter import messagebox
import spidev

VoltageChangePin = 26 # Broadcom pin 18 PWM1 (P1 pin 33),  PWM 只有12, 13
HeatingPin = 20 # Broadcom pin 17 (P1 pin 11)
ReservePin = 21 # Broadcom pin 27 (P1 pin 13)

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

ser = serial.Serial()
# ser.port = "COM4"
ser.port = "/dev/ttyUSB0"

input_temperature = 80.0
temperature_max = 0
temperature_min = temperature_max - 10
heat_time = time()                      # 開始加熱時間
elapsed_time = 0                        # 已加熱秒數
estimated_heat_time = "10:00"           # 預計加熱時長
heat_power = 12*7.5                     # 固定加熱功率
heat_kwh = 0.0                          # 累計加熱功率，度數

# 57600,N,8,1
ser.baudrate = 57600
ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser.parity = serial.PARITY_NONE  # set parity check
ser.stopbits = serial.STOPBITS_ONE  # number of stop bits

ser.timeout = 0.5  # non-block read 0.5s
ser.writeTimeout = 0.5  # timeout for write 0.5s
ser.xonxoff = False  # disable software flow control
ser.rtscts  = False  # disable hardware (RTS/CTS) flow control
ser.dsrdtr  = False  # disable hardware (DSR/DTR) flow control

# 建立 flag
fun1_exit_flag = threading.Event()
fun2_exit_flag = threading.Event()
fun3_exit_flag = threading.Event()
fun4_exit_flag = threading.Event()

log_fun1 = False
log_fun2 = False
log_fun3 = False
log_fun4 = False

# 設定 fun1的函数
fun1_thread = None  # 目前溫度
fun2_thread = None  # Heating
fun3_thread = None  # 加熱時間
fun4_thread = None  # 冷卻時間

def crc16(data, ifrom, ito) :
    uchCRCHi = 0xff
    uchCRCLo = 0xff
    for i in range(ifrom, ito):
        uindex = uchCRCHi ^ data[i]
        uchCRCHi = uchCRCLo ^ auchCRCHi[uindex]
        uchCRCLo = auchCRCLo[uindex]
    return (uchCRCHi << 8) | uchCRCLo

def GetShortFromBigEndianArray(data, sindex):
    bb = [0, 0]
    if (data.Length >= sindex + 2):
        bb = data.copy()
        bb.reverse()
        return bb
    return 0

def Reserve_on():
    GPIO.output(ReservePin, GPIO.LOW)
    print("[Status]預留Relay開啟")

def Reserve_off():
    GPIO.output(ReservePin, GPIO.HIGH)
    print("[Status]預留Relay關閉")

def Heating_on():
    GPIO.output(HeatingPin, GPIO.LOW)
    print("[Status]開始加熱")

def Heating_off():
    GPIO.output(HeatingPin, GPIO.HIGH)
    print("[Status]停止加熱")

def VoltageChange_on():
    GPIO.output(VoltageChangePin, GPIO.LOW)
    print("[Status]切換到48V")

def VoltageChange_off():
    GPIO.output(VoltageChangePin, GPIO.HIGH)
    print("[Status]切換到36V")

def update_heat_time(wtime):
    heat_time_label['text'] = str(wtime)
    # print('time: ', strftime("%M:%S", gmtime()))

def update_cooling_time(wtime):
    cooling_time_label['text'] = str(wtime)

def set_heat_time():
    global input_heating_time
    global set_heating_time_value
    try:
        value = datetime.strptime(str(input_heating_time.get()), "%M:%S").time()
        print('輸入加熱時間 heat_time: ', str(value)[3:])
        set_heating_time_value.set(str(value)[3:])
    except:
        print("時間設定格式錯誤，請重新設定")
        set_heating_time_value.set("00:00")

def set_input_temperature():
    global input_temperature
    global set_temperature_value
    value = float(input_temperature.get())
    print('輸入溫度 input_temperature: ', value)
    set_temperature_value.set(str(value))

def Get_Temperature():
    if ser.isOpen():
        ser.flushInput()  # flush input buffer
        ser.flushOutput() # flush output buffer

        cmd_read_temp = [0x01, 0x03, 0x00, 0x02, 0x00, 0x06]
        hi_c = crc16(cmd_read_temp, 0, 6) >> 8
        lo_c = crc16(cmd_read_temp, 0, 6) & 0x00ff
        cmd_read_temp.append(hi_c)
        cmd_read_temp.append(lo_c)
        try:
            ser.write(cmd_read_temp)
            sleep(0.1)  # wait 0.1s
            response = ser.read(68)

            # print("read byte data:", response.hex())
            read_temp, heat_temp = 0, 0
            if (response[2] == 12):
                read_temp = ((response[3] << 24) + (response[4] << 16) + (response[5] << 8) + (response[6]))/100
                heat_temp = ((response[7] << 8) + response[8]) / 100
                print(f"紅外線量測溫度: {read_temp}. 探頭量測溫度:{heat_temp}")

            return read_temp

        except Exception as e1:
            print("communicating error " + str(e1))
            ser.close()

def Get_Current_Power():
    if ser.isOpen():
        ser.flushInput()  # flush input buffer
        ser.flushOutput() # flush output buffer

        cmd_read_AT2P = [0x37, 0x03, 0x00, 0xAA, 0x00, 0x0e]
        hi_c = crc16(cmd_read_AT2P, 0, 6) >> 8
        lo_c = crc16(cmd_read_AT2P, 0, 6) & 0x00ff
        cmd_read_AT2P.append(hi_c)
        cmd_read_AT2P.append(lo_c)
        try:
            ser.write(cmd_read_AT2P)
            sleep(0.1)  # wait 0.1s
            response = ser.read(132)

            read_power = 0
            if (response[2] == 28):
                # 電壓
                read_vol = (response[3] * 100) >> 24
                read_vol += (response[4] * 100) >> 16 & 0x00ff00
                read_vol += (response[5] * 100) >> 8 & 0x0000ff
                read_vol += (response[6] * 100) & 0x0000ff
                # 電流
                read_amp = (response[7] * 100) >> 24
                read_amp += (response[8] * 100) >> 16 & 0x00ff00
                read_amp += (response[9] * 100) >> 8 & 0x0000ff
                read_amp += (response[10] * 100) & 0x0000ff
                # 功率
                read_power = (response[11] * 100) >> 24
                read_power += (response[12] * 100) >> 16 & 0x00ff00
                read_power += (response[13] * 100) >> 8 & 0x0000ff
                read_power += (response[14] * 100) & 0x0000ff

                print(f"量測功率: {read_power}. 量測電壓:{read_vol}. 量測電流:{read_amp}")
            return read_power

        except Exception as e1:
            print("communicating error " + str(e1))
            ser.close()

def Update_Current_Temperature(temp):
    global current_temperature_label
    current_temperature_label['text'] = str(temp)

def Update_Current_Power(current_power):
    global current_power_label
    current_power_label['text'] = str(current_power)

def Pilotlamp_switch(heating_color, retaning_color, cooling_color="red"):
    global heating_light, retaining_light, cooling_light
    heating_light.delete()
    heating_light.create_oval(5, 5, 20, 20, width=2, fill=heating_color, outline=heating_color)
    retaining_light.delete()
    retaining_light.create_oval(5, 5, 20, 20, width=2, fill=retaning_color, outline=retaning_color)
    cooling_light.delete()
    cooling_light.create_oval(5, 5, 20, 20, width=2, fill=cooling_color, outline=cooling_color)

def init():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
    GPIO.setup(HeatingPin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(ReservePin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(VoltageChangePin, GPIO.OUT)  # PWM pin set as output
    VoltageChange_off()
    Heating_off()
    Reserve_off()

def Start():
    Pilotlamp_switch(heating_color="#00FF00", retaning_color="red", cooling_color="red")
    try:
        ser.open()
    except Exception as ex:
        print("溫度感測模組未插入(open serial port error) " + str(ex))
        GPIO.cleanup()
        exit()

    VoltageChange_on()
    Heating_on()
    Reserve_on()

def Stop(r):
    Pilotlamp_switch(heating_color="red", retaning_color="red", cooling_color="#00FF00")
    VoltageChange_off()
    Heating_off()
    Reserve_off()
    ser.close()
    exit_program(r)

def exit_program(root):
    global fun1_thread, fun2_thread, fun3_thread
    if fun1_thread:
        stop_fun1()
    if fun2_thread:
        stop_fun2()
    if fun3_thread:
        stop_fun3()
    # root.quit()
def fun1(): # 更新加熱時間Thread，並判定是否有超過預設加熱時間
    global elapsed_time, estimated_heat_time, fun1_thread
    while not fun1_exit_flag.is_set():
        if log_fun1:
            print("執行 fun1:目前溫度 執行序...")
        minutes, seconds = map(int, estimated_heat_time.split(':'))
        mins, secs = divmod(elapsed_time, 60)

        # 更新加熱時間
        # print(f'加熱時間: {mins}:{secs}')
        update_heat_time(f'{mins}:{secs}')

        if (mins * 60 + secs) >= (minutes * 60 + seconds):
            if fun2_thread:
                stop_fun2()
            break
        sleep(1)

def fun2(): #進行加熱(同時更新溫度)Thread
    global input_temperature, heat_time
    while not fun2_exit_flag.is_set():
        if log_fun2:
            print("執行 fun2:Heating 執行序...")
        temp = Get_Temperature()
        # current_power = Get_Current_Power()
        if (temp != None):
            Update_Current_Temperature(f'{temp:.1f}')
            # Update_Current_Power(f'{current_power:.1f}')
        sleep(1)
def fun3(): # 更新當前加熱時間，回傳給fun1()
    global heat_time, elapsed_time
    current_time = time()
    while not fun3_exit_flag.is_set():
        elapsed_time = int(time() - current_time)
        if log_fun3:
            print("執行 fun3:加熱時間 執行序...")
            print(f'elapsed_time: {elapsed_time}')
        sleep(1)

def fun4():
    global elapsed_time
    current_time = time()
    while not fun4_exit_flag.is_set():
        elapsed_time = int(time() - current_time)
        if log_fun4:
            print("執行 fun4:冷卻時間確認 執行序...")
            print(f'elapsed_time: {elapsed_time}')
        temp = Get_Temperature()
        # current_power = Get_Current_Power()
        if (temp != None):
            Update_Current_Temperature(f'{temp:.1f}')
            # Update_Current_Power(f'{current_power:.1f}')
        mins, secs = divmod(elapsed_time, 60)
        update_cooling_time(f'{mins}:{secs}')
        sleep(1)

# run fun1的函数
def start_fun1():
    global fun1_thread
    fun1_exit_flag.clear()
    fun1_thread = threading.Thread(target=fun1, daemon=True)
    fun1_thread.start()

def start_fun2():
    global fun2_thread
    fun2_exit_flag.clear()
    fun2_thread = threading.Thread(target=fun2, daemon=True)
    fun2_thread.start()
    # STOP_button.configure(bg="red")

def start_fun3():
    global fun3_thread
    fun3_exit_flag.clear()
    fun3_thread = threading.Thread(target=fun3, daemon=True)
    fun3_thread.start()

def start_fun4():
    global fun4_thread
    try:
        ser.open()
    except Exception as ex:
        print("open serial port error " + str(ex))
        GPIO.cleanup()
        exit()

    fun4_exit_flag.clear()
    fun4_thread = threading.Thread(target=fun4, daemon=True)
    fun4_thread.start()

def stop_fun1():
    global fun1_thread
    fun1_exit_flag.set()
    fun1_thread.join()  # wait for thread stop
    print("fun1 has stopped.")

def stop_fun2():
    global fun2_thread
    fun2_exit_flag.set()
    fun2_thread.join()  # wait for thread stop
    print("fun2:Heating has stopped.")

def stop_fun3():
    global fun3_thread
    fun3_exit_flag.set()
    fun3_thread.join()  # wait for thread stop
    print("fun3:heat_time has stopped.")

def stop_fun4():
    global fun4_thread
    if fun4_thread:
        ser.close()
        fun4_exit_flag.set()
        fun4_thread.join()  # wait for thread stop
        print("fun4:Cooling time counter has stopped.")

def get_total_seconds(minutes, seconds):
    return minutes * 60 + seconds

##### 主要視窗設定
root = tk.Tk()
root.title("高溫感應式加熱模組電控")
root.geometry('600x350')

display_data_region = tk.LabelFrame(root, text='顯示區域', padx=10, pady=10)
display_data_region.place(x=0, y=0)

# create display UI
entry_width=10
# 加熱時間
w_row = 0
w_column = 0
heat_time_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid")
heat_time_label.grid(column=w_column, row=w_row)

w_row = 1
w_lable = tk.Label(display_data_region, text='Heating Time(mins)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)

# 當前溫度
w_row = 2
w_column = 0
current_temperature_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid")
current_temperature_label.grid(column=w_column, row=w_row, pady=2)

w_row = 3
w_column = 0
w_lable = tk.Label(display_data_region, text='Current Temp(°C)', borderwidth=0, relief="solid")
w_lable.grid(column=0, row=w_row, pady=2)

# create display UI
# 持溫時間
w_row = 0
w_column = 1
retaining_time_entry = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
retaining_time_entry.grid(column=w_column, row=w_row)

w_row = 1
w_column = 1
w_lable = tk.Label(display_data_region, text='Retaining Time(mins)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row, padx=20, pady=2)

# 目前輸出功率
w_row = 2
w_column = 1
current_power_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
current_power_label.grid(column=w_column, row=w_row, pady=2)

w_row = 3
w_column = 1
w_lable = tk.Label(display_data_region, text='Current Power(W)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row, pady=2)

# create display UI
# 冷卻時間
w_row = 0
w_column = 2
cooling_time_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid")
cooling_time_label.grid(column=w_column, row=w_row)

w_row = 1
w_column = 2
w_lable = tk.Label(display_data_region, text='Cooling Time(mins)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row, padx=2, pady=2)

# 加熱器累積耗能功率
w_row = 2
w_column = 2
electric_energy_consumption_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
electric_energy_consumption_label.grid(column=w_column, row=w_row, pady=2)

w_row = 3
w_column = 2
w_lable = tk.Label(display_data_region, text='electric_energy_consumption(度)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row, pady=2)

# 加熱指示燈
w_row = 0
w_column = 4
heating_light = tk.Canvas(display_data_region, width=20, height=20)
heating_light.create_oval(5, 5, 20, 20, width=2, fill='red', outline='red')
heating_light.grid(column=w_column, row=w_row)
w_row = 0
w_column = 5
w_lable = tk.Label(display_data_region, text='加熱', borderwidth=1, relief="solid")
w_lable.grid(column=w_column, row=w_row, pady=0)

# 持溫指示燈
w_row = 1
w_column = 4
retaining_light = tk.Canvas(display_data_region, width=20, height=20)
retaining_light.create_oval(5, 5, 20, 20, width=2, fill='red', outline='red')
retaining_light.grid(column=w_column, row=w_row)
w_row = 1
w_column = 5
w_lable = tk.Label(display_data_region, text='持溫', borderwidth=1, relief="solid")
w_lable.grid(column=w_column, row=w_row, pady=0)

# 冷卻指示燈
w_row = 2
w_column = 4
cooling_light = tk.Canvas(display_data_region, width=20, height=20)
cooling_light.create_oval(5, 5, 20, 20, width=2, fill='red', outline='red')
cooling_light.grid(column=w_column, row=w_row)
w_row = 2
w_column = 5
w_lable = tk.Label(display_data_region, text='冷卻', borderwidth=1, relief="solid")
w_lable.grid(column=w_column, row=w_row, pady=0)
display_data_region.pack(side='top', anchor='w')

##### 設定區域設定
setting_data_region = tk.LabelFrame(root, text='設定區域', padx=10, pady=10)
setting_data_region.place(x=0, y=0)
# 目標溫度設定
w_row = 4
w_column = 0
input_temperature = tk.Entry(setting_data_region, width=entry_width, borderwidth=1, relief="solid")
input_temperature.grid(column=w_column, row=w_row, pady=2)
w_row = 4
w_column = 1
set_temperature_value = tk.StringVar()  # 建立文字變數
set_temperature_value.set('0.0')      # 設定內容
set_temperature = tk.Label(setting_data_region, textvariable=set_temperature_value, width=entry_width, borderwidth=1, relief="solid")
set_temperature.grid(column=w_column, row=w_row, pady=2)
w_row = 5
w_column = 0
w_lable = tk.Label(setting_data_region, text='Temp. Input(°C)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)
w_row = 5
w_column = 1
w_lable = tk.Label(setting_data_region, text='Set Temp.(°C)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)
w_row = 4
w_column = 2
set_temperature_button = tk.Button(setting_data_region, text="確認", command=set_input_temperature)
set_temperature_button.grid(column=w_column, row=w_row)
setting_data_region.pack(side='top', anchor='w')

# 加熱時間設定
w_row = 6
w_column = 0
input_heating_time = tk.Entry(setting_data_region, width=entry_width, borderwidth=1, relief="solid")
input_heating_time.grid(column=w_column, row=w_row, pady=2)
w_row = 6
w_column = 1
set_heating_time_value = tk.StringVar()  # 建立文字變數
set_heating_time_value.set('00:00')      # 設定內容
set_heating_time = tk.Label(setting_data_region, textvariable=set_heating_time_value, width=entry_width, borderwidth=1, relief="solid")
set_heating_time.grid(column=w_column, row=w_row, pady=2)
w_row = 7
w_column = 0
w_lable = tk.Label(setting_data_region, text="Heating Time.(mins)", borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)
w_row = 7
w_column = 1
w_lable = tk.Label(setting_data_region, text='Set Heating Time.(mins)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)
w_row = 6
w_column = 2
set_heating_time_button = tk.Button(setting_data_region, text="確認", command=set_heat_time)
set_heating_time_button.grid(column=w_column, row=w_row)
setting_data_region.pack(side='left', anchor='w')

# 開始運行/停止運行-按鈕
w_row = 8
w_column = 0
START_button = tk.Button(setting_data_region, text="START", bg="#00FF00", command=lambda: [Start(), start_fun1(), start_fun2(), start_fun3(), stop_fun4()])
START_button.grid(column=w_column, row=w_row)
w_row = 8
w_column = 1
STOP_button = tk.Button(setting_data_region, text="STOP", bg="#FF0000", command=lambda: [Stop(root), start_fun4()])
STOP_button.grid(column=w_column, row=w_row)
setting_data_region.pack(side='top', anchor='w')

# 低壓加溫/高壓加溫模式切換按鈕
w_row = 8
w_column = 2
Voltage48_button = tk.Button(setting_data_region, text="48v模式", bg="#00AAFF", command=VoltageChange_on)
Voltage48_button.grid(column=w_column, row=w_row)
w_row = 9
w_column = 2
Voltage36_button = tk.Button(setting_data_region, text="36v模式", bg="#007700", command=VoltageChange_off)
Voltage36_button.grid(column=w_column, row=w_row)
setting_data_region.pack(side='top', anchor='w')

# 加溫模式切換按鈕
w_row = 8
w_column = 3
HeatingOn_button = tk.Button(setting_data_region, text="加熱開啟", bg="#00AAFF", command=Heating_on)
HeatingOn_button.grid(column=w_column, row=w_row)
w_row = 9
w_column = 3
HeatingOff_button = tk.Button(setting_data_region, text="停止加熱", bg="#007700", command=Heating_off)
HeatingOff_button.grid(column=w_column, row=w_row)
setting_data_region.pack(side='top', anchor='w')

init()

# main
root.mainloop()