#!/usr/bin/python3

# External module imports
import tkinter as tk
import threading
import RPi.GPIO as GPIO
from time import time, sleep
from datetime import datetime
# from tkinter import messagebox
import spidev


# Pin Definitons:
# spi SCLK Broadcom pin 11 (P1 pin 23)
# spi CE0 Broadcom pin 8 (P1 pin 24)
# spi MISE Broadcom pin 9 (P1 pin 21)
PWMPin = 13 # Broadcom pin 18 PWM1 (P1 pin 33),  PWM 只有12, 13
SSRPin = 17 # Broadcom pin 17 (P1 pin 11)
relayPin = 27 # Broadcom pin 27 (P1 pin 13)
dc = 0 # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(SSRPin, GPIO.OUT) # LED pin set as output
GPIO.setup(relayPin, GPIO.OUT) # LED pin set as output
GPIO.setup(PWMPin, GPIO.OUT) # PWM pin set as output
pwm = GPIO.PWM(PWMPin, 50)  # Initialize PWM on PWMPin 100Hz frequency
pwm.start(0)
##GPIO.setup(butPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Button pin set as input w/ pull-up

# Initial state for LEDs:
GPIO.output(SSRPin, GPIO.LOW)
GPIO.output(relayPin, GPIO.LOW)

SSR_ON_TIME = 0.3
SSR_OFF_TIME = 0.3
input_temperature = 80.0
temperature_max = 0
temperature_min = temperature_max - 10
heat_time = time()                      # 開始加熱時間
elapsed_time = 0                        # 已加熱秒數
estimated_heat_time = "10:00"           # 預計加熱時長
heat_power = 12*7.5                     # 固定加熱功率
heat_kwh = 0.0                          # 累計加熱功率，度數
wind_kwh = 0.0                          # 累計鼓風機功率，度數
wind_watt = 0.0                         # 鼓風機功率

log_fun1 = False
log_fun2 = False
log_fun3 = False


# function
# 加熱器累計耗能
def add_heat_kwh(w_heat_kwh):
    global heat_kwh
    heat_kwh = heat_kwh + w_heat_kwh
    return heat_kwh

# 鼓風機累計耗能
def add_wind_kwh():
    global wind_kwh, wind_watt
    # wind_kwh = heat_kwh + w_wind_kwh
    wtime = heat_time_entry.get()
    minutes, seconds = map(int, wtime.split(':'))
    total_seconds = get_total_seconds(minutes, seconds)
    wind_kwh = wind_watt * total_seconds / 1000 / 3600

    return wind_kwh

# 離開程式
def exit_program(root):
    global fun1_thread, fun2_thread, fun3_thread
    if fun1_thread:
        stop_fun1()
    if fun2_thread:
        stop_fun2()
    if fun3_thread:
        stop_fun3()
    root.quit()


# 處理目前溫度顯示
def fun1():
    global elapsed_time, estimated_heat_time, fun1_thread, fun2_thread, fun3_thread
    while not fun1_exit_flag.is_set():
        if log_fun1:
            print("執行 fun1:目前溫度 執行序...")
        minutes, seconds = map(int, estimated_heat_time.split(':'))
        mins, secs = divmod(elapsed_time, 60)
        
        # 更新加熱時間
        # print(f'加熱時間: {mins}:{secs}')
        update_heat_time(f'{mins}:{secs}')

        if (mins*60+secs) >= (minutes*60+seconds):
            if fun2_thread:
                stop_fun2()
            if fun3_thread:
                stop_fun3()
            stop_gpio()
            break
        sleep(1)

# 處理加熱時間 heat time，按下SSR啟動
def fun3():
    global heat_time, estimated_heat_time, elapsed_time
    current_time = time()
    minutes, seconds = map(int, estimated_heat_time.split(':'))
    total_seconds = get_total_seconds(minutes, seconds)
    while not fun3_exit_flag.is_set():
        elapsed_time = int(time() - current_time)
        if log_fun3:
            print("執行 fun3:加熱時間 執行序...")
            print(f'elapsed_time: {elapsed_time}')
        sleep(1)


# 處理SSR on 的時間
def fun2():
    global SSR_ON_TIME, SSR_OFF_TIME, input_temperature, heat_time, wind_watt, wind_kwh
    heat_time = datetime.now()          # 開始加熱時間
    while not fun2_exit_flag.is_set():
        print("執行 fun2:SSR 執行序...")
        
        temp = read_max6675(0, 0)
        # print("temp: ", temp)
        # print("input_temperature: ", input_temperature)
        update_current_temperature(f'{temp:.1f}')

        # if temp > input_temperature and SSR_ON_TIME >= 0.5:
        # SSR_ON_TIME = SSR_ON_TIME - 0.5
        
        if temp > input_temperature:
            SSR_ON_TIME = 0
            SSR_OFF_TIME = 0.3
        elif temp <= input_temperature:
            SSR_ON_TIME = SSR_ON_TIME + 0.1

        # wade: ToDo
        # 開啟SSR 並計算功率
        # print(f'SSR_ON_TIME: {SSR_ON_TIME:.2f}')
        print('SSR_ON_TIME:', SSR_ON_TIME)
        SSR_on()
        sleep(SSR_ON_TIME)
        # print(f'SSR_OFF_TIME: {SSR_OFF_TIME:.2f}')
        print('SSR_OFF_TIME:', SSR_OFF_TIME)
        SSR_off()
        sleep(SSR_OFF_TIME)

        heat_watt = get_heat_watt()
        # print('heat watt:', heat_watt)
        update_fixed_heat_watt(heat_watt)
        # print(f'加熱kwh：{heat_watt/1000/3600}')
        add_heat_kwh(heat_watt/1000/3600)
        update_heat_kwh()

        # 更新鼓風機累計耗能
        # print(f'鼓風機kwh：{wind_kwh}')
        add_wind_kwh()
        update_wind_kwh()
        

        # # # 更新目前溫度
        # temp = read_max6675(0, 0)
        # print("temp: ", temp)
        # print("input_temperature: ", input_temperature)
        # update_current_temperature(f'{temp:.1f}')


        # if temp > input_temperature and SSR_ON_TIME >= 0.5:
        #     SSR_ON_TIME = SSR_ON_TIME - 0.5
        # elif temp <= input_temperature:
        #     SSR_ON_TIME = SSR_ON_TIME + 0.2
    stop_gpio()

def get_heat_watt():
    global SSR_ON_TIME, SSR_OFF_TIME, heat_power
    return heat_power * (SSR_ON_TIME)/(SSR_ON_TIME + SSR_OFF_TIME)

def get_total_seconds(minutes, seconds):
    return minutes * 60 + seconds

def get_wind_LPM_to_watt_and_pwm(lpm):
    if lpm <= 0:
        return (0, 0)
    elif lpm <= 500:
        return (11.4, 26)
    elif lpm <= 560:
        return (12.1, 28)
    elif lpm <= 620:
        return (12.7, 30)
    elif lpm <= 680:
        return (13.4, 32)
    
    
    elif lpm <= 740:
        return (14.2, 34)
    elif lpm <= 800:
        return (14.9, 36)
    elif lpm <= 860:
        return (15.6, 38)
    elif lpm <= 920:
        return (16.5, 40)
    elif lpm <= 980:
        return (17.5, 42)
    elif lpm <= 1040:
        return (18.5, 44)
    elif lpm <= 1100:
        return (19.4, 46)
    elif lpm <= 1160:
        return (20.5, 48)
    elif lpm <= 1220:
        return (21.5, 50)
    elif lpm <= 1280:
        return (22.6, 52)
    elif lpm <= 1340:
        return (23.7, 54)
    elif lpm <= 1400:
        return (25	, 56)
    elif lpm <= 1460:
        return (26.3, 58)
    elif lpm <= 1520:
        return (27.5, 59)
    elif lpm <= 1580:
        return (28.6, 60)
    elif lpm <= 1640:
        return (29.5, 62)
    elif lpm <= 1700:
        return (30.5, 64)
    elif lpm <= 1760:
        return (31.4, 66)
    elif lpm <= 1820:
        return (32.1, 68)
    elif lpm <= 1880:
        return (33.3, 69)
    elif lpm <= 1940:
        return (34.9, 70)
    elif lpm <= 2000:
        return (36.3, 72)
    else:
        return (36.3, 72)

    # if lpm <= 0:
    #     return (0, 0)
    # if lpm <= 453:
    #     return (14.2, 34)
    # elif lpm <= 521:
    #     return (14.9, 36)
    # elif lpm <= 589:
    #     return (15.6, 38)
    # elif lpm <= 657:
    #     return (16.5, 40)
    # elif lpm <= 703:
    #     return (17.5, 42)
    # elif lpm <= 748:
    #     return (18.5, 44)
    # elif lpm <= 816:
    #     return (19.4, 46)
    # elif lpm <= 907:
    #     return (20.5, 48)
    # elif lpm <= 997:
    #     return (21.5, 50)
    # elif lpm <= 1134:
    #     return (22.6, 52)
    # elif lpm <= 1270:
    #     return (23.7, 54)
    # elif lpm <= 1428:
    #     return (25	, 56)
    # elif lpm <= 1542:
    #     return (26.3, 58)
    # elif lpm <= 1655:
    #     return (27.5, 59)
    # elif lpm <= 1701:
    #     return (28.6, 60)
    # elif lpm <= 1769:
    #     return (29.5, 62)
    # elif lpm <= 1814:
    #     return (30.5, 64)
    # elif lpm <= 1859:
    #     return (31.4, 66)
    # elif lpm <= 1882:
    #     return (32.1, 68)
    # elif lpm <= 1927:
    #     return (33.3, 69)
    # elif lpm <= 1950:
    #     return (34.9, 70)
    # elif lpm <= 1973:
    #     return (36.3, 72)
    # else:
    #     return (36.3, 72)

def countdown(total_seconds):
    start_time = time.time()

    while total_seconds > 0:
        elapsed_time = time.time() - start_time
        remaining_time = max(0, total_seconds - int(elapsed_time))
        mins, secs = divmod(remaining_time, 60)
        timeformat = '{:02d}:{:02d}'.format(mins, secs)
        print(timeformat, end='\r')
        time.sleep(1)

def help():
    print('1: Relay on')
    print('2: Relay off')
    print('3: SSR on')
    print('4: SSR off')
    print('5: set SSR on time')
    print('6: set SSR off time')
    print('7: SSR start')
    print('8: show SSR on/off time')

def init():
    global SSR_ON_TIME, SSR_OFF_TIME, input_temperature, temperature_max, temperature_min, heat_time, elapsed_time, estimated_heat_time, heat_power, heat_kwh, wind_kwh, wind_watt
    SSR_ON_TIME = 3.0
    SSR_OFF_TIME = 3.0
    input_temperature = 80.0
    temperature_max = 0
    temperature_min = temperature_max - 10
    heat_time = time()                      # 開始加熱時間
    elapsed_time = 0                        # 已加熱秒數
    estimated_heat_time = "10:00"           # 預計加熱時長
    heat_power = 12*7.5                     # 固定加熱功率
    heat_kwh = 0.0                          # 累計加熱功率，度數
    wind_kwh = 0.0                          # 累計鼓風機功率，度數
    wind_watt = 0.0                         # 鼓風機功率

def PWM_off():
    global dc
    dc = 0
    # wind_LPM_Scale.delete(0, 'end')
    # wind_LPM_Scale.insert(0, str(dc))
    wind_LPM_Scale.set(0)
    print(f'dc: {dc}')
    pwm.start(dc)

def relay_on():
    GPIO.output(relayPin, GPIO.HIGH)

def relay_off():
    GPIO.output(relayPin, GPIO.LOW)

def show_SSR_on_off_time():
    print('SSR_ON_TIME = ', SSR_ON_TIME)
    print('SSR_OFF_TIME = ', SSR_OFF_TIME)

def set_heat_time():
    global heat_time, estimated_heat_time
    estimated_heat_time = str(estimated_heat_time_scale.get()) + ':00'
    print('輸入加熱時間 heat_time: ', estimated_heat_time)
    heat_time_end_entry.delete(0, 'end')
    heat_time_end_entry.insert(0, '目標時間  ' + str(estimated_heat_time))

def set_input_temperature():
    global input_temperature
    input_temperature = float(input_temperature_scale.get())
    print('輸入溫度 input_temperature: ', input_temperature)
    target_temperature_entry.delete(0, 'end')
    target_temperature_entry.insert(0, str(input_temperature))

def set_PWM_dc():
    global dc, wind_watt
    # w_wind_LPM = int(wind_LPM_Scale.get())
    # if w_wind_LPM > 2000:
    #     update_wind_LPM_Scale('2000')
    # elif w_wind_LPM < 100:
    #     update_wind_LPM_Scale('100')
    lpm = get_wind_LPM_to_watt_and_pwm(int(wind_LPM_Scale.get()))
    wind_watt = lpm[0]
    dc = lpm[1]
    print(f'dc: {dc}, wind_watt:{wind_watt}')
    # msg = messagebox.showinfo("LPM",f"LPM:{lpm} watt:{wind_watt}")
    pwm.start(dc)

def set_SSR_on_time():
    global SSR_ON_TIME
    SSR_ON_TIME = float(set_SSR_on_time_entry.get())
    print('set SSR_ON_TIME: ', SSR_ON_TIME)

def set_SSR_off_time():
    global SSR_OFF_TIME
    SSR_OFF_TIME = float(set_SSR_off_time_entry.get())
    print('set SSR_OFF_TIME: ', SSR_OFF_TIME)

def SSR_on():
    GPIO.output(SSRPin, GPIO.HIGH)

def SSR_off():
    GPIO.output(SSRPin, GPIO.LOW)

def stop_gpio():
    relay_off()
    SSR_off()
    PWM_off()
    init()

def read_max6675(spi_channel, spi_device):
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device)
    spi.max_speed_hz = 50000  # You may need to adjust this value based on your hardware

    try:
        while True:
            # Read the raw ADC value from the MAX6675
            adc_data = spi.xfer2([0, 0])            

            # Combine the two bytes into a 12-bit value
            output = ((adc_data[0] & 0x7F) << 5) | (adc_data[1] >> 3)

            # Convert the raw value to Celsius
            temperature_celsius = output * 0.25
            temperature_celsius = (temperature_celsius - 2) * 97 / 100
            
            # Convert to Fahrenheit
            temperature_fahrenheit = (temperature_celsius * 9 / 5) + 32
            
            print(f"Temperature: {temperature_celsius:.2f} °C, {temperature_fahrenheit:.2f} °F")
            return temperature_celsius

    except KeyboardInterrupt:
        print("Temperature reading stopped.")
    finally:
        spi.close()


def update_current_temperature(temp):
    global current_temperature_label
    current_temperature_label['text'] = str(temp)

def update_heat_time(wtime):
    heat_time_entry.delete(0, 'end')
    heat_time_entry.insert(0, wtime)
    # print('time: ', strftime("%M:%S", gmtime()))

def update_heat_kwh():
    global heat_kwh
    heat_kwh_entry.delete(0, 'end')
    heat_kwh_entry.insert(0, f'{heat_kwh:0.8f}')

def update_fixed_heat_watt(watt):
    fixed_heat_watt_entry.delete(0, 'end')
    fixed_heat_watt_entry.insert(0, f"{watt:0.2f}")

def update_wind_LPM_Scale(lpm):
    wind_LPM_Scale.delete(0, 'end')
    wind_LPM_Scale.insert(0, str(lpm))
    

def update_wind_kwh():
    global wind_kwh
    wind_kwh_entry.delete(0, 'end')
    wind_kwh_entry.insert(0, f'{wind_kwh:0.8f}')


# 建立 flag
fun1_exit_flag = threading.Event()
fun2_exit_flag = threading.Event()
fun3_exit_flag = threading.Event()

# 設定 fun1的函数
fun1_thread = None  # 目前溫度 
fun2_thread = None  # SSR
fun3_thread = None  # 加熱時間


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
    exit_button['state'] = tk.DISABLED
    stop_button.configure(bg="red")

def start_fun3():
    global fun3_thread
    fun3_exit_flag.clear()
    fun3_thread = threading.Thread(target=fun3, daemon=True)
    fun3_thread.start()


# 停止fun1的函数
def stop_fun1():
    global fun1_thread
    fun1_exit_flag.set()
    fun1_thread.join()  # wait for thread stop
    print("fun1 has stopped.")

def stop_fun2():
    global fun2_thread
    fun2_exit_flag.set()
    fun2_thread.join()  # wait for thread stop
    print("fun2:SSR has stopped.")
    exit_button['state'] = tk.NORMAL
    stop_button.configure(bg="#f0f0f0")

def stop_fun3():
    global fun3_thread
    fun3_exit_flag.set()
    fun3_thread.join()  # wait for thread stop
    print("fun3:heat_time has stopped.")

def stop_all_funs():
    global estimated_heat_time
    estimated_heat_time = '00:00'


def show_threads():
    for thread in threading.enumerate(): 
        print(thread.name)

# 主要視窗設定
root = tk.Tk()
root.title("碳捕捉電控")
root.geometry('480x550+0+0')

# create routine thread
# w_lable = tk.Label(root, text='Relay 開關')
# w_lable.grid(column=0, row=0)

# start_button = tk.Button(root, text="Start Relay fun1", command=start_fun1)
# start_button.grid(column=1, row=0)

# stop_button = tk.Button(root, text="Stop Relay fun1", command=stop_fun1)
# stop_button.grid(column=2, row=0)

# create display UI
# 目前溫度
entry_width=10
w_row = 0
w_lable = tk.Label(root, text='目前溫度(°C)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

current_temperature_label = tk.Label(root, width=entry_width, borderwidth=1, relief="solid", bg='white', anchor="w")
# current_temperature_label['text'] = ''
current_temperature_label.grid(column=1, row=w_row)

# 預定溫度
w_row = 1
w_lable = tk.Label(root, text='預定溫度(°C)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

target_temperature_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
target_temperature_entry.insert(0, '')
target_temperature_entry.grid(column=1, row=w_row)

# 輸入溫度
w_row = 2
w_lable = tk.Label(root, text='輸入溫度(°C)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

# input_temperature_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# input_temperature_entry.insert(0, '80')
input_temperature_scale = tk.Scale(root, width=entry_width, borderwidth=1, relief="solid",from_=55, to=85, orient='horizontal')
input_temperature_scale.grid(column=1, row=w_row)


start_button = tk.Button(root, text="設定", command=set_input_temperature)
start_button.grid(column=2, row=w_row)


# create config UI
# 加熱時間
w_row = 3
w_lable = tk.Label(root, text='加熱時間(mins)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

heat_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
heat_time_entry.insert(0, '00:00')
heat_time_entry.grid(column=1, row=w_row)

heat_time_end_entry = tk.Entry(root, borderwidth=1, relief="solid")
heat_time_end_entry.insert(0, '目標時間  ')
heat_time_end_entry.grid(column=2, row=w_row)


# 預計加熱時間
w_row = 4
w_lable = tk.Label(root, text='預計加熱時間(分)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

# estimated_heat_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# estimated_heat_time_entry.insert(0, estimated_heat_time)
estimated_heat_time_scale = tk.Scale(root, width=entry_width, borderwidth=1, relief="solid",from_=1, to=60, orient='horizontal')
estimated_heat_time_scale.grid(column=1, row=w_row)

start_button = tk.Button(root, text="設定", command=set_heat_time)
start_button.grid(column=2, row=w_row)


# 固定加熱功率
w_row = 5
w_lable = tk.Label(root, text='即時功率狀態(W)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

fixed_heat_watt_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
fixed_heat_watt_entry.insert(0, '')
fixed_heat_watt_entry.grid(column=1, row=w_row)

# start_button = tk.Button(root, text="設定", command='')
# start_button.grid(column=2, row=w_row)

## 加熱器累計耗能
w_row = 6
w_lable = tk.Label(root, text='加熱器累計耗能(度)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

heat_kwh_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
heat_kwh_entry.insert(0, '')
heat_kwh_entry.grid(column=1, row=w_row)


# 風量
w_row = 7
w_lable = tk.Label(root, text='風量(LPM)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

# wind_LPM_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# wind_LPM_entry.insert(0, '')
# wind_LPM_entry.grid(column=1, row=w_row)
wind_LPM_Scale = tk.Scale(root, width=entry_width, borderwidth=1, relief="solid",from_=100, to=2000, orient='horizontal')
# wind_LPM_Scale.insert(0, '')
wind_LPM_Scale.grid(column=1, row=w_row)

start_button = tk.Button(root, text="設定", command=set_PWM_dc)
start_button.grid(column=2, row=w_row)


## 鼓風機累計耗能
w_row = 8
w_lable = tk.Label(root, text='鼓風機累計耗能(度)：', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

wind_kwh_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
wind_kwh_entry.insert(0, '')
wind_kwh_entry.grid(column=1, row=w_row)


# create Relay
w_row = 9
w_lable = tk.Label(root, text='Relay 開關', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

start_button = tk.Button(root, text="relay on", command=relay_on)
start_button.grid(column=1, row=w_row)

stop_button = tk.Button(root, text="relay off", command=relay_off)
stop_button.grid(column=2, row=w_row)


# create SSR
w_row = 10
w_lable = tk.Label(root, text='SSR 開關', borderwidth=1, relief="solid")
w_lable.grid(column=0, row=w_row, pady=10)

# start_button = tk.Button(root, text="Start SSR", command=start_fun2)
start_button = tk.Button(root, text="Start SSR", command=lambda: [start_fun1(), start_fun2(), start_fun3()])
start_button.grid(column=1, row=w_row)

stop_button = tk.Button(root, text="Stop SSR", command=stop_all_funs)
# stop_button = tk.Button(root, text="Stop SSR", command=lambda: [stop_fun3(), stop_fun1(), stop_fun2()])
stop_button.grid(column=2, row=w_row)

# create SSR
# w_row = 11
# w_lable = tk.Label(root, text='SSR ON 時間(秒)', borderwidth=1, relief="solid")
# w_lable.grid(column=0, row=w_row, pady=10)

# set_SSR_on_time_button = tk.Button(root, text="set SSR ON time", command=set_SSR_on_time)
# set_SSR_on_time_button.grid(column=1, row=w_row)

# set_SSR_on_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# set_SSR_on_time_entry.insert(0, str(SSR_ON_TIME))
# set_SSR_on_time_entry.grid(column=2, row=w_row)


w_row = 12
# w_lable = tk.Label(root, text='SSR OFF 時間(秒)', borderwidth=1, relief="solid")
# w_lable.grid(column=0, row=w_row, pady=10)

# set_SSR_off_time_button = tk.Button(root, text="set SSR OFF time", command=set_SSR_off_time,)
# set_SSR_off_time_button.grid(column=1, row=w_row)

# set_SSR_off_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# set_SSR_off_time_entry.insert(0, str(SSR_OFF_TIME))
# set_SSR_off_time_entry.grid(column=2, row=w_row)


# exit button
w_row = 11
exit_button = tk.Button(root, text="Exit", command=lambda: exit_program(root), width=50)
exit_button.grid(column=0, row=w_row, columnspan=3, pady=30)

# main
root.mainloop()
stop_gpio()
GPIO.cleanup()