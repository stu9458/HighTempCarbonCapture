#!/usr/bin/python3

# External module imports
import tkinter as tk
import threading
# import RPi.GPIO as GPIO
from time import time, sleep
from datetime import datetime
# from tkinter import messagebox
# import spidev


def relay_on():
    # GPIO.output(relayPin, GPIO.HIGH)
    print("OKOK")

def relay_off():
    # GPIO.output(relayPin, GPIO.LOW)
    print("OKOK")

def set_PWM_dc():
    print("OKOK")
    # global dc, wind_watt
    # # w_wind_LPM = int(wind_LPM_Scale.get())
    # # if w_wind_LPM > 2000:
    # #     update_wind_LPM_Scale('2000')
    # # elif w_wind_LPM < 100:
    # #     update_wind_LPM_Scale('100')
    # lpm = get_wind_LPM_to_watt_and_pwm(int(wind_LPM_Scale.get()))
    # wind_watt = lpm[0]
    # dc = lpm[1]
    # print(f'dc: {dc}, wind_watt:{wind_watt}')
    # # msg = messagebox.showinfo("LPM",f"LPM:{lpm} watt:{wind_watt}")
    # pwm.start(dc)

def set_heat_time():
    print("OKOK")
    # global heat_time, estimated_heat_time
    # estimated_heat_time = str(estimated_heat_time_scale.get()) + ':00'
    # print('輸入加熱時間 heat_time: ', estimated_heat_time)
    # heat_time_end_entry.delete(0, 'end')
    # heat_time_end_entry.insert(0, '目標時間  ' + str(estimated_heat_time))

def set_input_temperature():
    global input_temperature
    global set_temperature_value
    value = float(input_temperature.get())
    print('輸入溫度 input_temperature: ', value)
    set_temperature_value.set(str(value))

# 主要視窗設定
root = tk.Tk()
root.title("高溫感應式加熱模組電控")
root.geometry('700x400')

display_data_region = tk.LabelFrame(root, text='顯示區域', padx=10, pady=10)
display_data_region.place(x=0, y=0)

# create display UI
entry_width=10
# 加熱時間
w_row = 0
w_column = 0
heat_time_entry = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
heat_time_entry.grid(column=w_column, row=w_row)

w_row = 1
w_lable = tk.Label(display_data_region, text='Heating Time(mins)', borderwidth=0, relief="solid")
w_lable.grid(column=w_column, row=w_row)


# 當前溫度
w_row = 2
w_column = 0
current_temperature_label = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
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
cooling_time_entry = tk.Label(display_data_region, width=entry_width, borderwidth=1, relief="solid", anchor="w")
cooling_time_entry.grid(column=w_column, row=w_row)

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

setting_data_region = tk.LabelFrame(root, text='設定區域', padx=10, pady=10)
setting_data_region.place(x=0, y=0)
# 設定溫度
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


# # # 預定溫度
# # w_row = 1
# # w_lable = tk.Label(root, text='預定溫度(°C)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # target_temperature_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # target_temperature_entry.insert(0, '')
# # target_temperature_entry.grid(column=1, row=w_row)
# #
#
# #
# #
# # start_button = tk.Button(root, text="設定", command=set_input_temperature)
# # start_button.grid(column=2, row=w_row)
# #
# #
# # # 預計加熱時間
# # w_row = 4
# # w_lable = tk.Label(root, text='預計加熱時間(分)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # # estimated_heat_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # # estimated_heat_time_entry.insert(0, estimated_heat_time)
# # estimated_heat_time_scale = tk.Scale(root, width=entry_width, borderwidth=1, relief="solid",from_=1, to=60, orient='horizontal')
# # estimated_heat_time_scale.grid(column=1, row=w_row)
# #
# # start_button = tk.Button(root, text="設定", command=set_heat_time)
# # start_button.grid(column=2, row=w_row)
# #
# #
# # # 固定加熱功率
# # w_row = 5
# # w_lable = tk.Label(root, text='即時功率狀態(W)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # fixed_heat_watt_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # fixed_heat_watt_entry.insert(0, '')
# # fixed_heat_watt_entry.grid(column=1, row=w_row)
# #
# # # start_button = tk.Button(root, text="設定", command='')
# # # start_button.grid(column=2, row=w_row)
# #
# # ## 加熱器累計耗能
# # w_row = 6
# # w_lable = tk.Label(root, text='加熱器累計耗能(度)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # heat_kwh_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # heat_kwh_entry.insert(0, '')
# # heat_kwh_entry.grid(column=1, row=w_row)
# #
# #
# # # 風量
# # w_row = 7
# # w_lable = tk.Label(root, text='風量(LPM)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # # wind_LPM_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # # wind_LPM_entry.insert(0, '')
# # # wind_LPM_entry.grid(column=1, row=w_row)
# # wind_LPM_Scale = tk.Scale(root, width=entry_width, borderwidth=1, relief="solid",from_=100, to=2000, orient='horizontal')
# # # wind_LPM_Scale.insert(0, '')
# # wind_LPM_Scale.grid(column=1, row=w_row)
# #
# # start_button = tk.Button(root, text="設定", command=set_PWM_dc)
# # start_button.grid(column=2, row=w_row)
# #
# #
# # ## 鼓風機累計耗能
# # w_row = 8
# # w_lable = tk.Label(root, text='鼓風機累計耗能(度)：', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # wind_kwh_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # wind_kwh_entry.insert(0, '')
# # wind_kwh_entry.grid(column=1, row=w_row)
# #
# #
# # # create Relay
# # w_row = 9
# # w_lable = tk.Label(root, text='Relay 開關', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # start_button = tk.Button(root, text="relay on", command=relay_on)
# # start_button.grid(column=1, row=w_row)
# #
# # stop_button = tk.Button(root, text="relay off", command=relay_off)
# # stop_button.grid(column=2, row=w_row)
# #
# #
# # # create SSR
# # w_row = 10
# # w_lable = tk.Label(root, text='SSR 開關', borderwidth=1, relief="solid")
# # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # # start_button = tk.Button(root, text="Start SSR", command=start_fun2)
# # ## start_button = tk.Button(root, text="Start SSR", command=lambda: [start_fun1(), start_fun2(), start_fun3()])
# # start_button.grid(column=1, row=w_row)
# #
# # ## stop_button = tk.Button(root, text="Stop SSR", command=stop_all_funs)
# # # stop_button = tk.Button(root, text="Stop SSR", command=lambda: [stop_fun3(), stop_fun1(), stop_fun2()])
# # stop_button.grid(column=2, row=w_row)
# #
# # # create SSR
# # # w_row = 11
# # # w_lable = tk.Label(root, text='SSR ON 時間(秒)', borderwidth=1, relief="solid")
# # # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # # set_SSR_on_time_button = tk.Button(root, text="set SSR ON time", command=set_SSR_on_time)
# # # set_SSR_on_time_button.grid(column=1, row=w_row)
# #
# # # set_SSR_on_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # # set_SSR_on_time_entry.insert(0, str(SSR_ON_TIME))
# # # set_SSR_on_time_entry.grid(column=2, row=w_row)
# #
# #
# # w_row = 12
# # # w_lable = tk.Label(root, text='SSR OFF 時間(秒)', borderwidth=1, relief="solid")
# # # w_lable.grid(column=0, row=w_row, pady=10)
# #
# # # set_SSR_off_time_button = tk.Button(root, text="set SSR OFF time", command=set_SSR_off_time,)
# # # set_SSR_off_time_button.grid(column=1, row=w_row)
# #
# # # set_SSR_off_time_entry = tk.Entry(root, width=entry_width, borderwidth=1, relief="solid")
# # # set_SSR_off_time_entry.insert(0, str(SSR_OFF_TIME))
# # # set_SSR_off_time_entry.grid(column=2, row=w_row)
# #
# #
# # # exit button
# # w_row = 11
# # ## exit_button = tk.Button(root, text="Exit", command=lambda: exit_program(root), width=50)
# # ## exit_button.grid(column=0, row=w_row, columnspan=3, pady=30)

# main
root.mainloop()