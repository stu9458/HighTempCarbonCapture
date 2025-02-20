import hid
import sys
import time

'''
Company: 广东大镓传感技术有限公司
Version: V1.0
DateTime: 2024/2/23
Author: YouYou
注意：
先使用 pip install hidapi
基于python v3.8.1
电脑插入多路USB测温仪后，运行此python文件可以实现读取温度值
1S读取一次温度值
'''

def EnumDevice():
    for device_dict in hid.enumerate():
        keys = list(device_dict.keys())
        keys.sort()
        for key in keys:
            print("%s : %s" % (key, device_dict[key]))
        print()

def LoadDevice():
    try:
        print("Opening the device")

        h = hid.device()
        h.open(0xDA9A, 0x0066)  # TREZOR VendorID/ProductID

        print("Manufacturer: %s" % h.get_manufacturer_string())
        print("Product: %s" % h.get_product_string())
        print("Serial No: %s" % h.get_serial_number_string())

        # # enable non-blocking mode
        # h.set_nonblocking(1)
        #
        # # write some data to the device
        # print("Write the data")
        # h.write([0, 63, 35, 35] + [0] * 61)
        #
        # # wait
        # time.sleep(0.05)
        #
        # # read back the answer
        # print("Read the data")
        # while True:
        #     d = h.read(64)
        #     if d:
        #         print(d)
        #     else:
        #         break

        #print("Closing the device")
        #h.close()
    except IOError as ex:
        h = None
        print(ex)
        print("You probably don't have the hard-coded device.")
        print("Update the h.open() line in this script with the one")
        print("from the enumeration list output above and try again.")
    print("Done")
    return h

DATAPACKLENGTH = 64
#组装命令包
def ProduceCmd(cmds):
    cmd_data = [0]
    cnt = len(cmds)
    if(cnt < (DATAPACKLENGTH - 1)):
        cmd_data[0] = cnt
        cmd_data[1:cnt] = cmds
    else:
        #对于超过63字节的包先不展示如何操作
        cmd_data = cnt
    return cmd_data

#解析响应
def ParsePacket(response):
    response_data = []
    cnt = len(response)

#判断空包
def IsEmptyPacket(data, offset,  dlen):
    for i in range(0,dlen):
        if (data[offset + i] != 0):
            return False
    return True

#计算modbus crc16
def CRC16(data, ifrom, ito):
    auchCRCHi = \
    [
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
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
        0x40
    ]

    auchCRCLo = \
    [
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
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
        0x40
    ]

    uchCRCHi = 0xff
    uchCRCLo = 0xff
    uindex = 0

    for i in range(ifrom, (ito+1)):
        uindex = (uchCRCHi ^ data[i]) & 0xff
        uchCRCHi = (uchCRCLo ^ auchCRCHi[uindex]) & 0xff
        uchCRCLo = auchCRCLo[uindex] & 0xff
    return ((uchCRCHi << 8) | uchCRCLo) & 0xffff

#发送命令并获取数据
def SendCmdGetResp(device, cmd):
    cmds = ProduceCmd(cmd)
    device.write(cmds)
    #print("Send cmd!")
    time.sleep(0.05)

    resp = [0]
    allrecdata = [0]
    recindex = 0
    while True:
        recdata = device.read(DATAPACKLENGTH)
        if (recdata[0] == (DATAPACKLENGTH - 1)):
            # get more
            if ((not IsEmptyPacket(recdata, 1, (len(recdata) - 1))) and (recindex + DATAPACKLENGTH - 1 < 256)):
                allrecdata[recindex:(recindex+DATAPACKLENGTH - 2)] = recdata[1:(len(recdata) - 1 + 1)]
                recindex += DATAPACKLENGTH - 1
            else:
                # full
                resp[0:(recindex)] = allrecdata[0:(recindex)]
                break
        elif (recdata[0] > (DATAPACKLENGTH - 1)):
            # error
            if (recindex > 0):
                resp[0:(recindex)] = allrecdata[0:(recindex)]
            else:
                resp = recdata
            break
        elif (recdata[0] > 0):
            # rec all
            if (recindex + recdata[0] < 256):
                allrecdata[recindex:(recindex+recdata[0]-1)] = recdata[1:(recdata[0] + 1)]
                recindex += recdata[0]
            resp[0:(recindex)] = allrecdata[0:(recindex)]
            break
    return resp

#将Modbus大端序字节数据转化为Int
def  GetIntFromBigEndianArray(data, sindex):
    bytes = [0]
    if (len(data) >= sindex + 4):
        bytes[0:4] = data[sindex:(sindex + 4)]
        return int.from_bytes(bytes, byteorder='big', signed=True)
    return None

def ReadTemp(device):
#1	2	3	4	5	6	9	10	11-63
#01	04	10	18	00	02	F5	0C	00
    #读取8个通道的温度，多路usb有8通道
    #Modbus RTU寄存器地址0x1018,连续读取16个字（一个通道温度数据占用两个字）
    cmd = [0x01, 0x04, 0x10, 0x18, 0x00, 0x10, 0x75, 0x01]
    cmd_packet = ProduceCmd(cmd)
    resp = SendCmdGetResp(device, cmd_packet)

    dcrc = ((resp[len(resp) - 2] * 256 + resp[len(resp) - 1]) & 0xFFFF)
    crc = CRC16(resp, 0, len(resp) - 3)
    if (dcrc == crc):
        i = 3
        while i < (3+4*7+1):
            value = GetIntFromBigEndianArray(resp, i)
            # 读取到温度通道的寄存器进行大小端转化后
            # 寄存器值需要乘以寄存器值的单位转化为实际温度值，温度单位为0.01℃
            value = value / 100.0
            print("T{0}:{1} ℃".format((int)((i-3)/4+1),value))
            i = i + 4
    else:
        print("Failed to check crc!")

def Main():
    device = LoadDevice()
    #EnumDevice()
    if device is None:
        print("No device!")
    else:
        while True:
            ReadTemp(device)
            time.sleep(1)
        print("Closing the device")
        device.close()

Main()