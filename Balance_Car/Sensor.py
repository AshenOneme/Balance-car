#coding:UTF-8
#=================================================================================================================
#=================================================================================================================

# import lib.device_model as deviceModel
# from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
# from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver
# def AccelerationCalibration(device):
#     """
#     加计校准
#     :param device: 设备模型
#     :return:
#     """
#     device.AccelerationCalibration()                 # 加计校准
#     print("加计校准结束")
# device = deviceModel.DeviceModel(
# "我的JY901",
# WitProtocolResolver(),
# JY901SDataProcessor(),
# "51_0")
# device.serialConfig.portName = "/dev/ttyUSB0"
# device.serialConfig.baud = 9600
# device.openDevice()                                 #打开串口
# AccelerationCalibration(device)
# device.closeDevice()

#=================================================================================================================
#=================================================================================================================

import serial

ser=None                   #定义一个全局串口
 
ACCData=[0.0]*10
GYROData=[0.0]*10
AngleData=[0.0]*10          
FrameState = 0            #通过0x后面的值判断属于哪一种情况
Bytenum = 0               #读取到这一段的第几位
CheckSum = 0              #求和校验位         
 
a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3
def DueData(inputdata):   #新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global  FrameState    #在局部修改全局变量，要进行global的定义
    global  Bytenum
    global  CheckSum
    global  a
    global  w
    global  Angle
    global Out
    for data in inputdata:  #在输入的数据进行遍历
        #Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
        if FrameState==0:   #当未确定状态的时候，进入以下判断
            if data==0x55 and Bytenum==0: #0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum=data
                Bytenum=1
                continue
            elif data==0x51 and Bytenum==1:#在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum+=data
                FrameState=1
                Bytenum=2
            elif data==0x52 and Bytenum==1: #同理
                CheckSum+=data
                FrameState=2
                Bytenum=2
            elif data==0x53 and Bytenum==1:
                CheckSum+=data
                FrameState=3
                Bytenum=2
        elif FrameState==1: # acc    #已确定数据代表加速度
            
            if Bytenum<10:            # 读取8个数据
                ACCData[Bytenum-2]=data # 从0开始
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):  #假如校验位正确
                    a = get_acc(ACCData)
                CheckSum=0                  #各数据归零，进行新的循环判断
                Bytenum=0
                FrameState=0
        elif FrameState==2: # gyro
            
            if Bytenum<10:
                GYROData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    w = get_gyro(GYROData)
                CheckSum=0
                Bytenum=0
                FrameState=0
        elif FrameState==3: # angle
            
            if Bytenum<10:
                AngleData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Angle = get_angle(AngleData)
                    d = list(a)+list(w)+list(Angle)
                    Out=d
                    #print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f"%d)
                CheckSum=0
                Bytenum=0
                FrameState=0
    return d
 
def get_acc(datahex):  
    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]
    
    k_acc = 16.0
 
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc
    
    return acc_x,acc_y,acc_z
 
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro
    return gyro_x,gyro_y,gyro_z
 
 
def get_angle(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return angle_x,angle_y,angle_z

# import global_var
# global_var._init()
# global_var.set_value('Out',Out)

if __name__=='__main__':
    import time

    # use raw_input function for python 2.x or input function for python3.x
    global serial                                                          #局部设置全局变量串口：打开串口
    port = "/dev/ttyUSB0"                #Python2软件版本用    port = raw_input('please input port No. such as com7:');*****************************************************************************************************
    baud = 115200
    ser = serial.Serial(port, baud, timeout=0.5)  # ser = serial.Serial('com7',115200, timeout=0.5) 
    print(ser.is_open)

    while True:
        global Out
        time_start=time.time()
        datahex = ser.read(53)
        
        #time.sleep(0.01)
        DueData(datahex)
        time_end=time.time()
        
        AccelX=Out[0]
        AccelY=Out[1]
        AngleX=Out[6]
        AngleY=Out[7]
        
        print(time_end-time_start, AccelX,AccelY,AngleX,AngleY)
# 
#         
#     
# def getData(key):
#     return 'value'
# 
# def setData(addr,sValue):          #设置传感器 :第一个参数表示寄存器地址；第二个参数表示设置值
#     #nihao
#     tempBytes = [0xff,0xaa,addr,sValue&0xff,sValue>>8]
#     success_bytes = ser.write(tempBytes)
