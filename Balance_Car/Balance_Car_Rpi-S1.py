from Sensor import *
import time
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
# use raw_input function for python 2.x or input function for python3.x
port = "/dev/ttyUSB0"                #Python2软件版本用    port = raw_input('please input port No. such as com7:');*****************************************************************************************************
baud = 115200
ser = serial.Serial(port, baud, timeout=0.5)  # ser = serial.Serial('com7',115200, timeout=0.5) 
print(ser.is_open)

GPIO.setmode(GPIO.BCM)
STBY = 17
PWMA = 18
AIN1 = 14
AIN2 = 15
PWMB = 19
BIN1 = 23
BIN2 = 24

#设置 GPIO 的工作方式
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
pwma = GPIO.PWM(PWMA,500)
pwmb = GPIO.PWM(PWMB,500)

def Motor(speed):
    if(speed>0):
        if(speed>=100):
            speed=100
        pwma.ChangeDutyCycle(speed)
        pwmb.ChangeDutyCycle(speed)
        GPIO.output(14, GPIO.LOW)
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(23, GPIO.LOW)
    elif{speed<0}:
        if(speed<=-100):
            speed=-100
        pwma.ChangeDutyCycle(-speed)
        pwmb.ChangeDutyCycle(-speed)
        GPIO.output(14, GPIO.HIGH)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(23, GPIO.HIGH)
    else:
        GPIO.output(14, GPIO.LOW)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)



import threading
import RPi.GPIO as GPIO #霍尔脉冲读取函数
import time

gpioA=20
gpioB=21
GPIO.setwarnings(False)  #忽略警告
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpioA,GPIO.IN,pull_up_down=GPIO.PUD_UP)   #通过18号引脚读取A脉冲数据
GPIO.setup(gpioB,GPIO.IN,pull_up_down=GPIO.PUD_UP)   #通过35号引脚读取B脉冲数据

global counter1
encoder_precision = 1*500*30# 19 #编码器精度=倍频数*编码器精度（电机驱动线数)*电机减速比
encoder_freq = 500
counter1=0     #A相脉冲初值
counter2=0      #B相脉冲初值
speed=0

def target_enc_process(speed):
    '''
    函数功能：将目标速度值转化为目标编码器值
    入口参数：目标速度值
    返回值  ：目标编码器值
    # 目标编码器值=(目标速度*编码器精度)/(轮子周长*控制频率)
    '''
    enc = (speed*encoder_precision) / ( (2*3.1416*0.067) * encoder_freq)
    return enc


def my_callback1(channel1):         #边缘检测回调函数
    global counter1#设置为全局变量
    if GPIO.input(gpioA):
        if not GPIO.input(gpioB):
            counter1 += 1
        elif GPIO.input(gpioB):
            counter1 -= 1
     
def my_callback2(channel2):            #这里的channel1和channel2无须赋确定值，但不能不写
    global counter2
    if GPIO.event_detected(gpioB):
        counter2=counter2+1
               
GPIO.add_event_detect(gpioA,GPIO.RISING,callback=my_callback1) #在引脚上添加上升临界值检测再回调
GPIO.add_event_detect(gpioB,GPIO.RISING,callback=my_callback2)


def Interrupt():
#    print('这是中断')
    global counter1,speed
    #speed = (counter2/encoder_precision) * (2*3.1416*0.067) * encoder_freq   
    speed=counter1/500/0.01
    #print('speed:',speed)
    counter1=0
    #time.sleep(0.01)
    t = threading.Timer(0.01,Interrupt)
    t.start()   
#每过1秒切换
t = threading.Timer(0.01,Interrupt)
t.start()    

try:
    
    a=0.7
    Target_angle_y=-1.0
    Target_speed_y=0.0
    
    GPIO.output(STBY,GPIO.HIGH)
    pwma.start(10)
    pwmb.start(10)
 
    EnC_Err_Lowout_last=0
    Encoder_S=0
    
    Kp_Vertical=3.0
    Kd_Vertical=0.40
    
    Kp_Velocity=-0.20
    Ki_Velocity=-0.0008
    #Kp_Velocity=-0.005
    #Ki_Velocity=-0.0008
    
    while True:
        timestart=time.time()
        datahex = ser.read(50)
        
        time.sleep(0.0001)
        Out=DueData(datahex)
        
        GyroX=Out[3]
        GyroY=Out[4]
        AngleX=Out[6]
        AngleY=Out[7]
        
        Encoder_Err=speed-Target_speed_y
        EnC_Err_Lowout=(1-a)* Encoder_Err+a*EnC_Err_Lowout_last
        EnC_Err_Lowout_last=EnC_Err_Lowout
        Encoder_S+=EnC_Err_Lowout
        if Encoder_S>10000:
            Encoder_S=10000
        if Encoder_S<-10000:
            Encoder_S=-10000
        PWM_out=Kp_Velocity*EnC_Err_Lowout+Ki_Velocity*Encoder_S
        
        PWM_Balance=Kp_Vertical*(AngleY+(PWM_out+Target_angle_y))+Kd_Vertical*(GyroY)
        
        Motor(PWM_Balance)
        
        timeend=time.time()
        print(timeend-timestart,PWM_Balance,PWM_out,speed)
        
except KeyboardInterrupt:
    GPIO.cleanup()
    pass
    
    
