import sensor
import image
import time
from pyb import UART, LED

# 定义颜色阈值
green_threshold = [30, 100, -64, -8, -32, 32]  # 绿色阈值
stick_aspect_ratio = 5  # 细杆的长宽比阈值

# 初始化变量
Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0

# 初始化LED
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)

def get_rangefinder(uart_line):
    try:
        if len(uart_line) < 8:
            return 0xFF
        if uart_line[0] != 0x59:
            return 0xFF
        if uart_line[1] != 0x59:
            return 0xFF
        if uart_line[2] is not None and uart_line[3] is not None:
            return uart_line[3] * 256 | uart_line[2]
    except:
        return 20
        pass

def ExceptionVar(var):
    data = [0, 0]
    if var == -1:
        data[0] = 0
        data[1] = 0
    else:
        data[0] = var & 0xFF
        data[1] = var >> 8
    return data

def UART_Send(FormType, Loaction0, Location1, range_finder=0):
    global Frame_Cnt
    global fCnt_tmp
    Frame_Head = [170, 170]  # 帧头
    Frame_End = [85, 85]  # 帧尾
    fFormType_tmp = [FormType]  # 类型
    Frame_Cnt += 1

    if Frame_Cnt > 65534:
        FrameCnt = 0

    fHead = bytes(Frame_Head)  # 转换为字节

    fCnt_tmp[0] = range_finder & 0xFF  # 距离信息低字节
    fCnt_tmp[1] = range_finder >> 8  # 距离信息高字节
    fCnt = bytes(fCnt_tmp)  # 转换为字节

    fFormType = bytes(fFormType_tmp)  # 转换为字节
    fLoaction0 = bytes(ExceptionVar(Loaction0))  # 转换为字节
    fLoaction1 = bytes(ExceptionVar(Location1))  # 转换为字节
    fEnd = bytes(Frame_End)  # 转换为字节
    FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + fLoaction1 + fEnd  # 组合帧
    return FrameBuffe

# main函数区
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # 必须关闭此功能以进行颜色追踪
sensor.set_auto_whitebal(False)  # 必须关闭此功能以进行颜色追踪
clock = time.clock()
uart = UART(3, 115200)
uart_Rangefinder = UART(1, 115200)

while(True):
    clock.tick()
    img = sensor.snapshot()

    # 找到绿色色块
    blobs = img.find_blobs([green_threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10)
    if blobs:
        green_led.on()
        for blob in blobs:
            # 计算长宽比
            aspect_ratio = blob.w() / blob.h()
            if aspect_ratio > stick_aspect_ratio:  # 筛选细杆
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                P0 = blob.cx()
                P1 = blob.cy()
                Type = 200
                print("Green Stick:", Type, P0, P1)
                uart.write(UART_Send(Type, P0, P1, rangefinder))
    else:
        green_led.off()
        Type = 0xFF
        P0 = -1
        P1 = -1
        uart.write(UART_Send(Type, P0, P1, rangefinder))

    if count % 2 == 0:
        if uart_Rangefinder.any():
            p = uart_Rangefinder.readline()
            rangefinder = get_rangefinder(p)
            print(rangefinder)
    count += 1
