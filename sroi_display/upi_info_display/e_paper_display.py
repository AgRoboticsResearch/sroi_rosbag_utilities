from datetime import datetime  # 用于获取当前时间
from PIL import Image, ImageDraw, ImageFont
from lib.waveshare_epd import epd2in13_V4
from INA219 import INA219
import time
import shutil
import os
# 初始化e-Paper
epd = epd2in13_V4.EPD()
epd.init()
epd.Clear(0xFF)  # 清空屏幕

# 加载自定义字体并设置字体大小
font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
font_size = 24  # 设置为较大的字体大小
font = ImageFont.truetype(font_path, font_size)
font_med = ImageFont.truetype(font_path, 18)
font_small = ImageFont.truetype(font_path, 13)
font_tiny = ImageFont.truetype(font_path, 10)

def display_on_epaper_inverse(text, update_time=True):
    image = Image.new('1', (epd.height, epd.width), 0)  # 创建一个黑色背景的图像
    draw = ImageDraw.Draw(image)
    
    # 显示系统状态文本，使用大字体
    draw.text((10, 10), text, font=font, fill=255)  # 使用白色字体

    if update_time:
        # 显示时间在右下角
        current_time = datetime.now().strftime("%H:%M:%S")
        
        # 获取文本边界框的大小
        bbox = draw.textbbox((0, 0), current_time, font=font)
        time_width = bbox[2] - bbox[0]
        time_height = bbox[3] - bbox[1]

        draw.text((epd.height - time_width - 10, epd.width - time_height - 10), current_time, font=font, fill=255)  # 使用白色字体

    epd.displayPartial(epd.getbuffer(image))

def get_disk_info(verbose=False):
    total, used, free = shutil.disk_usage("/")
    total = total // (2**30)  # convert to GB
    used = used // (2**30)
    free = free // (2**30)
    if verbose:
        # Print the results in a user-friendly format
        print(f"Total space: {total} GB")
        print(f"Used space: {used} GB")
        print(f"Free space: {free} GB")
    return total, used, free

def get_battery_info(verbose=False):
    ina219 = INA219(addr=0x42)
    bus_voltage = ina219.getBusVoltage_V()             # voltage on V- (load side)
    shunt_voltage = ina219.getShuntVoltage_mV() / 1000 # voltage between V+ and V- across the shunt
    current = ina219.getCurrent_mA()                   # current in mA
    power = ina219.getPower_W()                        # power in W
    bat_percent = (bus_voltage - 6)/2.4*100
    if(bat_percent > 100):bat_percent = 100
    if(bat_percent < 0):bat_percent = 0
    if verbose:
        # INA219 measure bus voltage on the load side. So PSU voltage = bus_voltage + shunt_voltage
        #print("PSU Voltage:   {:6.3f} V".format(bus_voltage + shunt_voltage))
        #print("Shunt Voltage: {:9.6f} V".format(shunt_voltage))
        print("Load Voltage:  {:6.3f} V".format(bus_voltage))
        print("Current:       {:9.6f} A".format(current/1000))
        print("Power:         {:6.3f} W".format(power))
        print("Bat Percent:       {:3.1f}%".format(bat_percent))
        print("")
    return bus_voltage, current, power, bat_percent

def display_on_epaper(text, update_time=True, battery_info=False, iversion=False, disk_info=False, topic_info=False):
    background_color = 0 if iversion else 255
    fill_color = 255 if iversion else 0
    
    image = Image.new('1', (epd.height, epd.width), background_color)  # create a black/white image
    draw = ImageDraw.Draw(image)
    
    # system status text, using large font
    draw.text((10, 10), text, font=font, fill=fill_color)

    if update_time:
        # display time in the bottom right corner
        current_time = datetime.now().strftime("%H:%M:%S")
        
        # get the size of the text bounding box
        bbox = draw.textbbox((0, 0), current_time, font=font_med)
        time_width = bbox[2] - bbox[0]
        time_height = bbox[3] - bbox[1]
        draw.text((epd.height - time_width - 10, epd.width - time_height - 10), current_time, font=font_med, fill=fill_color)
    
    if battery_info:
        bus_voltage, current, power, bat_percent = get_battery_info(verbose=False)
        # display battery info in the bottom left corner
        battery_info_text = f"⚡:{bat_percent:.1f}% {power:.1f}W"
        bbox = draw.textbbox((0, 0), battery_info_text, font=font_small)
        battery_width = bbox[2] - bbox[0]
        battery_height = bbox[3] - bbox[1]
        draw.text((10, epd.width - battery_height - 10), battery_info_text, font=font_small, fill=fill_color)
    
    if disk_info:
        total, used, free = get_disk_info(verbose=False)
        # display disk info in the top left corner
        disk_info_text = f"Disk: {used}/{total}GB"
        bbox = draw.textbbox((0, 0), disk_info_text, font=font_small)
        disk_width = bbox[2] - bbox[0]
        disk_height = bbox[3] - bbox[1]
        draw.text((10, epd.width - 2*disk_height - 10), disk_info_text, font=font_small, fill=fill_color)
    
    if topic_info:
        text_file_path = "/home/brl/codes/rosbags/recording_status.txt"
        print("Reading from file:", text_file_path)
        topic_info_text = ""
        try:
            with open(text_file_path, "r") as f:
                topic_info_text = f.read()
        except Exception as e:
            print(f"Error reading from file: {e}")
            topic_info_text = "No topic info"
        print("Topic info:", topic_info_text)
        bbox = draw.textbbox((0, 0), topic_info_text, font=font_small)
        topic_width = bbox[2] - bbox[0]
        topic_height = bbox[3] - bbox[1]
        draw.text((10, epd.width - 4*topic_height - 10), topic_info_text, font=font_tiny, fill=fill_color)

    epd.displayPartial(epd.getbuffer(image))

if __name__ == '__main__':
    message = "Hello, World!"
    display_on_epaper(message, update_time=True, battery_info=True, disk_info=True, topic_info=True)
    time.sleep(2)

