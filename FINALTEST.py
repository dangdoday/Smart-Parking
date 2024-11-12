import RPi.GPIO as GPIO
import cv2
import time
import spidev
import numpy as np


# Cau hinh GPIO va cac thong so LCD
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6  # Cong dieu khien Servo
LCD_WIDTH = 16
LCD_CHR = True
LCD_CMD = False
E_PULSE = 0.0005
E_DELAY = 0.0005
LCD_LINE_1 = 0x80  # Dia chi cua dong 1 tren LCD
LCD_LINE_2 = 0xC0  # Dia chi cua dong 2 tren LCD

# Khoi tao SPI cho LED ma tran
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# Bien toan cuc
parking_lot = [0xFF] * 8  # Mo phong bai do xe voi tat ca cac den sang (tat ca vi tri trong)
total_slots = 64  # Tong so cho do xe
available_slots = total_slots  # Bien dem cho trong hien tai

# Khoi tao GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Cấu hình các nút bấm
GPIO.setup(BTS["BT1"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTS["BT2"], GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Cấu hình chân GPIO cho cảm biến khoảng cách
TRIG_PIN = 15  # Chân phát tín hiệu
ECHO_PIN = 4  # Chân nhận tín hiệu

# Cấu hình các chân của cảm biến khoảng cách
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)


# Cau hinh cong Servo
GPIO.setup(SERVO_PIN, GPIO.OUT)  # Dong co Servo
servo = GPIO.PWM(SERVO_PIN, 50)  # PWM cho Servo voi tan so 50Hz
servo.start(0)  # Bat dau voi chu ky 0 (cong ban dau dong)

# Ham dieu khien Servo den goc chi dinh
def move_servo(angle):
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Dung mot chut de Servo co thoi gian di chuyen
    servo.ChangeDutyCycle(0)  # Ngat tin hieu PWM

# Ham mo cong
def open_gate():
    move_servo(90)  # Mo cong den 90 do

# Ham dong cong
def close_gate():
    move_servo(0)  # Dong cong ve 0 do

# Ham viet du lieu ra LED ma tran
def max7219_write(register, data):
    spi.xfer2([register, data])

# Ham khoi tao LED ma tran
def max7219_init():
    max7219_write(0x0C, 0x01)  # Bat LED
    max7219_write(0x0B, 0x07)  # Gioi han quet = 8 LED
    max7219_write(0x09, 0x00)  # Che do giai ma = khong
    max7219_write(0x0A, 0x0F)  # Do sang = toi da
    max7219_write(0x0F, 0x00)  # Tat che do thu nghiem

# Ham cap nhat ma tran LED
def update_matrix():
    for row in range(8):
        max7219_write(row + 1, parking_lot[row])

# Khoi tao man hinh LCD
def lcd_init():
    for pin in LCD_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
    for byte in [0x33, 0x32, 0x28, 0x0C, 0x06, 0x01]:
        lcd_byte(byte, LCD_CMD)
    GPIO.output(LCD_PINS["BL"], True)

# Ham xoa LCD
def lcd_clear():
    lcd_byte(0x01, LCD_CMD)

# Ham gui byte lenh hoac du lieu den LCD
def lcd_byte(bits, mode):
    GPIO.output(LCD_PINS['RS'], mode)
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << (4 + bit_num)) != 0)
    time.sleep(E_DELAY)
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << bit_num) != 0)
    time.sleep(E_DELAY)
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)

# Ham hien thi chuoi len LCD
def lcd_display_string(message, line):
    lcd_byte(LCD_LINE_1 if line == 1 else LCD_LINE_2, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)

# Ham cap nhat so luong cho trong hien thi tren LCD
def update_lcd(available_slots):
    lcd_clear()
    message = f"Available: {available_slots}"
    lcd_display_string(message, 1)
    if available_slots == 0:
        lcd_display_string("Bai da day", 2)  # Thông báo ở dòng 2 khi hết chỗ
    elif available_slots == 64:
        lcd_display_string("Khong co xe", 2) 
    else:
        lcd_display_string("Bai con cho", 2)  # Xóa dòng 2 nếu còn chỗ trống

# Hàm đo khoảng cách từ cảm biến siêu âm
def measure_distance():
    # Phát xung bằng cách đặt chân TRIG xuống thấp rồi lên cao
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.2)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # Tín hiệu cao trong 10 micro giây
    GPIO.output(TRIG_PIN, False)

    # Đo thời gian tín hiệu ECHO lên cao
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    # Tính toán thời gian xung và khoảng cách
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Tốc độ âm thanh = 34300 cm/s
    distance = round(distance, 2)  # Làm tròn khoảng cách

    return distance

# Ham cho xe vao bai do
def car_enter():
    global available_slots
    if available_slots == 0:
        lcd_display_string("Bai da day", 2)  # Hien thi thong bao nhac nho nguoi dung
        time.sleep(2)  # Thoi gian cho de doc thong bao
        return
    distance = measure_distance()
    if distance < 10:  # Giả sử khoảng cách nhỏ hơn 10 cm nghĩa là có xe
        for row in range(8):
            for col in range(8):
                if parking_lot[row] & (1 << (7 - col)):  # Kiem tra vi tri trong
                    parking_lot[row] &= ~(1 << (7 - col))  # Tat den (xe vao)
                    available_slots -= 1  # Giam so cho trong
                    update_matrix()  # Cap nhat ma tran LED
                    update_lcd(available_slots)  # Cap nhat LCD
                    open_gate()  # Mo cong
                    time.sleep(2)  # Thoi gian cho xe vao
                    close_gate()  # Dong cong
                    return

# Ham cho xe ra khoi bai do
def car_exit():
    global available_slots
    if available_slots == total_slots:
        lcd_display_string("Bai trong", 2)  # hien thi thong bao nhac nho nguoi dung
        time.sleep(2)  # Thoi gian cho de doc thong bao
        return
    for row in range(8):
        for col in range(8):
            if not parking_lot[row] & (1 << (7 - col)):  # Kiem tra vi tri co xe do
                parking_lot[row] |= (1 << (7 - col))  # Bat den (xe ra)
                available_slots += 1  # Tang so cho trong
                update_matrix()  # Cap nhat ma tran LED
                update_lcd(available_slots)  # Cap nhat LCD
                open_gate()  # Mo cong
                time.sleep(2)  # Thoi gian cho xe ra
                close_gate()  # Dong cong
                return

# Ham don dep va tat SPI khi chuong trinh ket thuc
def cleanup():
    max7219_write(0x0C, 0x00)  # Tat LED ma tran
    lcd_clear()  # Xoa LCD
    servo.stop()  # Dung dong co Servo
    GPIO.cleanup()  # Don dep GPIO
    spi.close()  # Dong SPI

# Chuong trinh chinh
def main():
    max7219_init()  # Khoi tao ma tran LED
    lcd_init()  # Khoi tao LCD
    update_lcd(available_slots)  # Hien thi so cho trong ban dau

    while True:
        if GPIO.input(BTS["BT1"]) == 0 and available_slots > 0:
            car_enter()  # Cho xe vào
        elif GPIO.input(BTS["BT2"]) == 0 and available_slots < total_slots:
            car_exit()  # Cho xe ra
        time.sleep(0.1)  # thoi gian delay ngan chan bam nut lien tuc


try:
    main()
except KeyboardInterrupt:
    cleanup()  # don dep khi ngat chuong trinh
