import RPi.GPIO as GPIO
import time

# Cau hinh chan cho LCD, cam bien hong ngoai, cac nut nhan va Servo
LCD_PINS = {
    'RS': 23, 'E': 27, 'D4': 17, 'D5': 18, 'D6': 14, 'D7': 3, 'BL': 2  # BL duoc noi vao GPIO 2
}

IR_SS = 22  # Chan cua cam bien hong ngoai
BT1 = 21  # Chan cua nut nhan 1 (xe vao)
BT2= 26  # Chan cua nut nhan 2 (xe ra)
SERVO_PIN = 6  # Chan dieu khien dong co Servo (dong/mo cong)

LCD_WIDTH = 16  # So ky tu toi da tren moi dong cua LCD
LCD_CHR = True  # Dang gui du lieu
LCD_CMD = False  # Dang gui lenh
LCD_LINE_1 = 0x80  # Dia chi cua dong 1 tren LCD
LCD_LINE_2 = 0xC0  # Dia chi cua dong 2 tren LCD

E_PULSE = 0.0005  # Thoi gian xung cho chan enable (E)
E_DELAY = 0.0005  # Thoi gian cho giua cac thao tac

# Khoi tao GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR_SS, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Cau hinh cam bien hong ngoai
GPIO.setup(BT1, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Cau hinh nut nhan 1
GPIO.setup(BT2, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Cau hinh nut nhan 2
GPIO.setup(SERVO_PIN, GPIO.OUT)  # Cau hinh chan dieu khien cho Servo

# Cấu hình chân GPIO cho cảm biến khoảng cách
TRIG_PIN = 15  # Chân phát tín hiệu
ECHO_PIN = 4  # Chân nhận tín hiệu

# Cấu hình các chân của cảm biến khoảng cách
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Khoi tao dong co Servo
servo = GPIO.PWM(SERVO_PIN, 50)  # PWM cho Servo voi tan so 50Hz
servo.start(0)  # Bat dau voi chu ky 0 (cong ban dau dong)


# Ham dieu khien Servo den goc chi dinh
def move_servo(angle):
    duty_cycle = 2 + (angle / 18)  # Tinh toan chu ky PWM dua tren goc
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Dung lai mot chut de Servo co thoi gian di chuyen
    servo.ChangeDutyCycle(0)  # Ngat tin hieu sau khi Servo da di chuyen


# Ham mo cong
def open_gate():
    move_servo(90)  # Quay Servo den goc 90 do (mo cong)


# Ham dong cong
def close_gate():
    move_servo(0)  # Quay Servo ve goc 0 do (dong cong)


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


# Ham khoi tao man hinh LCD
def lcd_init():
    for pin in LCD_PINS.values():
        GPIO.setup(pin, GPIO.OUT)

    # Chuoi khoi tao LCD
    for byte in [0x33, 0x32, 0x28, 0x0C, 0x06, 0x01]:
        lcd_byte(byte, LCD_CMD)


# Ham xoa man hinh LCD
def lcd_clear():
    lcd_byte(0x01, LCD_CMD)


# Ham gui du lieu/lenh den LCD
def lcd_byte(bits, mode):
    GPIO.output(LCD_PINS['RS'], mode)  # Chon che do du lieu hoac lenh

    # Gui nibble cao (cac bit 4-7)
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << (4 + bit_num)) != 0)

    # Kich hoat chan enable
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)

    # Gui nibble thap (cac bit 0-3)
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << bit_num) != 0)

    # Kich hoat chan enable
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)


# Ham hien thi chuoi ky tu tren LCD
def lcd_display_string(message, line):
    lcd_byte(LCD_LINE_1 if line == 1 else LCD_LINE_2, LCD_CMD)

    for char in message:
        lcd_byte(ord(char), LCD_CHR)


# Ham chinh dieu khien he thong bai do xe
def main():
    lcd_init()  # Khoi tao man hinh LCD
    lcd_display_string("Parking System", 1)
    lcd_display_string("Ready...", 2)
    GPIO.output(LCD_PINS["BL"], True)  # Bat den nen cho LCD
    time.sleep(1)

    car_in_park = False  # Bien de theo doi trang thai xe trong bai do

    while True:
        # Kiem tra xem cam bien hong ngoai co phat hien xe khong
        if GPIO.input(IR_SS) == 0 and not car_in_park:
            lcd_display_string("Car Detected", 2)
            car_in_park = True
            time.sleep(1)

        # Nut nhan 1 (Xe vao bai do)
        if GPIO.input(BT1) == 0 and car_in_park:
            lcd_display_string("Car Entered", 2)
            open_gate()  # Mo cong
            time.sleep(2)  # Cho cho xe vao
            close_gate()  # Dong cong sau khi xe vao
            car_in_park = True
            time.sleep(1)

        # Nut nhan 2 (Xe roi khoi bai do)
        if GPIO.input(BT2) == 0 and car_in_park:
            lcd_display_string("Car Left", 2)
            open_gate()  # Mo cong
            time.sleep(2)  # Cho cho xe ra
            close_gate()  # Dong cong sau khi xe roi
            car_in_park = False
            time.sleep(1)

        # Xoa man hinh LCD sau moi thao tac
        lcd_clear()
        lcd_display_string("Ready...", 2)


# Chay chuong trinh
try:
    main()
except KeyboardInterrupt:
    servo.stop()  # Dung dong co Servo
    GPIO.cleanup()  # Don dep cac thiet lap GPIO khi thoat
