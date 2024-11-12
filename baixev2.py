import cv2
import RPi.GPIO as GPIO
import time
import os
import pytesseract
import spidev
import numpy as np

# Đường dẫn lưu ảnh
SAVE_PATH = "./captured_images"
os.makedirs(SAVE_PATH, exist_ok=True)

# Lưu biển số và đường dẫn ảnh
car_data = {}  # Lưu {biển số: đường dẫn ảnh}

# GPIO Configurations
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6
TRIG_PIN = 15
ECHO_PIN = 4

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BTS["BT1"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTS["BT2"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Servo setup
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# LED Matrix SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# LED Matrix Data
parking_lot = [0xFF] * 8
total_slots = 64
available_slots = total_slots

# Functions for existing features
def move_servo(angle):
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def open_gate():
    move_servo(90)

def close_gate():
    move_servo(0)

def max7219_write(register, data):
    spi.xfer2([register, data])

def max7219_init():
    max7219_write(0x0C, 0x01)
    max7219_write(0x0B, 0x07)
    max7219_write(0x09, 0x00)
    max7219_write(0x0A, 0x0F)
    max7219_write(0x0F, 0x00)

def update_matrix():
    for row in range(8):
        max7219_write(row + 1, parking_lot[row])

def measure_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.2)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def lcd_clear():
    pass  # Giữ nguyên logic LCD hiện tại

def update_lcd(available_slots):
    pass  # Giữ nguyên logic LCD hiện tại

# New Feature: Recognize license plate
def detect_plate(image_path):
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    text = pytesseract.image_to_string(gray, config='--psm 8')
    plate_number = ''.join(filter(str.isalnum, text))
    return plate_number

# Capture and save image
def capture_and_save():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        file_name = f"{int(time.time())}.jpg"
        file_path = os.path.join(SAVE_PATH, file_name)
        cv2.imwrite(file_path, frame)
        print(f"Image saved: {file_path}")
        cap.release()
        return file_path
    cap.release()
    return None

# Process car entering
def car_enter():
    global available_slots
    if available_slots == 0:
        print("Bãi đã đầy!")
        return

    distance = measure_distance()
    if distance < 10:
        for row in range(8):
            for col in range(8):
                if parking_lot[row] & (1 << (7 - col)):
                    parking_lot[row] &= ~(1 << (7 - col))
                    available_slots -= 1
                    update_matrix()
                    update_lcd(available_slots)
                    open_gate()

                    # Capture and recognize license plate
                    img_path = capture_and_save()
                    if img_path:
                        plate_number = detect_plate(img_path)
                        car_data[plate_number] = img_path
                        print(f"Car entered: {plate_number}")
                    time.sleep(2)
                    close_gate()
                    return

# Process car exiting
def car_exit():
    global available_slots
    if available_slots == total_slots:
        print("Bãi trống hoàn toàn!")
        return

    for plate, path in list(car_data.items()):
        print(f"Checking plate from: {path}")
        detected_plate = detect_plate(path)
        print(f"Detected plate: {detected_plate}, Stored plate: {plate}")
        if detected_plate == plate:
            os.remove(path)
            del car_data[plate]
            available_slots += 1
            update_matrix()
            update_lcd(available_slots)
            open_gate()
            print(f"Car exited: {plate}")
            time.sleep(2)
            close_gate()
            return

# Cleanup
def cleanup():
    spi.close()
    servo.stop()
    GPIO.cleanup()

# Main
try:
    max7219_init()
    update_matrix()
    update_lcd(available_slots)

    while True:
        if GPIO.input(BTS["BT1"]) == GPIO.LOW:
            car_enter()
            time.sleep(1)
        elif GPIO.input(BTS["BT2"]) == GPIO.LOW:
            car_exit()
            time.sleep(1)

except KeyboardInterrupt:
    cleanup()
