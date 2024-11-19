import RPi.GPIO as GPIO
import cv2
import time
import spidev
import os
import threading
from datetime import datetime, timedelta
from easyocr import Reader  # Thư viện OCR mới

# Cấu hình GPIO và các thông số
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6
TRIG_PIN = 15
ECHO_PIN = 4
SAVE_PATH = "captured_images"  # Thư mục lưu ảnh
ARCHIVE_PATH = "archive"  # Thư mục lưu ảnh chờ xóa
LCD_WIDTH = 16
LCD_CHR = True
LCD_CMD = False
E_PULSE = 0.0005
E_DELAY = 0.0005
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

# Khởi tạo SPI cho LED matrix
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# Biến toàn cục
parking_lot = [0xFF] * 8
total_slots = 64
available_slots = total_slots
car_data = {}  # Lưu thông tin {vị trí: {"image": ..., "plate": ..., "time_in": ...}}

# Khởi tạo GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BTS["BT1"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTS["BT2"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Tạo thư mục lưu ảnh nếu chưa tồn tại
os.makedirs(SAVE_PATH, exist_ok=True)
os.makedirs(ARCHIVE_PATH, exist_ok=True)

# EasyOCR reader
reader = Reader(['en'])  # Khởi tạo OCR cho tiếng Anh (phù hợp với biển số)

# Hàm điều khiển servo
def move_servo(angle):
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def open_gate():
    move_servo(90)

def close_gate():
    move_servo(0)

# Gửi dữ liệu đến LED matrix
def max7219_write(register, data):
    spi.xfer2([register, data])

def max7219_init():
    max7219_write(0x0C, 0x01)  # Bật LED
    max7219_write(0x0B, 0x07)  # Giới hạn quét = 8 hàng
    max7219_write(0x09, 0x00)  # Không sử dụng giải mã
    max7219_write(0x0A, 0x0F)  # Độ sáng tối đa
    max7219_write(0x0F, 0x00)  # Thoát chế độ test

def update_matrix():
    for row in range(8):
        max7219_write(row + 1, parking_lot[row])

# Đo khoảng cách từ cảm biến siêu âm
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

# Xử lý ảnh để nhận diện biển số
def recognize_plate(image_path):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    _, img_thresh = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    results = reader.readtext(img_thresh)
    if results:
        return results[0][1]  # Trả về chuỗi biển số đầu tiên
    return None

# Chụp ảnh xe và nhận diện biển số
def capture_image(slot):
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        print("Cannot open camera")
        return None
    ret, frame = capture.read()
    if ret:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(SAVE_PATH, f"slot_{slot}_{timestamp}.jpg")
        cv2.imwrite(filename, frame)
        plate = recognize_plate(filename)
        print(f"Biển số nhận diện: {plate}")
        return filename, plate
    capture.release()
    cv2.destroyAllWindows()
    return None, None

# Chuyển ảnh vào thư mục lưu trữ và xóa sau 24 giờ
def archive_image(file_path):
    if os.path.exists(file_path):
        os.rename(file_path, os.path.join(ARCHIVE_PATH, os.path.basename(file_path)))
        threading.Timer(86400, os.remove, args=[os.path.join(ARCHIVE_PATH, os.path.basename(file_path))]).start()

# Cho xe vào bãi
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
                    slot = row * 8 + col
                    image, plate = capture_image(slot)
                    if image and plate:
                        car_data[slot] = {"image": image, "plate": plate, "time_in": datetime.now()}
                        available_slots -= 1
                        update_matrix()
                        open_gate()
                        time.sleep(2)
                        close_gate()
                    return

# Cho xe ra khỏi bãi
def car_exit():
    global available_slots
    if available_slots == total_slots:
        print("Bãi trống!")
        return
    for row in range(8):
        for col in range(8):
            if not parking_lot[row] & (1 << (7 - col)):
                parking_lot[row] |= (1 << (7 - col))
                slot = row * 8 + col
                image, plate = capture_image(slot)
                if slot in car_data and car_data[slot]["plate"] == plate:
                    archive_image(car_data[slot]["image"])
                    del car_data[slot]
                    available_slots += 1
                    update_matrix()
                    open_gate()
                    time.sleep(2)
                    close_gate()
                return

# Chương trình chính
def main():
    max7219_init()
    try:
        while True:
            if GPIO.input(BTS["BT1"]) == GPIO.LOW:
                car_enter()
            elif GPIO.input(BTS["BT2"]) == GPIO.LOW:
                car_exit()
            time.sleep(0.1)
    except KeyboardInterrupt:
        servo.stop()
        GPIO.cleanup()
        max7219_write(0x0C, 0x00)

if __name__ == "__main__":
    main()
