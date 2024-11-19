import RPi.GPIO as GPIO
import cv2
import pytesseract
import numpy as np
import time
import spidev
import os
from datetime import datetime, timedelta

# Các thông số cấu hình
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6
TRIG_PIN = 15
ECHO_PIN = 4
SAVE_PATH = "captured_images"
ARCHIVE_PATH = "archive_images"
LCD_WIDTH = 16
LCD_CHR = True
LCD_CMD = False
E_PULSE = 0.0005
E_DELAY = 0.0005
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

# SPI LED matrix
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# Biến toàn cục
parking_lot = [0xFF] * 8
total_slots = 64
available_slots = total_slots
car_data = {}  # Lưu thông tin ảnh và biển số xe
cleanup_time = 24 * 60 * 60  # Thời gian chờ xóa ảnh (24 giờ)

# GPIO khởi tạo
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BTS["BT1"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTS["BT2"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Tạo thư mục nếu chưa tồn tại
os.makedirs(SAVE_PATH, exist_ok=True)
os.makedirs(ARCHIVE_PATH, exist_ok=True)

# Hàm tiền xử lý ảnh
def preprocess_image(image):
    # Chuyển sang grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Lọc nhiễu
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Tăng cường độ sắc nét
    edged = cv2.Canny(blurred, 50, 200)
    return edged

# Hàm trích xuất biển số từ ảnh
def extract_license_plate(image):
    processed = preprocess_image(image)
    # Thay đổi để xử lý 3 giá trị trả về từ cv2.findContours
    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    for contour in contours:
        # Xác định vùng bao quanh (bounding box)
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = w / float(h)
        if 2 < aspect_ratio < 6:  # Biển số thường có tỉ lệ chiều rộng/chiều cao hợp lý
            plate = image[y:y+h, x:x+w]
            plate_resized = cv2.resize(plate, (400, 100))  # Chuẩn hóa kích thước
            return pytesseract.image_to_string(plate_resized, config='--psm 7').strip()
    return None
    
# Hàm lưu ảnh
def save_image(slot, image, is_entry=True):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    status = "entry" if is_entry else "exit"
    filename = f"slot_{slot}_{status}_{timestamp}.jpg"
    filepath = os.path.join(SAVE_PATH, filename)
    cv2.imwrite(filepath, image)
    return filename, filepath

# Cập nhật số ô trống trên LED matrix
def update_matrix():
    for row in range(8):
        spi.xfer2([row + 1, parking_lot[row]])

# Xóa ảnh sau 24 giờ
def cleanup_old_images():
    now = time.time()
    for filename in os.listdir(ARCHIVE_PATH):
        filepath = os.path.join(ARCHIVE_PATH, filename)
        if os.path.isfile(filepath):
            file_time = os.path.getctime(filepath)
            if now - file_time > cleanup_time:
                os.remove(filepath)

# Xử lý khi xe vào
def car_enter():
    global available_slots
    if available_slots == 0:
        print("Bãi đã đầy!")
        return
    for row in range(8):
        for col in range(8):
            if parking_lot[row] & (1 << (7 - col)):
                parking_lot[row] &= ~(1 << (7 - col))
                slot = row * 8 + col
                capture = cv2.VideoCapture(0)
                ret, frame = capture.read()
                if ret:
                    filename, filepath = save_image(slot, frame, is_entry=True)
                    plate = extract_license_plate(frame)
                    car_data[slot] = {'plate': plate, 'entry_time': datetime.now(), 'image': filepath}
                    available_slots -= 1
                    print(f"Xe vào: Biển số {plate}")
                capture.release()
                update_matrix()
                return

# Xử lý khi xe ra
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
                capture = cv2.VideoCapture(0)
                ret, frame = capture.read()
                if ret:
                    plate_out = extract_license_plate(frame)
                    if car_data[slot]['plate'] == plate_out:
                        archive_file = os.path.join(ARCHIVE_PATH, os.path.basename(car_data[slot]['image']))
                        os.rename(car_data[slot]['image'], archive_file)
                        print(f"Xe ra: Biển số {plate_out}. Ảnh lưu tại {archive_file}")
                        del car_data[slot]
                        available_slots += 1
                capture.release()
                update_matrix()
                return

# Chương trình chính
def main():
    try:
        while True:
            cleanup_old_images()
            if GPIO.input(BTS["BT1"]) == GPIO.LOW:
                car_enter()
            elif GPIO.input(BTS["BT2"]) == GPIO.LOW:
                car_exit()
            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        spi.close()

if __name__ == "__main__":
    main()
