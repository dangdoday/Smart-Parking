import RPi.GPIO as GPIO
import cv2
import time
import spidev
import os

# Cấu hình GPIO và các thông số
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6
TRIG_PIN = 15
ECHO_PIN = 4
SAVE_PATH = "captured_images"  # Thư mục lưu ảnh
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
car_images = {}  # Lưu thông tin ảnh ứng với vị trí xe

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
if not os.path.exists(SAVE_PATH):
    os.makedirs(SAVE_PATH)

# Hàm điều khiển servo
def move_servo(angle):
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

# Mở và đóng cổng
def open_gate():
    move_servo(90)

def close_gate():
    move_servo(0)

# Gửi dữ liệu đến LED matrix
def max7219_write(register, data):
    spi.xfer2([register, data])

# Khởi tạo LED matrix
def max7219_init():
    max7219_write(0x0C, 0x01)  # Bật LED
    max7219_write(0x0B, 0x07)  # Giới hạn quét = 8 hàng
    max7219_write(0x09, 0x00)  # Không sử dụng giải mã
    max7219_write(0x0A, 0x0F)  # Độ sáng tối đa
    max7219_write(0x0F, 0x00)  # Thoát chế độ test

# Cập nhật LED matrix
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

# Chụp ảnh và lưu vào thư mục
def capture_image(slot):
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        print("Cannot open camera")
        return False
    ret, frame = capture.read()
    if ret:
        filename = os.path.join(SAVE_PATH, f"car_slot_{slot}.jpg")
        cv2.imwrite(filename, frame)
        car_images[slot] = filename  # Lưu tên file ứng với vị trí xe
        print(f"Image saved as {filename}")
    capture.release()
    cv2.destroyAllWindows()

# Xóa ảnh khi xe rời khỏi vị trí
def delete_image(slot):
    if slot in car_images:
        try:
            os.remove(car_images[slot])
            print(f"Deleted image: {car_images[slot]}")
            del car_images[slot]
        except FileNotFoundError:
            print(f"File not found: {car_images[slot]}")

# Cho xe vào bãi đỗ
def car_enter():
    global available_slots
    if available_slots == 0:
        print("Bãi đã đầy!")
        return
    distance = measure_distance()
    if distance < 10:  # Kiểm tra xe ở gần
        for row in range(8):
            for col in range(8):
                if parking_lot[row] & (1 << (7 - col)):
                    parking_lot[row] &= ~(1 << (7 - col))
                    slot = row * 8 + col  # Tính vị trí của xe
                    capture_image(slot)  # Chụp ảnh xe
                    available_slots -= 1
                    update_matrix()
                    open_gate()
                    time.sleep(2)
                    close_gate()
                    return

# Cho xe ra khỏi bãi đỗ
def car_exit():
    global available_slots
    if available_slots == total_slots:
        print("Bãi trống!")
        return
    for row in range(8):
        for col in range(8):
            if not parking_lot[row] & (1 << (7 - col)):
                parking_lot[row] |= (1 << (7 - col))
                slot = row * 8 + col  # Tính vị trí của xe
                delete_image(slot)  # Xóa ảnh xe
                available_slots += 1
                update_matrix()
                open_gate()
                time.sleep(2)
                close_gate()
                return

# Chương trình chính
def main():
    max7219_init()  # Khởi tạo LED matrix
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
        max7219_write(0x0C, 0x00)  # Tắt LED matrix

if __name__ == "__main__":
    main()
