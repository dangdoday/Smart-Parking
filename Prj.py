import RPi.GPIO as GPIO
import cv2
import time
import spidev
import os
import shutil
from datetime import datetime, timedelta
import pytesseract

# GPIO Pins
LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26}
SERVO_PIN = 6
TRIG_PIN = 15
ECHO_PIN = 4

# Constants
SAVE_PATH = "captured_images"
TEMP_SAVE_PATH = "temp_images"
LCD_WIDTH = 16
E_PULSE = 0.0005
E_DELAY = 0.0005
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

# SPI for LED matrix
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# Parking lot status
parking_lot = [0xFF] * 8
total_slots = 64
available_slots = total_slots
car_images = {}
license_plates = [[""] * 8 for _ in range(8)]  # 2D array to store license plates

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(BTS["BT1"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BTS["BT2"], GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

# Create folders if not exist
for folder in [SAVE_PATH, TEMP_SAVE_PATH]:
    if not os.path.exists(folder):
        os.makedirs(folder)

# Helper Functions
def get_current_time():
    """Return current time as a formatted string."""
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def move_servo(angle):
    """Move servo to a specific angle."""
    duty_cycle = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def open_gate():
    move_servo(90)

def close_gate():
    move_servo(0)

def max7219_write(register, data):
    """Send data to LED matrix."""
    spi.xfer2([register, data])

def max7219_init():
    """Initialize LED matrix."""
    max7219_write(0x0C, 0x01)
    max7219_write(0x0B, 0x07)
    max7219_write(0x09, 0x00)
    max7219_write(0x0A, 0x0F)
    max7219_write(0x0F, 0x00)

def update_matrix():
    """Update LED matrix with parking lot status."""
    for row in range(8):
        max7219_write(row + 1, parking_lot[row])

def measure_distance():
    """Measure distance using ultrasonic sensor."""
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

def capture_image(slot, is_exit=False):
    """Capture image and save with timestamp."""
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        print("Cannot open camera")
        return False
    ret, frame = capture.read()
    if ret:
        timestamp = get_current_time()
        filename = f"car_slot_{slot}_{'exit' if is_exit else 'entry'}_{timestamp}.jpg"
        full_path = os.path.join(SAVE_PATH if not is_exit else TEMP_SAVE_PATH, filename)
        cv2.imwrite(full_path, frame)
        if not is_exit:
            car_images[slot] = full_path
        print(f"Image saved as {full_path}")
        return full_path
    capture.release()
    cv2.destroyAllWindows()

def delete_image(slot):
    """Delete image file for a specific slot."""
    if slot in car_images:
        try:
            os.remove(car_images[slot])
            print(f"Deleted image: {car_images[slot]}")
            del car_images[slot]
        except FileNotFoundError:
            print(f"File not found: {car_images[slot]}")

def extract_license_plate(image_path):
    """Extract license plate from an image."""
    try:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        plate = pytesseract.image_to_string(thresh, config='--psm 8')
        return ''.join(filter(str.isalnum, plate)).upper()
    except Exception as e:
        print(f"Error extracting license plate: {e}")
        return ""

def display_on_lcd(line1, line2):
    """Display text on LCD."""
    lcd_clear()
    lcd_write(line1, LCD_LINE_1)
    lcd_write(line2, LCD_LINE_2)

def clean_temp_images():
    """Delete images older than 24 hours in the temp folder."""
    now = datetime.now()
    for file in os.listdir(TEMP_SAVE_PATH):
        file_path = os.path.join(TEMP_SAVE_PATH, file)
        if os.path.isfile(file_path):
            file_time = datetime.fromtimestamp(os.path.getctime(file_path))
            if now - file_time > timedelta(hours=24):
                os.remove(file_path)
                print(f"Deleted expired image: {file_path}")

def car_enter():
    """Handle car entering the parking lot."""
    global available_slots
    if available_slots == 0:
        print("Parking lot full!")
        display_on_lcd("Parking Full", "Please Wait")
        return

    distance = measure_distance()
    if distance < 10:
        for row in range(8):
            for col in range(8):
                if parking_lot[row] & (1 << (7 - col)):
                    parking_lot[row] &= ~(1 << (7 - col))
                    slot = row * 8 + col
                    image_path = capture_image(slot)
                    available_slots -= 1
                    update_matrix()

                    license_plate = extract_license_plate(image_path)
                    license_plates[row][col] = {"plate": license_plate, "entry_time": get_current_time()}
                    print(f"Car entered slot {slot}, plate: {license_plate}")

                    display_on_lcd(f"Slot: {slot}", f"Plate: {license_plate}")
                    open_gate()
                    time.sleep(2)
                    close_gate()
                    return

def car_exit():
    """Handle car exiting the parking lot."""
    global available_slots
    if available_slots == total_slots:
        print("Parking lot empty!")
        display_on_lcd("Parking Empty", "Please Enter")
        return

    for row in range(8):
        for col in range(8):
            if not parking_lot[row] & (1 << (7 - col)):
                slot = row * 8 + col
                temp_image_path = capture_image(slot, is_exit=True)
                temp_license_plate = extract_license_plate(temp_image_path)
                original_data = license_plates[row][col]

                if temp_license_plate == original_data["plate"]:
                    print(f"Car exited slot {slot}, plate: {temp_license_plate}")
                    license_plates[row][col] = ""
                    parking_lot[row] |= (1 << (7 - col))
                    available_slots += 1
                    update_matrix()

                    shutil.move(temp_image_path, TEMP_SAVE_PATH)
                else:
                    print(f"Plate mismatch at slot {slot}! Detected: {temp_license_plate}")

                display_on_lcd(f"Slot: {slot}", "Car Exited")
                open_gate()
                time.sleep(2)
                close_gate()
                return

def main():
    """Main function to handle car parking system."""
    max7219_init()
    try:
        while True:
            clean_temp_images()
            if GPIO.input(BTS["BT1"]) == GPIO.LOW:
                car_enter()
            elif GPIO.input(BTS["BT2"]) == GPIO.LOW:
                car_exit()
            display_on_lcd(f"Available:", f"Slots: {available_slots}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        servo.stop()
        GPIO.cleanup()
        max7219_write(0x0C, 0x00)

if __name__ == "__main__":
    main()
