from PIL import Image, ImageDraw, ImageFont
import spidev
import time
import numpy as np
import RPi.GPIO as GPIO


LCD_PINS = {'RS': 23, 'E': 27, 'D4': 18, 'D5': 17, 'D6': 14, 'D7': 3, 'BL': 2}
BTS = {"BT1": 21, "BT2": 26, "BT3": 20, "BT4": 19}
LCD_WIDTH = 16
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
E_PULSE = 0.0005
E_DELAY = 0.0005

# Khởi tạo SPI cho LED ma trận
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0x00

# Biến toàn cục
parking_lot = [0xFF] * 8  # Mô phỏng bãi đỗ xe với tất cả các đèn sáng (tất cả vị trí trống)
total_slots = 64  # Tổng số chỗ đỗ xe
available_slots = total_slots  # Biến đếm chỗ trống hiện tại


# Hàm viết dữ liệu ra LED ma trận
def max7219_write(register, data):
    spi.xfer2([register, data])


# Hàm khởi tạo LED ma trận
def max7219_init():
    max7219_write(0x0C, 0x01)  # Turn on
    max7219_write(0x0B, 0x07)  # Scan limit = 8 LEDs
    max7219_write(0x09, 0x00)  # Decode mode = none
    max7219_write(0x0A, 0x0F)  # Intensity = maximum (0x00 -> 0x0F)
    max7219_write(0x0F, 0x00)  # Test display = off


# Hàm cập nhật ma trận LED
def update_matrix():
    for row in range(8):
        max7219_write(row + 1, parking_lot[row])


# Khởi tạo màn hình LCD
def lcd_init():
    # Đặt các chân LCD ở chế độ output
    for pin in LCD_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
    # Các lệnh khởi tạo LCD
    for byte in [0x33, 0x32, 0x28, 0x0C, 0x06, 0x01]:
        lcd_byte(byte, LCD_CMD)
    # Bật đèn nền LCD
    GPIO.output(LCD_PINS["BL"], True)


# Hàm xóa LCD
def lcd_clear():
    lcd_byte(0x01, LCD_CMD)


# Hàm gửi byte lệnh hoặc dữ liệu đến LCD
def lcd_byte(bits, mode):
    # Chọn lệnh (mode = 0) hoặc dữ liệu (mode = 1)
    GPIO.output(LCD_PINS['RS'], mode)

    # Gửi 4 bit đầu tiên
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << (4 + bit_num)) != 0)
    time.sleep(E_DELAY)
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)

    # Gửi 4 bit cuối cùng
    for bit_num in range(4):
        GPIO.output(LCD_PINS[f'D{bit_num + 4}'], bits & (1 << bit_num) != 0)
    time.sleep(E_DELAY)
    GPIO.output(LCD_PINS['E'], True)
    time.sleep(E_PULSE)
    GPIO.output(LCD_PINS['E'], False)
    time.sleep(E_DELAY)


# Hàm hiển thị chuỗi lên LCD
def lcd_display_string(message, line):
    # Chọn dòng cần hiển thị
    lcd_byte(LCD_LINE_1 if line == 1 else LCD_LINE_2, LCD_CMD)

    # Gửi từng ký tự của chuỗi
    for char in message:
        lcd_byte(ord(char), LCD_CHR)


# Hàm cập nhật số lượng chỗ trống hiển thị trên LCD
def update_lcd(available_slots):
    # Xóa màn hình trước khi hiển thị
    lcd_clear()

    # Chuẩn bị chuỗi hiển thị
    message = f"Available: {available_slots}"

    # Hiển thị chuỗi lên dòng 1 của LCD
    lcd_display_string(message, 1)


# Hàm cho xe vào bãi đỗ
def car_enter():
    global available_slots
    for row in range(8):
        for col in range(8):
            if parking_lot[row] & (1 << (7 - col)):  # Kiểm tra vị trí trống (LED đang sáng)
                parking_lot[row] &= ~(1 << (7 - col))  # Tắt đèn (xe vào)
                available_slots -= 1  # Giảm số chỗ trống
                update_matrix()  # Cập nhật ma trận LED
                update_lcd()  # Cập nhật LCD
                return


# Hàm cho xe ra khỏi bãi đỗ
def car_exit():
    global available_slots
    for row in range(8):
        for col in range(8):
            if not parking_lot[row] & (1 << (7 - col)):  # Kiểm tra vị trí có xe đỗ (LED tắt)
                parking_lot[row] |= (1 << (7 - col))  # Bật đèn (xe ra)
                available_slots += 1  # Tăng số chỗ trống
                update_matrix()  # Cập nhật ma trận LED
                update_lcd()  # Cập nhật LCD
                return


# Hàm dọn dẹp và tắt SPI khi chương trình kết thúc
def cleanup():
    max7219_write(0x0C, 0x00)  # Tắt LED ma trận
    lcd.lcd_clear()  # Xóa LCD
    spi.close()


# Chương trình chính
def main():
    max7219_init()  # Khởi tạo ma trận LED
    lcd_init()  # Khởi tạo LCD
    update_lcd()  # Hiển thị số chỗ trống ban đầu

    try:
        while True:
            action = input("Nhập '1' để xe vào, '2' để xe ra: ")
            if action == '1' and available_slots > 0:
                car_enter()  # Cho xe vào
            elif action == '2' and available_slots < total_slots:
                car_exit()  # Cho xe ra
            else:
                print("Hành động không hợp lệ hoặc bãi đã đầy/hết xe.")
    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()

