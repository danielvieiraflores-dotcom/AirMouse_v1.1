import time
import serial
from pynput.mouse import Controller as Mouse, Button
from pynput.keyboard import Listener, Key

# ======== SETTINGS ========
SERIAL_PORT = 'COM7'   # Windows: COMx | macOS/Linux: /dev/ttyACM0 or /dev/ttyUSB0
BAUD = 115200
POLL_HZ = 120          # how often we read & move
DEADZONE = 5//2.8         # ignore tiny rotations (degrees/sec)
GAIN_X = 1.5           # sensitivity
GAIN_Y = 1.5
SMOOTH = 0.45          # 0..1 (higher = smoother/slower response)
INVERT_X = False
INVERT_Y = False       # upward rotation moves cursor up
# Hotkeys: ESC quits; F8 toggle enable; F9 recalibrate baseline
# ==========================

mouse = Mouse()
running = True
enabled = True

# filtered values & baseline
xf = 0.0
yf = 0.0
base_x = 0.0
base_y = 0.0
needs_recalibration = False

def lerp(a, b, t):
    return a + (b - a) * t

def parse_line(line):
    # expecting: "X:1.234 Y:-5.678 B:0"
    try:
        x_start = line.find('X:')
        y_start = line.find('Y:')
        b_start = line.find('B:')
        if x_start == -1 or y_start == -1 or b_start == -1:
            return None, None, None

        x_str = line[x_start+2:y_start]
        y_str = line[y_start+2:b_start]
        b_str = line[b_start+2:]

        x = float(x_str)
        y = float(y_str)
        b = int(b_str.strip())
        return x, y, b
    except (ValueError, IndexError):
        return None, None, None

def on_press(key):
    global running, enabled, needs_recalibration
    if key == Key.esc:
        print("\n[EXIT] ESC pressed.")
        running = False
        return False
    if key == Key.f8:
        enabled = not enabled
        print(f"[TOGGLE] Mouse control {'ON' if enabled else 'OFF'}")
    if key == Key.f9:
        print("[CALIBRATE] Hold device still; baseline will be captured.")
        needs_recalibration = True

listener = Listener(on_press=on_press)
listener.start()

def main():
    global running, enabled, xf, yf, base_x, base_y, needs_recalibration
    left_button_pressed = False
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.01)
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return
        
    time.sleep(2.0)
    print("IMU mouse active. ESC=quit, F8=toggle, F9=recalibrate baseline")

    baseline_set = False
    last_update_time = time.perf_counter()
    interval = 1.0 / POLL_HZ

    try:
        while running:
            # Read all available lines from serial to get the latest data
            latest_line = None
            while ser.in_waiting > 0:
                try:
                    latest_line = ser.readline().decode('utf-8', errors='ignore')
                except serial.SerialException:
                    time.sleep(0.1) # Avoid busy-looping on serial error
                    break

            if latest_line:
                x, y, b = parse_line(latest_line)
                if x is not None:
                    if not baseline_set or needs_recalibration:
                        base_x, base_y = x, y
                        baseline_set = True
                        needs_recalibration = False
                        print(f"[BASELINE] x={base_x:.3f} y={base_y:.3f}")

                    # centered values
                    x -= base_x
                    y -= base_y

                    # deadzone
                    if abs(x) < DEADZONE: x = 0.0
                    if abs(y) < DEADZONE: y = 0.0

                    # invert axes if desired
                    if INVERT_X: x = -x
                    if INVERT_Y: y = -y
                    
                    # Update smoothed values with the latest sensor data
                    xf = lerp(xf, x, 1.0 - SMOOTH)
                    yf = lerp(yf, y, 1.0 - SMOOTH)

                    # Handle button press
                    if b == 1 and not left_button_pressed:
                        mouse.press(Button.left)
                        left_button_pressed = True
                    elif b == 0 and left_button_pressed:
                        mouse.release(Button.left)
                        left_button_pressed = False

            # Update mouse position at a fixed interval
            current_time = time.perf_counter()
            if (current_time - last_update_time) >= interval:
                last_update_time = current_time
                if enabled:
                    dx = xf * GAIN_X
                    dy = yf * GAIN_Y
                    if abs(dx) > 0.01 or abs(dy) > 0.01:
                        mouse.move(dx, dy)
                        print(f"X: {dx:.3f}\tY: {dy:.3f}", end='\r')
            
            # Small sleep to prevent CPU maxing out if no serial data
            time.sleep(0.001)

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        listener.stop()
        print("Clean exit.")

if __name__ == "__main__":
    main()
