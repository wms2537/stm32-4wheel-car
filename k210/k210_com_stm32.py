import sensor, image, time, lcd
from machine import UART
from fpioa_manager import fm

fm.register(10,fm.fpioa.UART2_TX, force=True)
fm.register(11,fm.fpioa.UART2_RX, force=True)

uart_usbttl = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)


clock = time.clock()

# Thresholds for yellow, grey, and blue colors
grey_threshold = (4, 20, -12, 2, -3, 11)
yellow_threshold = (25, 46, -13, -3, 31, 52)
blue_threshold = (4, 22, 4, 20, -42, -20)

# Setup camera
sensor.reset(freq=24000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(0)
sensor.set_saturation(2)
sensor.set_jb_quality(100)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking

# Setup LCD
lcd.init()
lcd.clear(lcd.RED)
lcd.set_backlight(100)
lcd.rotation(1)


# Function to draw a red box around the grid area
def draw_grid_area(img, top_left, bottom_right):
    square_color = (255, 0, 0)  # Red color for the square
    img.draw_rectangle((top_left[0], top_left[1], bottom_right[0] - top_left[0], bottom_right[1] - top_left[1]), color=square_color)

# Initialize variables for grid size detection
top_left_detected = False
bottom_right_detected = False
top_left = [0, 0]
bottom_right = [0, 0]
grid_width = 0
grid_height = 0
box_width = 0
box_height = 0

# Initialize grid map
# empty = 0; grey = 1; yellow = 2; blue = 3; corner = 5;  
grid_map = [[0 for _ in range(8)] for _ in range(8)]

# Initialize variables
continuous_values = []  # To store continuous readings
same_value_counter = 0  # To count the occurrence of the same value

#start_scanning
start_str = "hi"
read_data = uart_usbttl.read()
read_str = read_data.decode('utf-8')
while True:
    if read_str == write_str:break
    else: continue
        
# Function to get the most frequent value in a list
def most_frequent(List):
    return max(set(List), key=List.count)

# Main loop
for n in range(30):
    clock.tick()
    img = sensor.snapshot()
    img.rotation_corr(z_rotation=90.0)
    lcd.display(img)

    # Detect grey blobs
    blobs = img.find_blobs([grey_threshold], pixels_threshold=100, area_threshold=100, merge=True)
    if blobs:
        for blob in blobs:
            # Determine top-left corner
            if not top_left_detected or (blob.cx() <= top_left[0] and blob.cy() <= top_left[1]):
                top_left = (blob.cx(), blob.cy())
                top_left_detected = True
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))

            # Determine bottom-right corner
            if not bottom_right_detected or (blob.cx() >= bottom_right[0] and blob.cy() >= bottom_right[1]):
                bottom_right = (blob.cx(), blob.cy())
                bottom_right_detected = True
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))

        # Draw the red box around the determined grid area
        if top_left_detected and bottom_right_detected:
            draw_grid_area(img, top_left, bottom_right)

    # Check if top-left and bottom-right detected
    if top_left_detected and bottom_right_detected:
        # Calculate grid dimensions and box size
        grid_width = abs(top_left[0] - bottom_right[0])
        grid_height = abs(top_left[1] - bottom_right[1])
        if grid_width != 0 and grid_height != 0:
            box_width = grid_width / 8
            box_height = grid_height / 8

# Main loop for scanning colored boxes within the grid
for n in range(50):
    clock.tick()
    img = sensor.snapshot()
    img.rotation_corr(z_rotation=90.0)
    lcd.display(img)

    # Scan for values if grid dimensions are valid
    if box_width != 0 and box_height != 0:
        # Reset continuous values for each frame
        continuous_values = []

        # Scan other colored boxes within the grid
        for color, threshold in zip(['grey', 'yellow', 'blue'], [grey_threshold, yellow_threshold, blue_threshold]):
            blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200, merge=True)
            if blobs:
                for blob in blobs:
                    # Calculate box indices within grid boundaries
                    box_index_x = int((blob.cx() - top_left[0]) / box_width)
                    box_index_y = int((blob.cy() - top_left[1]) / box_height)

                    # Clamp indices within grid boundaries
                    box_index_x = max(0, min(box_index_x, 7))
                    box_index_y = max(0, min(box_index_y, 7))

                    if 0 <= box_index_x < 8 and 0 <= box_index_y < 8:
                        grid_map[box_index_y][box_index_x] = {'grey': 1, 'yellow': 2, 'blue': 3}.get(color)
                        continuous_values.append(grid_map[box_index_y][box_index_x])

                    # Draw detected blobs with respective colors
                    rect_color = (200, 200, 200) if color == 'grey' else (255, 255, 0) if color == 'yellow' else (0, 0, 255)
                    img.draw_rectangle(blob.rect(), color=rect_color)
                    img.draw_cross(blob.cx(), blob.cy(), color=rect_color)

        # Set top left and bottom right corners as 'X'
        if top_left_detected:
            grid_map[0][0] = 5
        if bottom_right_detected:
            grid_map[7][7] = 5

        # Update continuous value list
        if continuous_values:
            most_freq_value = most_frequent(continuous_values)
            continuous_values = []  # Reset continuous values

            if 0 <= box_index_x < 8 and 0 <= box_index_y < 8:
                if grid_map[box_index_y][box_index_x] == most_freq_value:
                    same_value_counter += 1
                else:
                    same_value_counter = 0

                if same_value_counter >= 5:
                    grid_map[box_index_y][box_index_x] = most_freq_value
                    same_value_counter = 0  # Reset counter after updating the value
                elif same_value_counter < 5:
                    grid_map[box_index_y][box_index_x] = 0

        # Print the grid map
        print("Grid Map:")
        for row in grid_map:
            print(row)


flattened_buf = [item for sublist in grid_map for item in sublist]
uart_usbttl.write(bytearray(flattened_buf))
    




