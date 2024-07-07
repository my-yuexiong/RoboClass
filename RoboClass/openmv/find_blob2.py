import sensor
import time
import image

from machine import UART

black_threshold   = (20, 0)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)
clock = time.clock()

uart = UART(3, 9600)
self_x_max = 180
self_y_min = 140
self_x = 160
self_y = 200

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

while True:
    clock.tick()
    img = sensor.snapshot()

    x_sum = 0

    img_height = img.height()
    region_height = img_height // 5

    for i in range(5):
        y_start = i * region_height
        if i == 4:
            y_end = img_height
        else:
            y_end = (i + 1) * region_height

        roi = (0, y_start, img.width(), y_end - y_start)

        blobs = img.find_blobs([black_threshold], roi=roi, pixels_threshold=200, area_threshold=200, merge=True)
        if blobs:
            max_blob=find_max(blobs)
            img.draw_rectangle(max_blob.rect())
            img.draw_cross(max_blob.cx(), max_blob.cy())
            pcx = max_blob.cx()
#            print(f"x of roi[i]:{pcx}")
            x_sum = x_sum + pcx

    x_average = x_sum // 5
#    print(f"average of xs:{x_average}")
    error = self_x - x_average
    print(f"error:{error}")
    formatted_string = f"{error}\n"
    uart.write(formatted_string)
    time.sleep_ms(100)
