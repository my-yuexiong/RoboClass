import sensor
import time
import image

from machine import UART

#black_threshold   = (24, 100, -90, 78, -51, 127)
#black_threshold   = (22, 100, -69, 84, -115, 98)
black_threshold   = (5, 53)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_auto_whitebal(False)
clock = time.clock()  # Create a clock object to track the FPS.

uart = UART(3, 9600)
#num = 0
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
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
#    if num < 1000:
#        num = num + 1
#    elif num == 1000:
#        num = 0
#    formatted_string = f"{num}\n"
#    uart.write(formatted_string)
#    print(formatted_string)

    time.sleep_ms(100)
    blobs = img.find_blobs([black_threshold])
    if blobs:
        max_blob=find_max(blobs)
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())
        pcx = max_blob.cx()
#    for blob in blobs:
#        if blob.area() > 5000:
#            print(blob.cx())
#            print(blob.cy())
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())


#        output_str=json.dumps(max_blob.cx())
#            uart.write(output_str + '\r\n')
#        print(pcx)
    else:
        print('not found!')

