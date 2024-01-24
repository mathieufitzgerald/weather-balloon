# Script used to caputre image on raspberry pi camera every 60 seconds and save it locally. Sort of a backup feature in case the main camera(s) fail. System will also use the local pictures to send via LoRa.

import time
import picamera
import os

# Directory where images will be saved
image_directory = "/path/to/your/folder"

# Ensure the directory exists
if not os.path.exists(image_directory):
    os.makedirs(image_directory)

# Interval between pictures (60 seconds)
interval = 60

# Initialize the camera
with picamera.PiCamera() as camera:
    # Set camera resolution if desired (e.g., camera.resolution = (1024, 768))
    camera.resolution = (1080, 1080)
    camera.iso = 0
    camera.shutter_speed = 0
    camera.exposure_mode = 'auto'
    camera.awb_mode = 'auto'
    
    while True:
        # Create a timestamped filename
        filename = time.strftime("%Y%m%d-%H%M%S") + ".jpg"
        filepath = os.path.join(image_directory, filename)

        # Capture the image
        camera.capture(filepath)

        # Wait for the next capture
        time.sleep(interval)
