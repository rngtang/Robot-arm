import time
import picamera

# Create a PiCamera object
camera = picamera.PiCamera()

# Set the resolution and framerate (optional)
camera.resolution = (640, 480)  # Adjust the resolution as needed
camera.framerate = 30  # Adjust the framerate as needed

# Start previewing the camera feed on the screen
camera.start_preview()

try:
    while True:
        # Camera preview is running in the background

        # Add any other processing or actions you want to perform here

        time.sleep(1)  # You can adjust the time interval

except KeyboardInterrupt:
    pass
finally:
    # Stop the camera preview and release resources
    camera.stop_preview()
    camera.close()