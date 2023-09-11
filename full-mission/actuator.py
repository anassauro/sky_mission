import time
import argparse
import RPi.GPIO as GPIO

# To install:
# sudo apt-get -y install rpi.gpio

class actuator():
    def __init__(self, _pin1, _pin2):
        self.isClosed = False
        self.pin1 = _pin1
        self.pin2 = _pin2

        # Use board pin numbering
        GPIO.setmode(GPIO.BOARD)
        
        # Start GPIO pins
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

    # Opens the gripper
    def open(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)

        self.isClosed = False
    
    # Closes the gripper
    def close(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)

        self.isClosed = True


if __name__ == '__main__':
    actuator = actuator()
    parser = argparse.ArgumentParser(description='Open or close the actuator')
    parser.add_argument("--open", action="store_true", help="Open the actuator")
    parser.add_argument("--close", action="store_true", help="Close the actuator")
    
    args = parser.parse_args()
    if args.open:
        actuator.open()
    elif args.close:
        actuator.close()
    else:
        print("No action specified")