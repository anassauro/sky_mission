import RPi.GPIO as gpio
from time import sleep

class Eletroima():
    def __init__(self, pin) -> None:
        self.pin = pin
        gpio.setmode(gpio.BCM)
        gpio.setup(self.pin, gpio.OUT)
        self.state = False

    def on(self):
        gpio.output(self.pin, gpio.HIGH)
        self.state = True

    def off(self):
        gpio.output(self.pin, gpio.LOW)
        self.state = False

    def toggle(self):
        if self.state:
            self.off()
        else:
            self.on()

if __name__ == "__main__":
    eletroima = Eletroima(3)
    eletroima.on()
    time.sleep(10)
    eletroima.off()
