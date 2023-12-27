from machine import Pin
from time import sleep

print("")
print("Place card before reader to read from address 0x08")
print("")

# 23 hours, 56 minutes and four seconds => 86 344 seconds
# stepper = 2038 steps
step = 2038

IN1 = Pin(2, Pin.OUT)
IN2 = Pin(3, Pin.OUT)
IN3 = Pin(4, Pin.OUT)
IN4 = Pin(5, Pin.OUT)

pins = [IN1, IN2, IN3, IN4]

sequence = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

while True:
    for step in sequence:
        for i in range(len(pins)):
            pins[i].value(step[i])
            print("Place card before reader to read from address 0x08")
            sleep(0.001)
