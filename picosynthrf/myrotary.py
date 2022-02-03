# Frequency Synthesizer with Encoder with DM8BA10 Display
# rev 1.0 - shabaz - oct 2021

# imports
import utime
from machine import Pin, I2C

#  import countio
import math

# variables
forever = True

# board LED on the Pi Pico
boardled = Pin(25, Pin.OUT)

# rotary encoder
encpin_a = Pin(6, Pin.IN)
encpin_b = Pin(7, Pin.IN)
enc_min = 100000
enc_max = 1000e6
enc_value = 100000
enc_state = 0
enc_transitions = [
    [3,    2, 1, 0],
    [0x23, 0, 1, 0],
    [0x13, 2, 0, 0],
    [3,    5, 4, 0],
    [3,    2, 4, 0x10],
    [3,    5, 3, 0x20]]


enc_last_pos = 100000
rotval = 100000

# interrupt function for any rotation that occurs
def enc_irq(p):
    global enc_value
    global enc_state
    old = enc_value
    pinsval = (encpin_a.value() << 1) | (encpin_b.value())
    enc_state = enc_transitions[enc_state & 0x07][pinsval]
    enc_dir = enc_state & 0x30
    inc = 0
    if (enc_dir == 0x10):
        inc = 1
    elif (enc_dir == 0x20):
        inc = -1
    inc *= -1 #  reverse?
    enc_value =  min(enc_max, max(enc_min, enc_value + inc))

def enc_init():
     encpin_a.irq(enc_irq, Pin.IRQ_RISING | Pin.IRQ_FALLING)
     encpin_b.irq(enc_irq, Pin.IRQ_RISING | Pin.IRQ_FALLING)


# main program
def main():
    global enc_last_pos
    global rotval
    enc_init()

    utime.sleep(1)
    print(f"val={rotval}")  # display the initial value

    while forever:  # forever loop
        # handle rotary encoder
        enc_pos = enc_value  # read the rotary encoder value
        if enc_pos != enc_last_pos:  # encoder has been rotated!
            rotval = enc_pos
            enc_last_pos = enc_pos
            print(f"val={rotval}")
        # brief pause
        utime.sleep(0.04)  # pause briefly then loop back


# run the main program function
main()
