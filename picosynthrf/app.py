# Frequency Synthesizer with Encoder with DM8BA10 Display
# rev 1.0 - shabaz - oct 2021

# imports
import utime
from machine import Pin, I2C, SPI
import st7789py as st7789
import vga1_16x32 as font
import random
import micropython # use this to print out RAM usage
import meteo
import monofont72 as mfont72
import monofont64 as mfont64
#import monofont56 as mfont56
#import monofont48 as mfont48
import monofont40 as mfont40
import pytext

#  import countio
import math

# variables
forever = True

# board LED on the Pi Pico
boardled = Pin(25, Pin.OUT)

print(micropython.mem_info())

# TFT
st7789_res = 12
st7789_dc  = 13
tft_width = 135
tft_height = 240
spi1 = SPI(1, baudrate=40000000, polarity=1) # SPI port 1 defaults to SCK=GP10, MOSI=GP11, MISO=GP8 (unused)
print(spi1)
tft = st7789.ST7789(spi1, tft_width, tft_height,
                          reset=Pin(st7789_res, Pin.OUT),
                          dc=Pin(st7789_dc, Pin.OUT),
                          rotation=0)
font_data = meteo
freq_ypos=30
color1 = st7789.color565(255,0,0)
color2 = st7789.color565(255,255,255)
color3 = st7789.color565(150,0,150)
hist_m_txt = "*******"
hist_h_txt = "***"
hxloc = 146

# rotary encoder
encpin_a = Pin(6, Pin.IN)
encpin_b = Pin(7, Pin.IN)
enc_min = 440000  # 440 kHz
enc_max = 149000000  # 149 MHz
enc_state = 0
enc_transitions = [
    [3, 2, 1, 0],
    [0x23, 0, 1, 0],
    [0x13, 2, 0, 0],
    [3, 5, 4, 0],
    [3, 2, 4, 0x10],
    [3, 5, 3, 0x20],
]
enc_value = 1000000  # starting frequency lets make it 1 MHz
enc_last_pos = enc_value
rotval = enc_value

# push-button for manual speed (stepped) selection
speedbutton = Pin(8, Pin.IN)
speedrange = 1  # can be set to (say) 1, 1000, 1000000 for Hz, kHz and MHz
rangexpos = [195, 191, 118, 74] # used to display a bar on the TFT display
rangeypos = [128, 10, 10, 10]
rangewidth = [20, 28, 28, 28]

# clock generator I2C interface
i2c = I2C(0, scl=Pin(5), sda=Pin(4))

# pins for DM8BA10 LCD module
lcd_data = Pin(20, Pin.OUT)
lcd_wr = Pin(21, Pin.OUT)
lcd_csn = Pin(22, Pin.OUT)

# other variables
# clock generator module
CLKGEN_ADDR = 0x60
CLKGEN_XTAL = 25000000
MSDIVREG = [26, 34, 42, 50, 58]  # PLLA, PLLB, CLK0, CLK1, CLK2
PLLA = 0
PLLB = 1
CLK0 = 2
CLK1 = 3
CLK2 = 4
CLKCTRLREG = [16, 17, 18]  # CLK0, CLK1, CLK2
p1store = [0, 0]
p2store = [0, 0]
p3store = [0, 0]
# Bytes to write for decimal place positions 1-9 from the right
LCD_DPBITS = [0x0100, 0x0200, 0x0400, 0x0001, 0x0002, 0x0004, 0x0100, 0x0200, 0x0400]
# Bytes to write for ASCII character set (partial)
LCD_CHARBITS = [
    0x5C5C,  # *
    0x1448,  # +
    0x0800,  # ,
    0x1008,  # -
    0x0000,  # . (unsupported)
    0x0810,  # /
    0xABB3,  # 0
    0x0032,  # 1
    0x93A9,  # 2
    0x81AB,  # 3
    0x302A,  # 4
    0xB18B,  # 5
    0xB30B,  # 6
    0x80A2,  # 7
    0xB3AB,  # 8
    0xB0AB,  # 9
    0xB2AA,  # A
    0x85EB,  # B
    0xA381,  # C
    0x85E3,  # D
    0xB381,  # E
    0xB280,  # F
    0xA38B,  # G
    0x322A,  # H
    0x85C1,  # I
    0x0323,  # J
    0x3214,  # K
    0x2301,  # L
    0x6232,  # M
    0x6226,  # N
    0xA3A3,  # O
    0xB2A8,  # P
    0xA3A7,  # Q
    0xB2AC,  # R
    0xB18B,  # S
    0x84C0,  # T
    0x2323,  # U
    0x2A10,  # V
    0x2A26,  # W
    0x4814,  # X
    0x3428,  # Y
    0x8991,  # Z
    0x0000,  # space
]

boardled.value(1)

# initialize the I/O pins
def io_init():
    lcd_csn.value(1)
    lcd_wr.value(0)
    lcd_data.value(0)

# interrupt function for any rotation that occurs
def enc_irq(p):
    global enc_value
    global enc_state
    pinsval = (encpin_a.value() << 1) | (encpin_b.value())
    enc_state = enc_transitions[enc_state & 0x07][pinsval]
    enc_dir = enc_state & 0x30
    inc = 0
    if enc_dir == 0x10:
        inc = 1
    elif enc_dir == 0x20:
        inc = -1
    inc *= -1  # reverse?
    enc_value = min(enc_max, max(enc_min, enc_value + inc))


def enc_init():
    encpin_a.irq(enc_irq, Pin.IRQ_RISING | Pin.IRQ_FALLING)
    encpin_b.irq(enc_irq, Pin.IRQ_RISING | Pin.IRQ_FALLING)


# sends up to 8 bits to the LCD
def lcd_sendbits(value, n):
    for i in range(n - 1, -1, -1):
        if (value & (1 << i)) == 0:
            lcd_data.value(0)
        else:
            lcd_data.value(1)
        lcd_wr.value(1)
        lcd_wr.value(0)
    lcd_data.value(0)


# sends command 100 and then 0 and then 8 bits of command
def lcd_sendcmd(cmd):
    lcd_csn.value(0)
    lcd_sendbits(0x08, 4)
    lcd_sendbits(cmd, 8)
    lcd_csn.value(1)


# sends command 101 (Write mode) followed by an address and then 4 bits of data
def lcd_writemode(addr, data):
    aval = addr << 2
    for i in range(0, 5):
        dval = (data << (i * 4)) >> 12
        lcd_csn.value(0)
        lcd_sendbits(0x05, 3)
        lcd_sendbits(aval + i, 6)
        lcd_sendbits(dval, 4)
        lcd_csn.value(1)


def lcd_init():
    lcd_sendcmd(0x02)  # SYS_EN
    lcd_sendcmd(0x30)  # RC_32K
    lcd_sendcmd(0x06)  # LCD_ON


def lcd_off():
    lcd_sendcmd(0x04)  # LCD_OFF


def lcd_on():
    lcd_sendcmd(0x06)  # LCD_ON


def lcd_clear():
    for i in range(0, 12):  # 0-9 are the character locations, 10-11 are decimals
        lcd_writemode(i, 0x0000)


def lcd_allon():
    for i in range(0, 12):
        lcd_writemode(i, 0xFFFF)


# inserts a decimal place
def lcd_dp_insert(i):
    if i < 1:
        pass
    elif i < 4:
        lcd_writemode(11, LCD_DPBITS[i - 1])
    elif i < 10:
        lcd_writemode(10, LCD_DPBITS[i - 1])
    else:
        pass


# deletes decimal places
def lcd_dp_clear():
    lcd_writemode(10, 0x0000)
    lcd_writemode(11, 0x0000)


# display an integer number
def lcd_printint(n):
    lcd_clear()
    s = str(n)
    idx = 0
    for c in reversed(s):
        lcd_writemode(idx, LCD_CHARBITS[ord(c) - ord("*")])
        idx = idx + 1


# display a single character
def lcd_printchar(idx, c):
    if ord(c) == 32:  # found a space
        lcd_writemode(idx, 0x0000)
    elif ord(c) <= ord("9"):  # found a number
        lcd_writemode(idx, LCD_CHARBITS[ord(c) - ord("*")])
    else:  # letter A-Z
        lcd_writemode(idx, LCD_CHARBITS[ord(c) - ord("A") + 16])


# display text
def lcd_printtext(s):
    idx = 10 - len(s)
    for c in reversed(s):
        lcd_printchar(idx, c)
        idx = idx + 1


# update value based on encoder/button input
def accel_enc_val(curr_pos):
    # global enc_last_pos
    global enc_value
    diff = curr_pos - enc_last_pos  # amount of rotation
    rotspeed = 1
    #  rotspeed = rot_edge.count  # rotspeed is effectively the speed of rotation
    #  rot_edge.reset()
    if rotspeed <= 8:
        mult = 1  # slow speed, multiplier is 1
    elif rotspeed <= 10:  # adjust all these values to set acceleration
        mult = 50  # also adjust all these values for acceleration too
    elif rotspeed <= 14:
        mult = 100
    else:
        mult = 500
    diff = diff * mult  # apply acceleration factor based on speed of rotation
    diff = diff * speedrange  # multiply further based on button-selected factor
    new_value = enc_last_pos + diff  # update the value
    # set limit/boundaries for the rotation value:
    if new_value < enc_min:
        new_value = enc_min
    elif new_value > enc_max:
        new_value = enc_max
    enc_value = new_value
    return new_value


# handle speedrange button when it is pressed
def update_speedrange():
    global speedrange
    # button is pressed so we need to change the speedrange
    if speedrange == 1:
        speedrange = 1000
        lcd_printchar(9, "K")  # display K for kHz
    elif speedrange == 1000:
        speedrange = 100000
        lcd_printchar(9, "C")  # display C for 100kHz (not ideal!)
    elif speedrange == 100000:
        speedrange = 1000000
        lcd_printchar(9, "M")  # display M for MHz
    else:
        speedrange = 1
        lcd_printchar(8, "Z")
        lcd_printchar(9, "H")  # display HZ for Hz
    tft_speedrange_display()
    utime.sleep(0.1)
    while speedbutton.value() == 0:  # wait for button to be unpressed
        utime.sleep(0.1)
    # button is no longer pressed, we are done.


def i2c_write_byte(addr, reg, v):
    buf = bytearray(2)
    buf[0] = reg & 0xFF
    buf[1] = v & 0xFF
    i2c.writeto(addr, buf)
    utime.sleep(0.001)


def i2c_read_byte(addr, reg):
    buf = bytearray(1)
    buf[0] = reg & 0xFF
    i2c.writeto(addr, buf)
    utime.sleep(0.001)
    i2c.readfrom_into(addr, buf)
    utime.sleep(0.001)
    return buf[0]


def intdiv(n, d):
    res = int(n / d) - 1

    tot = res * d
    while forever:
        tot = tot + d
        res = res + 1
        if tot >= n:
            break
    return res


# configure multisynth divider
# entity is 0-4 for PLLA, PLLB, CLK0, CLK1, CLK2
# p1 is integer, p2 is numerator, p3 is denominator)
def config_multisynth_divider(entity, p1, p2, p3):
    basereg = MSDIVREG[entity]
    i2c_write_byte(CLKGEN_ADDR, basereg + 0, (p3 & 0x0000FF00) >> 8)
    i2c_write_byte(CLKGEN_ADDR, basereg + 1, (p3 & 0x000000FF))
    i2c_write_byte(CLKGEN_ADDR, basereg + 2, (p1 & 0x00030000) >> 16)
    i2c_write_byte(CLKGEN_ADDR, basereg + 3, (p1 & 0x0000FF00) >> 8)
    i2c_write_byte(CLKGEN_ADDR, basereg + 4, (p1 & 0x000000FF))
    i2c_write_byte(
        CLKGEN_ADDR, basereg + 5, ((p3 & 0x000F0000) >> 12) | ((p2 & 0x000F00000) >> 16)
    )
    i2c_write_byte(CLKGEN_ADDR, basereg + 6, (p2 & 0x0000FF00) >> 8)
    i2c_write_byte(CLKGEN_ADDR, basereg + 7, (p2 & 0x000000FF))
    if entity < 2:  # PLLs need resetting
        i2c_write_byte(CLKGEN_ADDR, 177, 0xA0)  # register 177 PLLB_RST and PLLA_RST


# integer PLL mode, entity is 0-4, m is the multiplier,
# assocpll is needed if entity is 2-4 (CLK0-2)
def config_integer(entity, m, assocpll=0):
    if entity < 2:  # PLLA or PLLB
        if (m > 90) or (m < 15):
            print("config_integer error 1! multiplier is out of range")
    else:  # CLK0-2
        if (m > 2049) or (m < 4):
            print("config_integer error2! multiplier is out of range")
    p1 = int((128 * m) - 512)
    p2 = 0
    p3 = 1
    config_multisynth_divider(entity, p1, p2, p3)
    if entity >= 2:  # CLK0-3
        ctrl = 0x0F  # 8mA drive
        if assocpll == 1:  # PLLB
            ctrl |= 0x20
        ctrl |= 0x40  # integer mode
        i2c_write_byte(CLKGEN_ADDR, CLKCTRLREG[entity - 2], ctrl)


# fractional PLL mode, entity is 0-4,
# p1, p2 and p3 are the integer, numerator and denominator respectively
# assocpll is needed if entity is 2-4 (CLK0-2)
def config_fractional(entity, p1, p2, p3, assocpll=0):
    if entity < 2:  # PLLA or PLLB
        if (p1 > 90) or (p1 < 15):
            print("config_fractional error 1! multiplier p1 is out of range")
    else:
        if (p1 > 2049) or (p1 < 4):
            print("config_fractional error2! multiplier p1 is out of range")
    if (p2 > 0xFFFFF) or (p2 < 0):
        print("error! numerator p2 is out of range")
    if (p3 > 0xFFFFF) or (p3 <= 0):
        print("error! denominator p3 is out of range")
    p1 = int((128 * p1) + math.floor((128 * (p2 / p3)) - 512))
    p2 = int((128 * p2) - (p3 * math.floor(128 * (p2 / p3))))
    config_multisynth_divider(entity, p1, p2, p3)
    if entity >= 2:  # CLK0-3
        ctrl = 0x0F  # 8mA drive
        if assocpll == 1:  # PLLB
            ctrl |= 0x20
        i2c_write_byte(CLKGEN_ADDR, CLKCTRLREG[entity - 2], ctrl)


# configure the R divider
# entity is 2-4 (CLK0-2)
def config_r_divider(entity, v, div4=0):
    reg = MSDIVREG[entity] + 2
    if (v < 0) or (v > 7):
        print("error! divider value out of range")
        r = i2c_read_byte(CLKGEN_ADDR, reg)
        r &= 0x0F
        v = (v & 0x07) << 4
        if div4 == 1:
            v |= 0x0C  # set the DIVBY4 bits
        r |= v
        i2c_write_byte(CLKGEN_ADDR, reg, r)


def clkgen_init():
    i2c_write_byte(CLKGEN_ADDR, 3, 0xFF)  # CLKx_OEB all out disabled
    for i in range(16, 24):  # CLKi_CONTROL
        i2c_write_byte(CLKGEN_ADDR, i, 0x80)  # powered off


# enable or disable the clock generator outputs
def clkgen_state(s):
    if s == 0:  # turn off the outputs
        i2c_write_byte(CLKGEN_ADDR, 3, 0xFF)  # CLKx_OEB high to disable
    else:
        i2c_write_byte(CLKGEN_ADDR, 3, 0xF8)  # CLKx_OEB low to enable


def rational_best_approx(rfrac, denom, maxnumer, maxdenom, bestnumer, bestdenom):
    n = rfrac
    d = denom
    n0 = 0
    n1 = 1
    d0 = 1
    d1 = 0
    t = 0
    while forever:
        if (n1 > maxnumer) or (d1 > maxdenom):
            n1 = n0
            d1 = d0
            break
        if d == 0:
            break
        t = d
        # a = n / d
        a = int(n / d)
        d = n % d
        # w = int(n / d)  # '%' is low-precision
        # d = n - (w * d)  # '%' is low precision

        n = t
        t = n0 + a * n1
        n0 = n1
        n1 = t
        t = d0 + a * d1
        d0 = d1
        d1 = t
    return n1, d1


# calculate divider values for desired frequency f
def pll_calc(f):
    ref_f = CLKGEN_XTAL
    f_calc = f
    if f_calc < 600e6:
        f_calc = 600e6
    if f_calc > 900e6:
        f_calc = 900e6
    a = int(f_calc / ref_f)
    if a < 15:
        f_calc = ref_f * 15
    if a > 90:
        f_calc = ref_f * 90
    denom = 1000000
    w = int(f / ref_f)
    lt = f - (w * ref_f)
    lt *= denom
    lt = intdiv(lt, ref_f)
    rfrac = int(lt)
    b = 0
    c = 1
    if rfrac:
        (b, c) = rational_best_approx(rfrac, denom, 1048574, 1048575, b, c)
    p3 = int(c)
    p2 = (128 * b) % int(c)
    p1 = 128 * a
    p1 += int(128 * b / c)
    p1 -= 512
    # recalculate frequency
    f_calc = ref_f
    f_calc *= b
    f_calc = int(f_calc / c)
    f_calc += ref_f * a
    return f_calc, p1, p2, p3


# entity is 0 for PLLA, 1 for PLLB
def set_pll(f, entity):
    (f_calc, p1, p2, p3) = pll_calc(f)
    config_multisynth_divider(entity, int(p1), int(p2), int(p3))
    # return f_calc
    return f


# calculate divider values for desired frequency f
# pllfreq is 0 if the dividers for the PLL need to be calculated
def msynth_calc(f, pllfreq):
    f_int = int(f)
    div4 = 0
    if f_int >= 150000000:
        div4 = 1
    if div4 == 0:
        a = int(900000000 / f_int)
    else:
        a = 4
    b = 0
    c = 1
    pllfreq_calc = int(a * f_int)

    if div4 == 1:
        p3 = 1
        p2 = 0
        p1 = 0
    else:
        p1 = int(128 * a + ((128 * b) / c) - 512)
        p2 = int(128 * b - c * ((128 * b) / c))
        p3 = c
        p2 = int((128 * b) % c)
        p1 = 128 * a
        p1 += int(128 * b / c)
        p1 -= 512
    return pllfreq_calc, p1, p2, p3


def msynth_recalc(f, pllfreq):
    f_calc = int(f)
    pllfreq_int = int(pllfreq)
    div4 = 0
    if f_calc >= 150000000:
        div4 = 1
    a = int(pllfreq_int / f_calc)
    if a < 6:
        # f_calc = int(pllfreq / 6)
        f_calc = int(pllfreq_int / 6)
    if a > 1800:
        # f_calc = int(pllfreq / 1800)
        f_calc = int(pllfreq_int / 1800)
    denom = 1000000
    lt = pllfreq_int % int(f)
    lt *= denom
    lt = int(lt / f)
    rfrac = int(lt)
    b = 0
    c = 1
    if rfrac:
        (b, c) = rational_best_approx(rfrac, denom, 1048574, 1048575, b, c)
    lt = pllfreq_int
    lt *= c
    lt = int(lt / (a * c + b))
    f_calc = lt
    if div4 == 1:
        p3 = 1
        p2 = 0
        p1 = 0
    else:
        p3 = c
        p2 = 128 * int(b) % int(c)
        p1 = 128 * a
        p1 += int(128 * b / c)
        p1 -= 512
    print("msynth_recalc")
    #print(
    #    f"msynth_recalc f_calc = {f_calc}, \
    #    p1 = {p1}, p2 = {p2}, p3 = {p3}"
    #)
    return f_calc, p1, p2, p3


def sel_rdiv(f):
    r_calc = 0
    f_calc = f
    e = 1
    bitval = 256
    for idx in range(7, 0, -1):
        mma = 4000 * e
        e = e * 2
        mmb = 4000 * e
        bitval = bitval / 2
        if (f_calc >= mma) and (f_calc < mmb):
            r_calc = idx
            f_calc *= bitval
            break
    return f_calc, r_calc


# this function calls the config_multisynth_divider
# and config_r_divider as appropriate
# entity is 2-4 for CLK0, CLK1 or CLK2 respectively
def clkgen_config_ms(entity, p1, p2, p3, mode, rdiv, div4):
    config_multisynth_divider(entity, p1, p2, p3)
    # set integer or fractional mode as needed
    ctrl = 0x0F  # 8mA drive
    if entity == 3:  # associate CLK2 with PLLB
        ctrl |= 0x20
    if mode == 1:  # integer mode
        ctrl |= 0x40
    else:  # fractional mode
        pass
    i2c_write_byte(CLKGEN_ADDR, CLKCTRLREG[entity - 2], ctrl)
    # config r divider
    config_r_divider(entity, rdiv, div4)


def clkgen_config_ms_source(entity):
    # sets integer or fractional mode and associates a PLL with CLK0 or CLK1
    ctrl = 0x0F  # 8mA drive
    if entity == 3:  # associate CLK2 with PLLB
        ctrl |= 0x20
    # ctrl |= 0x40 # integer mode, comment out for fractional
    i2c_write_byte(CLKGEN_ADDR, CLKCTRLREG[entity - 2], ctrl)


# programs up registers based on desired frequency
# entity is 0 for PLLA, 1 for PLLB
# init=1 will execute everything
def clkgen_set_freq(freq, pllfreq, entity, init=1):
    global p1store
    global p2store
    global p3store
    f = freq
    pllfreq_calc = pllfreq
    sp = 0
    if pllfreq == 0:
        (pllfreq_calc, m1, m2, m3) = msynth_calc(f, 0)
        sp = 1
    else:
        (dummy, m1, m2, m3) = msynth_recalc(f, pllfreq)
        sp = 0
    if (init == 1):
        clkgen_config_ms_source(entity + 2)
    (dummy, p1, p2, p3) = pll_calc(pllfreq_calc)
    if (sp == 1) and (init == 1):  # write the PLL parameters
        config_multisynth_divider(entity, int(p1), int(p2), int(p3))
        if (entity < 2):
            p1store[entity] = p1
            p2store[entity] = p2
            p3store[entity] = p3
            #print(f"storing e={entity}, {p1store[entity]} {p2store[entity]} {p3store[entity]}")
            print("storing e={}, {} {} {}".format(entity, p1store[entity], p2store[entity], p3store[entity]))
    elif (sp == 1):
        if (entity < 2):
            if (
                (p1store[entity] == p1)
                and (p2store[entity] == p2)
                and (p3store[entity] == p3)
            ):
                pass
            else:
                #print(f"new values e={entity}, {p1} {p2} {p3}")
                print("new values e={}, {} {} {}".format(entity, p1, p2, p3))
                #print(f"store has e={entity}, {p1store[entity]} {p2store[entity]} {p3store[entity]}")
                print("store has e={}, {} {} {}".format(entity, p1store[entity], p2store[entity], p3store[entity]))
                p1store[entity] = p1
                p2store[entity] = p2
                p3store[entity] = p3
                config_multisynth_divider(entity, int(p1), int(p2), int(p3))
                print("updated PLL")
    # write the CLK0/CLK1 params
    config_multisynth_divider(entity + 2, int(m1), int(m2), int(m3))
    clkgen_config_ms_source(entity + 2)
    return pllfreq_calc

# display true-type font text on TFT screen, centered
def center(font, string, row, color=st7789.WHITE):
    screen = tft.width                        # get screen width
    width = tft.write_width(font, string)     # get the width of the string
    if width and width < screen:              # if the string < display
        col = tft.width // 2 - width // 2     # find the column to center
    else:                                     # otherwise
        col = 0                               # left justify
    tft.write(font, string, col, row, color)  # and write the string

def tft_speedrange_display():
    for i in range(0,4):
        tft.fill_rect(rangexpos[i], rangeypos[i], rangewidth[i], 6, 0) # blanking
    sel = 0
    if (speedrange == 1):
        sel = 0
    elif (speedrange == 1000):
        sel = 1
    elif (speedrange == 100000):
        sel = 2
    elif (speedrange == 1000000):
        sel = 3
    else:
        pass
    tft.fill_rect(rangexpos[sel], rangeypos[sel], rangewidth[sel], 6, color3)

def tft_freq_display(n):
    global hist_m_txt
    global hist_h_txt
    m = int(n/1000)
    m_txt = f"{m/1000:7.3f}" # get the number into a format "  X.XXX" (MHz)
    h = n - (m*1000)
    h_txt = f"{h:03}" # get the remaining Hz into a format like "0XX"
    w = 0
    wtot = 0
    for i in range (0, 7):
        w = tft.write_width(mfont64, m_txt[i])
        if (m_txt[i] == ' '):
            w = tft.write_width(mfont64, "0")
        if (hist_m_txt[i] != m_txt[i]):
            if (m_txt[i] == ' '):
                tft.fill_rect(wtot, freq_ypos, mfont64.MAX_WIDTH, 64, 0)
            tft.write(mfont64, m_txt[i], wtot, freq_ypos, color2)
            t = list(hist_m_txt)
            t[i] = m_txt[i]
            hist_m_txt = "".join(t)
        wtot = wtot + w
    
    w = 26 # tft.write_width(mfont40, "1")
    wtot = hxloc
    for i in range (0, 3):
        w = tft.write_width(mfont40, h_txt[i])
        if (hist_h_txt[i] != h_txt[i]):
            tft.write(mfont40, h_txt[i], wtot, freq_ypos+64, color2)
            t = list(hist_h_txt)
            t[i] = h_txt[i]
            hist_h_txt = "".join(t)
        wtot = wtot + w

# main program
def app_main():
    global enc_last_pos
    global rotval
    io_init()
    enc_init()
    clkgen_init()
    lcd_init()
    lcd_clear()
    heartbeat_toggle = False
    utime.sleep(0.1)
    lcd_printtext("HI THERE")  # mainly only uppercase and numerics supported today
    
    # tft setup
    tft.rotation(1)
    tft.fill(0)
    #tft.text(font, "HI THERE", 0, 20, color1, color2)
    pytext.text(tft, font_data, "PicoSynth", 110, 0, color1, scale=0.9)
    
    utime.sleep(1)
    lcd_printint(rotval)  # display the initial value
    #tft.text(mfont, str(rotval), 0, 60, color1, color2)
    #center(mfont, str(rotval), freq_ypos, color2)
    tft_freq_display(rotval)
    tft_speedrange_display()

    pllfreq = clkgen_set_freq(rotval, 0, 0)
    clkgen_state(1)  # turn on the outputs
    
    print(micropython.mem_info())

    while forever:  # forever loop
        # alternately set the LED on/off during each loop
        heartbeat_toggle = not (heartbeat_toggle)
        boardled.value(heartbeat_toggle)  # toggle the LED
        # handle rotary encoder
        enc_pos = enc_value  # read the rotary encoder value
        if enc_pos != enc_last_pos:  # encoder has been rotated!
            rotval = accel_enc_val(enc_pos)
            enc_last_pos = rotval
            clkgen_set_freq(rotval, 0, 0, init=0)
            #lcd_printint(rotval)  # display the value
            #tft.text(font, str(rotval), 0, 80, color1, color2)
            tft_freq_display(rotval)
        # handle manual speed step button
        if speedbutton.value() == 0:  # button is pressed!
            update_speedrange()
            #lcd_printint(rotval)  # clear the display and print the value
            #tft.text(font, str(rotval), 0, 80, color1, color2)
            tft_freq_display(rotval)
            utime.sleep(0.04)  # button unpress debounce period
        # brief pause
        utime.sleep(0.04)  # pause briefly then loop back


# run the main program function
# execute the following from main.py: app_main()
