from machine import Pin, Timer
import time
import micropython

_DIR_CW = 0x10  # Clockwise ste
_DIR_CCW = 0x20  # Counter-clockwise ste

# Rotary Encoder States
_R_START = 0x0
_R_CW_1 = 0x1
_R_CW_2 = 0x2
_R_CW_3 = 0x3
_R_CCW_1 = 0x4
_R_CCW_2 = 0x5
_R_CCW_3 = 0x6
_R_ILLEGAL = 0x7

_transition_table = [

    # |------------- NEXT STATE -------------|            |CURRENT STATE|
    # CLK/DT    CLK/DT     CLK/DT    CLK/DT
    #   00        01         10        11
    [_R_START, _R_CCW_1, _R_CW_1, _R_START],  # _R_START
    [_R_CW_2, _R_START, _R_CW_1, _R_START],  # _R_CW_1
    [_R_CW_2, _R_CW_3, _R_CW_1, _R_START],  # _R_CW_2
    [_R_CW_2, _R_CW_3, _R_START, _R_START | _DIR_CW],  # _R_CW_3
    [_R_CCW_2, _R_CCW_1, _R_START, _R_START],  # _R_CCW_1
    [_R_CCW_2, _R_CCW_1, _R_CCW_3, _R_START],  # _R_CCW_2
    [_R_CCW_2, _R_START, _R_CCW_3, _R_START | _DIR_CCW],  # _R_CCW_3
    [_R_START, _R_START, _R_START, _R_START]]  # _R_ILLEGAL

_transition_table_half_step = [
    [_R_CW_3, _R_CW_2, _R_CW_1, _R_START],
    [_R_CW_3 | _DIR_CCW, _R_START, _R_CW_1, _R_START],
    [_R_CW_3 | _DIR_CW, _R_CW_2, _R_START, _R_START],
    [_R_CW_3, _R_CCW_2, _R_CCW_1, _R_START],
    [_R_CW_3, _R_CW_2, _R_CCW_1, _R_START | _DIR_CW],
    [_R_CW_3, _R_CCW_2, _R_CW_3, _R_START | _DIR_CCW],
    [_R_START, _R_START, _R_START, _R_START],
    [_R_START, _R_START, _R_START, _R_START]]

_STATE_MASK = 0x07
_DIR_MASK = 0x30


def _wrap(value, incr, lower_bound, upper_bound):
    range = upper_bound - lower_bound + 1
    value = value + incr

    if value < lower_bound:
        value += range * ((lower_bound - value) // range + 1)

    return lower_bound + (value - lower_bound) % range


def _bound(value, incr, lower_bound, upper_bound):
    return min(upper_bound, max(lower_bound, value + incr))


def _trigger(rotary_instance):
    for listener in rotary_instance._listener:
        listener()


class Rotary(object):
    RANGE_UNBOUNDED = 1
    RANGE_WRAP = 2
    RANGE_BOUNDED = 3

    def __init__(self, min_val, max_val, incr, reverse, range_mode, half_step, invert):
        self._min_val = min_val
        self._max_val = max_val
        self._incr = incr
        self._reverse = -1 if reverse else 1
        self._range_mode = range_mode
        self._value = min_val
        self._state = _R_START
        self._half_step = half_step
        self._invert = invert
        self._listener = []

    def set(self, value=None, min_val=None, incr=None,
            max_val=None, reverse=None, range_mode=None):
        # disable DT and CLK pin interrupts
        self._hal_disable_irq()

        if value is not None:
            self._value = value
        if min_val is not None:
            self._min_val = min_val
        if max_val is not None:
            self._max_val = max_val
        if incr is not None:
            self._incr = incr
        if reverse is not None:
            self._reverse = -1 if reverse else 1
        if range_mode is not None:
            self._range_mode = range_mode
        self._state = _R_START

        # enable DT and CLK pin interrupts
        self._hal_enable_irq()

    def value(self):
        return self._value

    def reset(self):
        self._value = 0

    def close(self):
        self._hal_close()

    def add_listener(self, l):
        self._listener.append(l)

    def remove_listener(self, l):
        if l not in self._listener:
            raise ValueError('{} is not an installed listener'.format(l))
        self._listener.remove(l)

    def _process_rotary_pins(self, pin):
        old_value = self._value
        clk_dt_pins = (self._hal_get_clk_value() <<
                       1) | self._hal_get_dt_value()

        if self._invert:
            clk_dt_pins = ~clk_dt_pins & 0x03

        # Determine next state
        if self._half_step:
            self._state = _transition_table_half_step[self._state &
                                                      _STATE_MASK][clk_dt_pins]
        else:
            self._state = _transition_table[self._state &
                                            _STATE_MASK][clk_dt_pins]
        direction = self._state & _DIR_MASK

        incr = 0
        if direction == _DIR_CW:
            incr = self._incr
        elif direction == _DIR_CCW:
            incr = -self._incr

        incr *= self._reverse

        if self._range_mode == self.RANGE_WRAP:
            self._value = _wrap(
                self._value,
                incr,
                self._min_val,
                self._max_val)
        elif self._range_mode == self.RANGE_BOUNDED:
            self._value = _bound(
                self._value,
                incr,
                self._min_val,
                self._max_val)
        else:
            self._value = self._value + incr

        try:
            if old_value != self._value and len(self._listener) != 0:
                _trigger(self)
        except:
            pass


IRQ_RISING_FALLING = Pin.IRQ_RISING | Pin.IRQ_FALLING


class RotaryIRQ(Rotary):
    def __init__(
            self,
            pin_num_clk,
            pin_num_dt,
            min_val=0,
            max_val=10,
            incr=1,
            reverse=False,
            range_mode=Rotary.RANGE_UNBOUNDED,
            pull_up=False,
            half_step=False,
            invert=False
    ):
        super().__init__(min_val, max_val, incr, reverse, range_mode, half_step, invert)

        if pull_up:
            self._pin_clk = Pin(pin_num_clk, Pin.IN, Pin.PULL_UP)
            self._pin_dt = Pin(pin_num_dt, Pin.IN, Pin.PULL_UP)
        else:
            self._pin_clk = Pin(pin_num_clk, Pin.IN)
            self._pin_dt = Pin(pin_num_dt, Pin.IN)

        self._hal_enable_irq()

    def _enable_clk_irq(self):
        self._pin_clk.irq(self._process_rotary_pins, IRQ_RISING_FALLING)

    def _enable_dt_irq(self):
        self._pin_dt.irq(self._process_rotary_pins, IRQ_RISING_FALLING)

    def _disable_clk_irq(self):
        self._pin_clk.irq(None, 0)

    def _disable_dt_irq(self):
        self._pin_dt.irq(None, 0)

    def _hal_get_clk_value(self):
        return self._pin_clk.value()

    def _hal_get_dt_value(self):
        return self._pin_dt.value()

    def _hal_enable_irq(self):
        self._enable_clk_irq()
        self._enable_dt_irq()

    def _hal_disable_irq(self):
        self._disable_clk_irq()
        self._disable_dt_irq()

    def _hal_close(self):
        self._hal_disable_irq()


r_l = RotaryIRQ(
    pin_num_clk=22,
    pin_num_dt=21,
    reverse=False,
    min_val=1,
    max_val=20,
    incr=1,
    range_mode=RotaryIRQ.RANGE_BOUNDED,
    pull_up=True,
    half_step=False,
)

r_r = RotaryIRQ(
    pin_num_clk=27,
    pin_num_dt=26,
    reverse=False,
    min_val=0,
    max_val=6,
    incr=1,
    range_mode=RotaryIRQ.RANGE_BOUNDED,
    pull_up=True,
    half_step=False,
)

valuesDictNeutre = {"1": ["10000", "10000", "10000", "10000", "10000", "10000", "10000"],
                    "2": ["8333", "10000", "10000", "10000", "10000", "10000", "10000"],
                    "3": ["7500", "9028", "10000", "10000", "10000", "10000", "10000"],
                    "4": ["6667", "8750", "9144", "10000", "10000", "10000", "10000"],
                    "5": ["5833", "8333", "9074", "9163", "10000", "10000", "10000"],
                    "6": ["5000", "7778", "8935", "9147", "9166", "10000", "10000"],
                    "7": ["4167", "7083", "8704", "9109", "9163", "9167", "10000"],
                    "8": ["3333", "6250", "8356", "9032", "9153", "9166", "9167"],
                    "9": ["2500", "5417", "7870", "8897", "9131", "9164", "9167"],
                    "10": ["1667", "4583", "7269", "8681", "9086", "9158", "9166"],
                    "11": ["0833", "3750", "6574", "8368", "9005", "9144", "9165"],
                    "12": ["0833", "2917", "5810", "7951", "8872", "9117", "9161"],
                    "13": ["0833", "2222", "5000", "7431", "8673", "9068", "9153"],
                    "14": ["0833", "1667", "4190", "6813", "8393", "8987", "9136"],
                    "15": ["0833", "1250", "3426", "6119", "8023", "8860", "9106"],
                    "16": ["0833", "0972", "2731", "5378", "7560", "8675", "9056"],
                    "17": ["0833", "0833", "2130", "4622", "7010", "8421", "8975"],
                    "18": ["0833", "0833", "1644", "3881", "6386", "8089", "8855"],
                    "19": ["0833", "0833", "1296", "3187", "5707", "7674", "8683"],
                    "20": ["0833", "0833", "1065", "2569", "5000", "7180", "8451"]}

valuesDictDefav = {"1": ["10000", "10000", "10000", "10000", "10000", "10000", "10000"],
                   "2": ["6944", "10000", "10000", "10000", "10000", "10000", "10000"],
                   "3": ["5625", "8160", "10000", "10000", "10000", "10000", "10000"],
                   "4": ["4444", "7697", "8362", "10000", "10000", "10000", "10000"],
                   "5": ["3403", "7037", "8245", "8396", "10000", "10000", "10000"],
                   "6": ["2500", "6204", "8017", "8370", "8402", "10000", "10000"],
                   "7": ["1736", "5220", "7650", "8305", "8396", "8403", "10000"],
                   "8": ["1111", "4109", "7120", "8180", "8380", "8401", "8403"],
                   "9": ["0625", "3137", "6404", "7966", "8343", "8398", "8403"],
                   "10": ["0278", "2303", "5567", "7633", "8270", "8388", "8402"],
                   "11": ["0069", "1609", "4668", "7167", "8142", "8366", "8399"],
                   "12": ["0069", "1053", "3764", "6571", "7937", "8322", "8393"],
                   "13": ["0069", "0648", "2905", "5862", "7637", "8245", "8379"],
                   "14": ["0069", "0370", "2143", "5071", "7230", "8118", "8353"],
                   "15": ["0069", "0197", "1520", "4242", "6712", "7926", "8306"],
                   "16": ["0069", "0104", "1030", "3428", "6091", "7655", "8227"],
                   "17": ["0069", "0069", "0664", "2672", "5390", "7291", "8105"],
                   "18": ["0069", "0069", "0407", "2004", "4641", "6833", "7926"],
                   "19": ["0069", "0069", "0243", "1445", "3880", "6284", "7678"],
                   "20": ["0069", "0069", "0147", "1001", "3144", "5657", "7351"]}

valuesDictFav = {"1": ["10000", "10000", "10000", "10000", "10000", "10000", "10000"],
                 "2": ["9722", "10000", "10000", "10000", "10000", "10000", "10000"],
                 "3": ["9375", "9896", "10000", "10000", "10000", "10000", "10000"],
                 "4": ["8889", "9803", "9925", "10000", "10000", "10000", "10000"],
                 "5": ["8264", "9630", "9904", "9930", "10000", "10000", "10000"],
                 "6": ["7500", "9352", "9853", "9925", "9930", "10000", "10000"],
                 "7": ["6597", "8947", "9757", "9912", "9929", "9931", "10000"],
                 "8": ["5556", "8391", "9593", "9883", "9926", "9930", "9931"],
                 "9": ["4375", "7697", "9336", "9827", "9919", "9930", "9931"],
                 "10": ["3056", "6863", "8970", "9728", "9901", "9928", "9930"],
                 "11": ["1597", "5891", "8480", "9569", "9868", "9923", "9930"],
                 "12": ["1597", "4780", "7857", "9332", "9807", "9912", "9929"],
                 "13": ["1597", "3796", "7095", "8999", "9709", "9892", "9926"],
                 "14": ["1597", "2963", "6236", "8555", "9556", "9855", "9919"],
                 "15": ["1597", "2303", "5332", "7996", "9335", "9793", "9907"],
                 "16": ["1597", "1840", "4433", "7328", "9030", "9696", "9884"],
                 "17": ["1597", "1597", "3596", "6572", "8630", "9551", "9845"],
                 "18": ["1597", "1597", "2880", "5758", "8130", "9344", "9783"],
                 "19": ["1597", "1597", "2350", "4929", "7535", "9065", "9689"],
                 "20": ["1597", "1597", "1983", "4138", "6856", "8703", "9551"]}

sr = 15
rang = 3
fav = 0
val = '0000'

dt_r = Pin(21, Pin.IN)
clk_r = Pin(22, Pin.IN)

mode = -1  # -1 = setup ; 1 = view

segDict = {" ": [0, 0, 0, 0, 0, 0, 0], "-": [0, 0, 0, 0, 0, 0, 1], "_": [0, 0, 0, 1, 0, 0, 0],
           "+": [1, 0, 0, 0, 0, 0, 0], "0": [1, 1, 1, 1, 1, 1, 0], "1": [0, 1, 1, 0, 0, 0, 0],
           "2": [1, 1, 0, 1, 1, 0, 1], "3": [1, 1, 1, 1, 0, 0, 1], "4": [0, 1, 1, 0, 0, 1, 1],
           "5": [1, 0, 1, 1, 0, 1, 1], "6": [1, 0, 1, 1, 1, 1, 1], "7": [1, 1, 1, 0, 0, 0, 0],
           "8": [1, 1, 1, 1, 1, 1, 1], "9": [1, 1, 1, 1, 0, 1, 1]}

timerD1 = Timer()
timerD2 = Timer()
timerD3 = Timer()
timerD4 = Timer()

d1 = Pin(12, Pin.OUT, Pin.PULL_UP)
d2 = Pin(13, Pin.OUT, Pin.PULL_UP)
d3 = Pin(14, Pin.OUT, Pin.PULL_UP)
d4 = Pin(15, Pin.OUT, Pin.PULL_UP)

aa = Pin(2, Pin.OUT)
ab = Pin(4, Pin.OUT)
ac = Pin(19, Pin.OUT)
ad = Pin(17, Pin.OUT)
ae = Pin(16, Pin.OUT)
af = Pin(3, Pin.OUT)
ag = Pin(20, Pin.OUT)

dp = Pin(18, Pin.OUT)

sw_r = Pin(9, Pin.IN, Pin.PULL_DOWN)
sw_l = Pin(8, Pin.IN, Pin.PULL_DOWN)

d1.value(1)
d2.value(1)
d3.value(1)
d4.value(1)

aa.value(1)
ab.value(1)
ac.value(1)
ad.value(1)
ae.value(1)
af.value(1)
ag.value(1)
dp.value(1)

def clear(timer):
    d1.value(1)
    d2.value(1)
    d3.value(1)
    d4.value(1)

    aa.value(0)
    ab.value(0)
    ac.value(0)
    ad.value(0)
    ae.value(0)
    af.value(0)
    ag.value(0)
    dp.value(0)

def writeDigit(arrSeg, d):
    aa.value(arrSeg[0])
    ab.value(arrSeg[1])
    ac.value(arrSeg[2])
    ad.value(arrSeg[3])
    ae.value(arrSeg[4])
    af.value(arrSeg[5])
    ag.value(arrSeg[6])
    dp.value(d)


def displayD1(timer):
    timClear = Timer()
    timClear.init(mode=Timer.ONE_SHOT, period=4, callback=clear)
    d1.value(0)
    d2.value(1)
    d3.value(1)
    d4.value(1)

    if mode == -1:
        writeDigit(segDict[str(rang)], 0)
    if mode == 1:
        writeDigit(segDict[val[0]], 0)


def displayD2(timer):
    timClear = Timer()
    timClear.init(mode=Timer.ONE_SHOT, period=4, callback=clear)
    d1.value(1)
    d2.value(0)
    d3.value(1)
    d4.value(1)

    if mode == -1:
        if fav == -1:
            writeDigit(segDict['_'], 0)
        else:
            if fav == 0:
                writeDigit(segDict['-'], 0)
            else:
                if fav == 1:
                    writeDigit(segDict['+'], 0)
    if mode == 1:
        if val == '10000':
            writeDigit(segDict[val[1]], 0)
        else:
            writeDigit(segDict[val[1]], 1)


def displayD3(timer):
    timClear = Timer()
    timClear.init(mode=Timer.ONE_SHOT, period=4, callback=clear)
    d1.value(1)
    d2.value(1)
    d3.value(0)
    d4.value(1)

    if mode == -1:
        if sr >= 10:
            writeDigit(segDict[f"{sr:02}"[0]], 0)
        else:
            writeDigit(segDict[" "], 0)
    if mode == 1:
        if val == '10000':
            writeDigit(segDict[val[2]], 1)
        else:
            writeDigit(segDict[val[2]], 0)

def displayD4(timer):
    timClear = Timer()
    timClear.init(mode=Timer.ONE_SHOT, period=4, callback=clear)
    d1.value(1)
    d2.value(1)
    d3.value(1)
    d4.value(0)

    if mode == -1:
        writeDigit(segDict[f"{sr:02}"[1]], 0)
    if mode == 1:
        writeDigit(segDict[val[3]], 0)


def timD1(timer):
    timerD1.init(mode=Timer.PERIODIC, period=24, callback=displayD1)

def timD2(timer):
    timerD2.init(mode=Timer.PERIODIC, period=24, callback=displayD2)

def timD3(timer):
    timerD3.init(mode=Timer.PERIODIC, period=24, callback=displayD3)


def timD4(timer):
    timerD4.init(mode=Timer.PERIODIC, period=24, callback=displayD4)


tim1 = Timer()
tim1.init(mode=Timer.ONE_SHOT, period=0, callback=timD1)
tim2 = Timer()
tim2.init(mode=Timer.ONE_SHOT, period=6, callback=timD2)
tim3 = Timer()
tim3.init(mode=Timer.ONE_SHOT, period=12, callback=timD3)
tim4 = Timer()
tim4.init(mode=Timer.ONE_SHOT, period=18, callback=timD4)

r_l.set(value=4)
sr = r_l.value()

r_r.set(value=2)
rang = r_r.value()

sw_r_oldV = sw_r.value()
sw_l_oldV = sw_l.value()

while True:
    if sw_r.value() != sw_r_oldV:
        if sw_r.value() > 0:
            mode = mode * (-1)
        sw_r_oldV = sw_r.value()

    sr = r_l.value()
    rang = r_r.value()

    if sw_l.value() != sw_l_oldV:
        if sw_l.value() > 0:
            fav = fav + 1
        if fav > 1:
            fav = -1
        sw_l_oldV = sw_l.value()

    if fav == -1:
        val = valuesDictDefav[str(sr)][rang]
    if fav == 0:
        val = valuesDictNeutre[str(sr)][rang]
    if fav == 1:
        val = valuesDictFav[str(sr)][rang]

    time.sleep_ms(50)
