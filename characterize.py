#coding=utf-8
import time
import Slush
import numpy as np
import RPi.GPIO as GPIO
from Slush.Devices import L6470Registers as LReg

GPIO.setwarnings(False)

s = Slush.sBoard()
m = Slush.Motor(0)
m.xfer(LReg.RESET_DEVICE)

# speeds to characterize
speeds = range(100, 1001, 50)
# target current
current = .7
start_k = 10


# this procedure finds the Krun value that just exceeds the stall threshold
def find_kval(start):
    krun = start

    # now use succesive approximation to find the next 4 bits
    while krun < 256:
        krun += 1
        m.setParam((0x0A,8), krun)
        m.getStatus()
        time.sleep(0.5)
        status = m.getStatus()
        print('k_val = %d, stall = %d' % (krun, (status >> 13) & 3))
        if status & (3 << 13) == 0:
            break

    return krun


m.xfer(LReg.NOP)  # Does nothing NOP
m.free()  # hardHiZ
m.xfer(LReg.RESET_DEVICE)  # resets chip
time.sleep(.05)
m.xfer(LReg.NOP)  # Does nothing NOP
m.free()
print(m.getStatus())

# set the BEMF compensation parameters to zero so I can find the 
# compensated value
m.setParam(LReg.ST_SLP, 0)
m.setParam(LReg.FN_SLP_ACC, 0)
m.setParam(LReg.FN_SLP_DEC, 0)
m.setParam(LReg.FS_SPD, 0x3FF)
m.setParam(LReg.MAX_SPEED, 0x300)
m.setParam(LReg.STALL_TH, 15)
m.setParam(LReg.STEP_MODE, 7)
m.setParam(LReg.K_THERM, 0)
m.setParam(LReg.ACC, 0x8A)
m.setParam(LReg.DEC, 0x8A)
m.setParam(LReg.MIN_SPEED, 0)
m.setParam(LReg.KVAL_HOLD, 4)
m.setParam(LReg.KVAL_ACC, 0x29)
m.setParam(LReg.KVAL_DEC, 0x29)
m.setParam(LReg.INT_SPD, 0)
m.setParam(LReg.KVAL_RUN, start_k)

kvals = []
k_val = start_k

# set the stall detect threshold to the target current
m.setParam(LReg.STALL_TH, int(current / 0.03125))

for speed in speeds:
    if k_val == 256:
        break

    m.getStatus()
    m.run(1, speed)

    # let the speed stabilize
    time.sleep(1)

    k_val = find_kval(k_val)
    print('Speed: %d, k_val: %d' % (speed, k_val))

    kvals.append(k_val)

m.free()

steps = []
k_val_arr = []

for (i, j) in zip(speeds, kvals):
    steps.append(i)
    k_val_arr.append(j)

    print('%d, %d' % (i, j))

m,b = np.polyfit(steps, k_val_arr, 1)
print("slope = " + str(m))
print("Y-int = " + str(b))
