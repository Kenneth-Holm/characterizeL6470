# SPINFamily Evaluation Tool Script file
# Wednesday, March 22, 2017
import time
import Slush

s = Slush.sBoard()
m = Slush.Motor(0)


# speeds to characterize
speeds = range(100, 1001, 50)
# target current
current = .7
start_k = 30

# this procedure finds the Krun value that just exceeds the stall threshold
def find_kval(start):

  krun = start

  # now use succesive approximation to find the next 4 bits
  while krun < 256:
    krun += 1
    m.setParam(0x0A, krun)
    m.getStatus()
    time.sleep(0.5)
    status = m.getStatus()
    print('k_val = %d, stall = %d'%(krun, (status >> 13) & 3))
    if status & (3 << 13) == 0:
      break

  return krun
    
m.xfer(0x00) #Does nothing NOP
m.xfer(0x00) #Does nothing NOP
m.xfer(0x00) #Does nothing NOP
m.free()     #hardHiZ
m.xfer(0x0C) #resets chip
time.sleep(.05)
m.xfer(0x00) #Does nothing NOP
m.xfer(0x00) #Does nothing NOP
m.xfer(0x00) #Does nothing NOP
m.free()
print('%d'% (m.getStatus() >> 13) & 3)

# set the BEMF compensation parameters to zero so I can find the 
# compensated value
m.setParam((0x0E , 8), 0)
m.setParam((0x0F , 8), 0)
m.setParam((0x10 , 8), 0)
m.setParam((0x15 , 10), 0x3FF)
m.setParam((0x07 , 10), 0x300)
m.setParam((0x14 , 7), 15)
m.setParam((0x16 , 8), 7)
m.setParam((0x11 , 4), 0)
m.setParam((0x05 , 12), 0x8A)
m.setParam(( 0x06 , 12 ), 0x8A)
m.setParam(( 0x08 , 12 ), 0)
m.setParam(( 0x09 , 8  ), 4)
m.setParam(( 0x0B , 8  ), 0x29)
m.setParam(( 0x0C , 8  ), 0x29)
m.setParam(( 0x0D , 14 ), 0)
m.setParam(( 0x0A , 8  ), start_k)

kvals = []
k_val = start_k

# set the stall detect threshold to the target current
m.setParam(( 0x14 , 7  ), int(current/0.03125))

for speed in speeds:
  m.getStatus()
  m.run(1, '%d S/s'%speed)

  # let the speed stabilize
  time.sleep(1)

  k_val = find_kval(k_val)
  print('Speed: %d, k_val: %d'%(speed, k_val))

  kvals.append(k_val)

m.free()

for (i,j) in zip(speeds, kvals):
  print('%d, %d'%(i,j))

