from hardware import Motor

m = Motor('/dev/cu.usbmodem1243201')
print(m.id, m.position, m.velocity)
