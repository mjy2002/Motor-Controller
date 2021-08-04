from hardware import Motor

m = Motor('/dev/cu.usbmodem1243201')
print(m.id, m.position, m.velocity)
print(m.setPIDs('angle', 10, D=0.1))
