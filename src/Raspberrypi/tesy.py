import serial
s = serial.Serial("/dev/ttyAMA0", 1000000, timeout=1)
s.write(b"hello"); print(s.read(5))