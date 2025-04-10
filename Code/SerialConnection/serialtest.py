import serial, time

ser = serial.Serial('/dev/cu.usbmodem21401', 9600)

while True:
  speed = [240,250]
  message = speed[0]<<8 | speed[1]
  print(message.to_bytes(2, 'little'))
  ser.write(message.to_bytes(2, 'little'))
  while ser.in_waiting:
    print(ser.readline().decode().strip())
  time.sleep(1)