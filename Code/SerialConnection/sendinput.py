import serial
import time
import pygame as pg



ser = serial.Serial('/dev/cu.usbmodem11401', 9600)

pg.init()
window = pg.display.set_mode((100, 100))

speed = [0,0]

# while True:
#   speed = [100,100]
#   message = speed[0]<<8 | speed[1]
#   print(message.to_bytes(2, 'big'))
#   ser.write(message.to_bytes(2, 'big'))
#   while ser.in_waiting:
#     print(ser.readline().decode().strip())
#   time.sleep(1)

while True:
  # detect keys
  keys = pg.key.get_pressed()
  if keys[pg.K_LEFT]:
    # print("left")
    speed[0] = max(0, speed[0]-50)
  elif keys[pg.K_RIGHT]:
    # print("right")
    speed[0] = min(255, speed[0]+50)
  else:
    # speed[0] = 0
    pass

  if keys[pg.K_DOWN]:
    # print("up")
    speed[1] = max(0, speed[1]-50)
  elif keys[pg.K_UP]:
    # print("down")
    speed[1] = min(255, speed[1]+50)
  else:
    # speed[1] = 0
    pass
  
  message = speed[0]<<8 | speed[1]
  print(message.to_bytes(2, 'big'))
  ser.write(message.to_bytes(2, 'big'))

  # read data if present
  # while ser.in_waiting:
  #   print(ser.readline().decode().strip())

  for event in pg.event.get():
    if event.type == pg.QUIT:
      pg.quit()
      ser.close()
      exit()

  pg.display.update() 
  

