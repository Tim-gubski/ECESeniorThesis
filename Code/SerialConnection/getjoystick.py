import pygame
import time
import serial
import math

pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks connected: {joystick_count}")

if joystick_count == 0:
    print("No joysticks found. Please connect a controller.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick name: {joystick.get_name()}")

ser = serial.Serial('/dev/cu.usbmodem1401', 9600)

left_x, left_y = 0, 0
right_x, right_y = 0, 0
send_angle = 0
target_enc = 0
enc_count = 10
while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            # Left joystick
            if event.axis == 0:  # X-axis (left/right)
                left_x = event.value
                # print(f"Left joystick X-axis: {left_x:.2f}")
            elif event.axis == 1:  # Y-axis (up/down)
                left_y = event.value
                # print(f"Left joystick Y-axis: {left_y:.2f}")

            # Right joystick
            elif event.axis == 2:  # X-axis (left/right)
                right_x = event.value
                # print(f"Right joystick X-axis: {right_x:.2f}")
            elif event.axis == 3:  # Y-axis (up/down)
                right_y = event.value
                # print(f"Right joystick Y-axis: {right_y:.2f}")
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 1:
                # reset position
                message = 0 << 16 | 0 << 8 | 3
                print(message.to_bytes(3, 'little'))
                ser.write(message.to_bytes(3, 'little'))
            if event.button == 10 or event.button == 9:
                message = target_enc << 16 | (1 if event.button == 10 else 2) << 8 | 2
                print(message.to_bytes(3, 'little'))
                ser.write(message.to_bytes(3, 'little'))
            elif event.button == 2:
                target_enc = (target_enc + 1) % enc_count
                
            
    # print(f"Left joystick: ({left_x:.2f}, {left_y:.2f})")
    # print(f"Right joystick: ({right_x:.2f}, {right_y:.2f})")
    
    left_angle = 360 - (int(math.degrees(math.atan2(left_y, left_x)))+180)
    left_mag = min(int(math.hypot(left_x, left_y)*255),255)
    if left_mag > 100:
      send_angle = left_angle
    # print(left_angle, left_mag)
    
    # print(f"Left joystick: ({left_angle}, {left_mag})")
    # print(int(left_angle/360*255))
    message = int(send_angle/360*254)<<16 | int(left_mag)<<8 | 1
    # print(message.to_bytes(3, 'little'))

    ser.write(message.to_bytes(3, 'little'))
    # ser.write(f"{left_angle},{left_mag}\n".encode())

    time.sleep(0.05) # small delay to reduce processing load