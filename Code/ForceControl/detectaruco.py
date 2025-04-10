import cv2
import numpy as np
import cv2.aruco as aruco
import serial
import threading

import leap
import time

# Load the image
# image = cv2.imread('/content/marker_42.png')

# Convert the image to grayscale
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


# # Create the ArUco detector
# detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
# # Detect the markers
# corners, ids, rejected = detector.detectMarkers(gray)
# # Print the detected markers
# print("Detected markers:", ids)
# if ids is not None:
#     cv2.aruco.drawDetectedMarkers(image, corners, ids)
#     cv2.imshow('Detected Markers', image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

class MyListener(leap.Listener):
    def on_connection_event(self, event):
        print("Connected")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Found device {info.serial}")

    def on_tracking_event(self, event):
        # print(f"Frame {event.tracking_frame_id} with {len(event.hands)} hands.")
        for hand in event.hands:
            hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
            handpos_mutex.acquire()
            handpos[0] = hand.palm.position.x
            handpos[1] = hand.palm.position.z
            # print(handpos)
            handpos_mutex.release()
            # print(
            #     f"Hand id {hand.id} is a {hand_type} hand with position ({hand.palm.position.x}, {hand.palm.position.y}, {hand.palm.position.z})."
            # )
        if len(event.hands) == 0:
            handpos_mutex.acquire()
            handpos[0] = 0
            handpos[1] = 0
            handpos_mutex.release()


# offset = -80

handpos_mutex = threading.Lock()
handpos = [0,0]

def detect_aruco():
    ser = serial.Serial('/dev/cu.usbmodem1401', 9600)

    # Load the predefined dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters = cv2.aruco.DetectorParameters()
    
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Capture video from webcam
    cap = cv2.VideoCapture(0)
    
    home = None
    offset = None
    last_position = [0,0]
    last_angle = 0
    

    # ser.write(message.to_bytes(3, 'little'))

    my_listener = MyListener()

    connection = leap.Connection()
    connection.add_listener(my_listener)

    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = detector.detectMarkers(gray)
            
            # Draw detected markers
            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
            
            centers = {}
            if ids is not None:
                for i in range(len(ids)):
                    c = corners[i][0]
                    x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                    y = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4
                    centers[ids[i][0]] = (x, y)

            if home != None and 0 in centers and offset == None:
                dx = centers[0][0] - home[0]
                dy = centers[0][1] - home[1]
                angle = (np.arctan2(dy, -dx) * 180 / np.pi + 180)%360
                offset = -angle
                print(f"Offset: {offset:.2f}")

            if home == None and 0 in centers:
                # calibration
                home = centers[0]
                print("Home set to", home)
                print("CALIBRATING")
                message = int(180/360*254)<<16 | int(0)<<8 | 1
                print(message.to_bytes(3, 'little'))
                ser.write(message.to_bytes(3, 'little'))
                time.sleep(0.5)
                message = int(180/360*254)<<16 | int(50)<<8 | 1
                print(message.to_bytes(3, 'little'))
                ser.write(message.to_bytes(3, 'little'))
                time.sleep(1)

                
            if home != None and 0 in centers and offset != None:
                x, y = centers[0]
                # error = observed_angle - last_angle
                # handpos_mutex.acquire()
                target = [home[0] + handpos[0]*6, home[1] + handpos[1]*6]
                # handpos_mutex.release()
                # print(target)
                dx = x - target[0]
                dy = y - target[1]
                # print(f"Delta: ({dx:.2f}, {dy:.2f})")
                angle = (np.arctan2(dy, -dx) * 180 / np.pi + 180 + offset)%360
                # last_angle = angle
                # print(f"Angle: {angle:.2f}")
                mag = min(150,np.hypot(dx, dy)/2)
                if abs(dx) < 60 and abs(dy) < 60:
                    mag = 0
                # print(f"Magnitude: {mag:.2f}")
                
                # send_angle = (angle + offset)%360

                message = int(angle/360*254)<<16 | int(mag)<<8 | 1
                # print(message.to_bytes(3, 'little'))
                ser.write(message.to_bytes(3, 'little'))
                
            # Display result
            cv2.imshow('ArUco Marker Detection', frame)
            time.sleep(0.05)
            
            # Exit on pressing 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco()
