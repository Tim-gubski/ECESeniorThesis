import Leap
import time

class HandTrackingListener(Leap.Listener):
    def on_connect(self, controller):
        print("Connected to Leap Motion Controller")

    def on_frame(self, controller):
        frame = controller.frame()
        if not frame.hands.is_empty:
            for hand in frame.hands:
                hand_type = "Left" if hand.is_left else "Right"
                position = hand.palm_position
                print(f"{hand_type} hand detected at position: {position}")

if __name__ == "__main__":
    listener = HandTrackingListener()
    controller = Leap.Controller()
    
    controller.add_listener(listener)
    
    try:
        print("Tracking hands... Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        controller.remove_listener(listener)
