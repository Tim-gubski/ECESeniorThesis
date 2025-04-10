import mediapipe as mp
import cv2
import time # to track fps

class posetracking:
    def __init__(self,static_image_mode = False,model_complexity = 1,smooth_landmarks = True,enable_segmentation = False,smooth_segmentation = True,min_detection_confidence  = 0.5,min_tracking_confidence=0.4):

        self.model_complexity = model_complexity
        self.smooth_landmarks = smooth_landmarks
        self.enable_segmentation = enable_segmentation
        self.smooth_segmentation = smooth_segmentation
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_detection_confidence

        self.mpPose = mp.solutions.pose # use to detect pose landmarks
        self.pose = self.mpPose.Pose(self.model_complexity,self.smooth_landmarks,self.enable_segmentation,self.smooth_segmentation,self.min_tracking_confidence)
        self.mpDraw = mp.solutions.drawing_utils # functions for visualizing the detected landmarks and segments on the image.


    # finding pose
    def findPose(self,img,draw=True):
        self.RGBimg = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.result = self.pose.process(self.RGBimg)

        if self.result.pose_landmarks:
            # for poseLms in self.result.pose_landmarks:
                self.mypose = self.mpDraw.draw_landmarks(img,self.result.pose_landmarks,self.mpPose.POSE_CONNECTIONS)

        return img

    # getting potions ========
    def findposition(self, img, draw=True):
        lst = []
        if self.result.pose_landmarks:
            for id,lm in enumerate(self.result.pose_landmarks.landmark):
                h,w,c = img.shape
                cx,cy = int(lm.x * w), int(lm.y * h)
                lst.append([id,cx,cy])
                cv2.circle(img,(cx,cy),5,(0,0,255),cv2.FILLED)
                cv2.putText(img, str(id), (cx + 5, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        return lst

# --------------------------end class




#python main-----------

if __name__ == "__main__":
    # some variables
    cTime = 0
    pTime = 0

    cap = cv2.VideoCapture(0)

    # clas obj
    detector = posetracking()
    while True:
        success,frame = cap.read()
        if not success:
            break

        frame = detector.findPose(frame)
        lst = detector.findposition(frame)
        print([i for i in lst if i[0] in [15,16]])


        cTime = time.time()
        fps = 1/(cTime - pTime)
        pTime = cTime

        cv2.putText(frame,str(int(fps)), (70, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        cv2.imshow('video',frame)
        if cv2.waitKey(20) & 0xFF == ord("d"):
            break