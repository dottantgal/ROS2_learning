import rclpy
import cv2  
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist

class program(Node):
    def __init__(self):
        super().__init__('face_detection')
        self.pub_camera = self.create_publisher(Image,'/webcam',100) 
        self.image = cv2.VideoCapture(0) #reading from the webcam.
        self.face_cascade = cv2.CascadeClassifier('/home/magnum/opencv_ws/src/ros2_cv/ros2_cv/facedetection.xml') #Add the path to your open_cv face detection.xml classifier.
        print(self.image.isOpened())
        self.store_bridge = CvBridge() 
        

    #create a method that executes the code.
    def show(self):

        if not self.image.isOpened():
            print("This camera can't be opened")

        while True:
            __ret,frame = self.image.read()
            reading =self.face_cascade.detectMultiScale(frame)
            for (x,y,w,h) in reading:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            print(frame[0]) 


            cv2.imshow("frame",frame)
            self.msg = self.store_bridge.cv2_to_imgmsg(frame,"bgr8")
            self.pub_camera.publish(self.msg)
            #cv2.imshow("frame_name",frame)
            
            if cv2.waitKey(20) and 0xFF == ord('q'):
                break

        self.image.release()
        #cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    store = program() 
    store.show()
    rclpy.spin(store)


if __name__ == '__main__':
    main()