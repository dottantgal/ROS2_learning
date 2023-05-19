import rclpy
import cv2  
from rclpy.node import Node
from geometry_msgs.msg import Twist #A Twist messgage type to drive the robot.
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Twist

#store the cvBridge, which is a type of the cv_bridge package.
#create a class that stores all the object and the methods we will use.
class program(Node):
    def __init__(self):
        super().__init__('face_detection')
        self.pub_camera = self.create_publisher(Image,'/webcam',100) #publish the camera image data to the topic(Note:Make sure the topic name is not the same as your robot topic , so you can use the webcam)
        #self.pub_cmd = self.create_publisher(Twist,'/cmd_vel',100) #publishing to command veloaicty,to drive the robot.
        #self.velocity_message = Twist()
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
            print(frame[0]) #print the co-ordinates to the terminal.
            # if frame.any():
            #     self.velocity_message.linear.x = 0.2
            #     self.pub_cmd.publish(self.velocity_message)
            # else:
            #     self.velocity_message.linear.x = 0.0
            #     self.pub_cmd.publish(self.velocity_message)


            cv2.imshow("frame",frame)
            self.msg = self.store_bridge.cv2_to_imgmsg(frame,"bgr8") #This line is very important.
            self.pub_camera.publish(self.msg) #publish the message over to ros2.
            #cv2.imshow("frame_name",frame)
            

            #if statement that tells what key to press to stop the node.
            if cv2.waitKey(20) and 0xFF == ord('q'):
                break

        self.image.release()
        #cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    store = program() #passing in an object.
    store.show()
    rclpy.spin(store)


if __name__ == '__main__':
    main()