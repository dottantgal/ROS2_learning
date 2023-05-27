import rclpy
import cv2  
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


class program(Node):
    def __init__(self):
        super().__init__('human_tracking')
        self.pub_camera = self.create_publisher(Image,'/webcam',100) #publish the camera image data to the topic(Note:Make sure the topic name is not the same as your robot topic , so you can use the webcam)
        self.image = cv2.VideoCapture(0) #reading from our webcam.
        print(self.image.isOpened())
        self.store_bridge = CvBridge()
        

    #create a method that executes the code.
    def show(self):

        if not self.image.isOpened():
            print("This camera can't be opened")
                
        while True:
            __ret,frame = self.image.read()
            cv2.imshow("frame_name",frame)
            msg = self.store_bridge.cv2_to_imgmsg(frame,"bgr8") 
            self.pub_camera.publish(msg) 
            #cv2.imshow("frame_name",frame)
            

            #if statement that tells what key to press to stop the node.
            if cv2.waitKey(20) and 0xFF == ord('q'):
                break

        self.image.release()
        # cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    store = program() 
    store.show()
    rclpy.spin(store)


if __name__ == '__main__':
    main()