import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3
from math import radians

cross=0
red=0
class TurtlebotController:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('detected_classes', String, self.callback)
        self.command = Twist()

    def callback(self, msg):
        print(msg.data)
        if 'speedlimit' in msg.data:
            move = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0))
            self.pub.publish(move)
        elif 'crosswalk' in msg.data:
            global cross
            if cross==0:
                # move = Twist(linear=Vector3(0.13, 0, 0), angular=Vector3(0, 0, 0))
                # for x in range(0,5):
                #     self.pub.publish(move)
                #     rospy.sleep(1.)
                move = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0))
                for x in range(0,5):
                    self.pub.publish(move)
                    rospy.sleep(1.)
                move = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0))
                for x in range(0,5):
                    self.pub.publish(move)
                    rospy.sleep(1.)
                move = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                for x in range(0,5):
                    self.pub.publish(move)
                    rospy.sleep(1.)

                turn = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, radians(-10)))
                for x in range(0,1):
                    self.pub.publish(turn)
                    rospy.sleep(1.)
                cross=1
            else:
                move = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, 0))
                self.pub.publish(move)
        elif 'stop' in msg.data:
            move = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            self.pub.publish(move)
        elif 'red' in msg.data:
            global red
            if red==0:
                move = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                self.pub.publish(move)
                rospy.sleep(2.)
                red=1
            else:
                move = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, 0))
                self.pub.publish(move)
        elif 'green' in msg.data:
            move = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, 0))
            self.pub.publish(move)
        elif 'yellow' in msg.data:
            move = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0))
            self.pub.publish(move)

    def start(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    controller = TurtlebotController()
    controller.start()
