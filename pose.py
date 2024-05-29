import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
import sys
import time
from rclpy.client import Client
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, PoseStamped
import serial
import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")

        self.upperPose = PoseStamped()
        self.bottomPose = PoseStamped()
        self.urpublisher_ = self.create_publisher(PoseStamped, "zk/upper_right/pose", 1)
        self.brpublisher_ = self.create_publisher(
            PoseStamped, "zk/bottom_right/pose", 1
        )
        self.theta = 0

        self.ser1 = serial.Serial("/dev/ttyACM2", 115200)
        self.ser2 = serial.Serial("/dev/ttyACM3", 115200)

        self.zkbottomright = 0
        self.zkupperright = 0

        self.uInitialX = 0
        self.uInitialY = 0
        self.uInitialZ = 0

        self.bInitialX = 0
        self.bInitialY = 0
        self.bInitialz = 0

        # self.initTheta = -32.6  # deg
        self.initTheta = 0

        while True:
            self.ser1.write(b"MFGMAC\n")
            self.ser2.write(b"MFGMAC\n")

            ser1response = self.ser1.readline()
            ser2response = self.ser2.readline()

            ser1ResponseStr = ser1response.decode("UTF8")
            ser2responseStr = ser2response.decode("UTF8")

            ser1ResponseSplit = ser1ResponseStr.split(":")
            ser2ResponseSplit = ser2responseStr.split(":")

            if (
                ser2ResponseSplit[0] == "|I| DEVICE MAC"
                and ser2ResponseSplit[1].replace(" ", "") == "E39D6D440F6B\n"
            ):
                cleanedmac = ser2ResponseSplit[1].replace(" ", "")
                print(cleanedmac)
                if cleanedmac == "E39D6D440F6B\n":
                    self.zkbottomright = self.ser2
                    self.zkupperright = self.ser1
                    break
            else:
                self.zkbottomright = self.ser1
                self.zkupperright = self.ser2
                break

        # timer_period = 1  # seconds
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        bottomX = 0
        bottomY = 0
        bottomZ = 0
        upperX = 0
        upperY = 0
        upperZ = 0
        first = False

        while True:
            zkBRResponse = self.zkbottomright.readline()
            zkURResponse = self.zkupperright.readline()
            #print(zkBRResponse)
            # print(zkURResponse)
            decodedUR = zkURResponse.decode("UTF8")
            decodedBR = zkBRResponse.decode("UTF8")

            splitUR = decodedUR.split(":")
            splitBR = decodedBR.split(":")

            if (
                splitUR[0] == "|I| process_positioning"
                and splitBR[0] == "|I| process_positioning"
            ):

                # Upper ZK point
                if splitUR[0] == "|I| process_positioning":

                    upperX = splitUR[2].split(" ")
                    upperY = splitUR[3].split(" ")
                    upperZ = splitUR[4].split(" ")
                    try:
                        float(upperX[1])
                        float(upperY[1])
                        float(upperZ[1])
                    except:
                        continue
                    else:
                        upperX = float(upperX[1])
                        upperY = -float(upperY[1])
                        upperZ = float(upperZ[1])

                        if not first:
                            self.uInitialX = upperX
                            self.uInitialY = upperY
                            self.uInitialZ = upperZ

                        else:
                            # print(float(upperX[1]), (float(upperY[1])), float(upperZ[1]))
                            self.upperPose._header._stamp = (self.get_clock().now().to_msg())  # GFR
                            self.upperPose._pose._position._x = (upperX - self.bInitialX)  # * math.sin(math.radians(124))
                            self.upperPose._pose._position._y = (upperY - self.bInitialY)  # * math.cos(math.radians(124))
                            self.upperPose._pose._position._z = upperZ - self.bInitialZ

                # Bottom ZeroKey config
                if splitBR[0] == "|I| process_positioning":
                    bottomX = splitBR[2].split(" ")
                    bottomY = splitBR[3].split(" ")
                    bottomZ = splitBR[4].split(" ")
                    try:
                        float(bottomX[1])
                        float(bottomY[1])
                        float(bottomZ[1])
                    except:
                        continue
                    else:
                        bottomX = float(bottomX[1])
                        bottomY = -float(bottomY[1])
                        bottomZ = float(bottomZ[1])

                        if first == False:
                            # Init x,y,z values of br zk
                            self.bInitialX = bottomX
                            self.bInitialY = bottomY
                            self.bInitialZ = bottomZ
                            print("boo!")

                            # Init theta offset
                            if (abs((self.uInitialY - self.bInitialY))) > 0.001:
                                self.initTheta = math.atan2((self.uInitialX - self.bInitialX),(self.uInitialY - self.bInitialY))
                                print("#############In Bottom Init")
                                print(math.degrees(self.initTheta))
                                print(upperX)
                                print(upperY)
                                print(upperZ)
                                print(bottomX)
                                print(bottomY)
                                print(bottomZ)
                                print("#############")
                                first = True

                        else:
                            # print(float(bottomX[1]), (float(bottomY[1])), float(bottomZ[1]))
                            newY = (bottomX - self.bInitialX) * math.cos((self.initTheta)) - (bottomY - self.bInitialY) * math.sin((self.initTheta))
                            newX = (bottomX - self.bInitialX) * math.sin((self.initTheta)) + (bottomY - self.bInitialY) * math.cos((self.initTheta))
                            self.bottomPose._header._stamp = (self.get_clock().now().to_msg())
                            self.bottomPose._pose._position._x = (newX)  # * math.sin(math.radians(124)))
                            self.bottomPose._pose._position._y = (newY)  # * math.cos(math.radians(124)))
                            self.bottomPose._pose._position._z = (bottomZ - self.bInitialZ)

                            # Bottom Orientation in radians
                            if (abs((upperY - bottomY))) > 0.001:
                                self.theta = math.atan2((upperX - bottomX), (upperY - bottomY))
                                self.bottomPose._pose._orientation.z = math.degrees(self.theta - self.initTheta)
                                # print(self.theta)

                            # Publish Bottom and upper pose
                            # self.pose._pose._orientation.x = conf
                            self.urpublisher_.publish(self.upperPose)
                            self.brpublisher_.publish(self.bottomPose)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
