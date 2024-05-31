import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
import sys
import time
from rclpy.client import Client

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

        self.ser1 = serial.Serial("/dev/ttyACM0", 115200)
        self.ser2 = serial.Serial("/dev/ttyACM1", 115200)

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
            # print(ser1response)
            # print(ser2response)

            if (
                ser2ResponseSplit[0]
                == "|I| DEVICE MAC"
                # and ser2ResponseSplit[1].replace(" ", "") == "ECF2386742C8\n"
            ):
                cleanedmac = ser2ResponseSplit[1].replace(" ", "")
                print(cleanedmac)
                if cleanedmac == "ECF2386742C8\n":
                    print("In")
                    self.zkbottomright = self.ser2
                    self.zkupperright = self.ser1
                    break
                else:
                    self.zkbottomright = self.ser1
                    self.zkupperright = self.ser2
                    break

        # init block
        self.uInitialX, self.uInitialY, self.uInitialZ = self.getPose(self.zkupperright)
        self.bInitialX, self.bInitialY, self.bInitialZ = self.getPose(
            self.zkbottomright
        )
        # Init theta offset
        if (abs((self.uInitialY - self.bInitialY))) > 0.001:
            self.initTheta = math.atan2(
                (self.uInitialX - self.bInitialX), (self.uInitialY - self.bInitialY)
            )

        print("#############In Bottom Init")
        print(math.degrees(self.initTheta))
        print(self.uInitialX, self.uInitialY, self.uInitialZ)
        print(self.bInitialX, self.bInitialY, self.bInitialZ)
        print("#############")

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
            bottomX, bottomY, bottomZ = self.getPose(self.zkbottomright)
            upperX, upperY, upperZ = self.getPose(self.zkupperright)

            # print(float(upperX[1]), (float(upperY[1])), float(upperZ[1]))
            self.upperPose._header._stamp = self.get_clock().now().to_msg()
            self.upperPose._pose._position._x = upperX - self.bInitialX
            self.upperPose._pose._position._y = upperY - self.bInitialY
            self.upperPose._pose._position._z = upperZ - self.bInitialZ
            # self.urpublisher_.publish(self.upperPose)

            # print(float(bottomX[1]), (float(bottomY[1])), float(bottomZ[1]))
            newY = (bottomX - self.bInitialX) * math.cos((self.initTheta)) - (
                bottomY - self.bInitialY
            ) * math.sin((self.initTheta))
            newX = (bottomX - self.bInitialX) * math.sin((self.initTheta)) + (
                bottomY - self.bInitialY
            ) * math.cos((self.initTheta))
            self.bottomPose._header._stamp = self.get_clock().now().to_msg()
            self.bottomPose._pose._position._x = newX
            self.bottomPose._pose._position._y = newY
            self.bottomPose._pose._position._z = bottomZ - self.bInitialZ

            # Bottom Orientation in radians
            if (abs((upperY - bottomY))) > 0.001:
                self.theta = math.atan2((upperX - bottomX), (upperY - bottomY))
                self.bottomPose._pose._orientation.z = math.degrees(
                    self.theta - self.initTheta
                )
                # print(self.theta)

            # Publish Bottom and upper pose
            # self.pose._pose._orientation.x = conf
            self.urpublisher_.publish(self.upperPose)
            self.brpublisher_.publish(self.bottomPose)

    # Return x,y,z coordinates of the passed in zerokey object
    def getPose(self, zeroKey):
        X = 0
        Y = 0
        Z = 0

        while True:
            zkResponse = zeroKey.readline()
            # print(len(zkResponse), zkResponse)
            # print(zkURResponse)
            decodedR = zkResponse.decode("UTF8")
            splitR = decodedR.split(":")

            # ZK point
            if splitR[0] == "|I| process_positioning": #and len(zkResponse) <= 135:

                X = splitR[2].split(" ")
                Y = splitR[3].split(" ")
                Z = splitR[4].split(" ")
                try:
                    float(X[1])
                    float(Y[1])
                    float(Z[1])
                except:
                    continue
                else:
                    X = float(X[1])
                    Y = float(Y[1])
                    Z = float(Z[1])
                    return X, Y, Z


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
