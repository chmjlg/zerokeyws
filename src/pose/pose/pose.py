import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
import sys
import time
from rclpy.client import Client
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose, PoseStamped
import serial


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.cli = self.create_client(GetEntityState, '/get_entity_state')
        
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = GetEntityState.Request()
        self.pose = PoseStamped()
        
        self.publisher_ = self.create_publisher(PoseStamped, 'topic', 1)
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        #timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        
        self.ser.write(b'REBOOT\n')
        i = 0
        while i < 10:
            
            bs = self.ser.readline()
            i += 1
        while True:
            bs = self.ser.readline()
            ba = bytearray(bs)
            decoded_ba = bs.decode('UTF8')
            # print(decoded_ba)
            pos = decoded_ba.split(":")
            
            if(len(pos) > 13):
                #print(pos[10])
                print("pf")
                conf = pos[14].split(" ")
                print(conf)
                conf = float(conf[1])
                print(pos[14])
                xyz = pos[10].split(",")
                if(len(xyz) > 2):
                    #print(xyz)
                    x = float(xyz[0])
                    y = float(xyz[1])
                    z_part = xyz[2]
                    z_part = z_part.split(' ')
                    z = float(z_part[0])
                    print('x: ', x,'\n', 'y: ', y)
                    self.pose._header._stamp = self.get_clock().now().to_msg()
                    self.pose._pose._position._x = x
                    self.pose._pose._position._y = y
                    self.pose._pose._position._z = z
                    self.pose._pose._orientation.x = conf
                    self.publisher_.publish(self.pose)
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        
        # Client = self.create_client(GetEntityState, '/get_entity_state')
        # state = GetEntityState.Request()
        self.req._name = "ae1932"
        #self.req._reference_frame = "world"
        while 1:
            self.future = self.cli.call_async(self.req)
            #print(self.i)
            while rclpy.ok():
                #self.get_logger().info('TRYING TO GET CUBE POSE!')
                rclpy.spin_until_future_complete(self, self.future)
                #result = spin_until_service_complete(node, response)
                #self.get_logger().info('GOT CUBE POSE!\n{}'.format(self.future.result()))
                #print(self.future.result().state.pose)
                if self.future.done():
                    break
            self.pose._header._stamp = self.get_clock().now().to_msg()#self.future.result().header
            self.pose._pose = self.future.result().state.pose
            self.publisher_.publish(self.pose)
        
            #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.timer_callback()
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    #rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()