import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from rclpy import qos
from time import sleep
import time
import copy
import math
from .capston_packet_handler import PacketHandler
from capston_msgs.msg import Motorcapston
# from std_msgs.msg import Bool


class CapstonMobileNode(Node):
  def __init__(self):
    super().__init__('capston_mobile_motor_node')
      
    _port_name = '/dev/ttyMCU'
    _port_baudrate = 115200
    print('PORT NAME:\t\t%s'%(_port_name))
    print('BAUDRATE:\t\t%s'%(_port_baudrate))
    self.ph = PacketHandler(_port_name, _port_baudrate)
    
    self.subMotor = self.create_subscription(Motorcapston, 'start_robot', self.cbMotor, qos.qos_profile_sensor_data)
    
    self.is_cleaner = False
    self.is_brush = False
    self.is_water = False
    
  def cbMotor(self, msg):
    self.is_cleaner = msg.cleaner
    self.is_brush = msg.brush
    self.is_water = msg.water
    self.ph.write_motor(self.is_cleaner, self.is_brush, self.is_water)
    print(f"get data from topic cleaner : {self.is_cleaner}, brush : {self.is_brush}, water : {self.is_water}")
    
  # def updateStates(self):
  #   print("")
  
def main(args=None):
  rclpy.init(args=args)
  capston = CapstonMobileNode()
  rclpy.spin(capston)

  capston.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
