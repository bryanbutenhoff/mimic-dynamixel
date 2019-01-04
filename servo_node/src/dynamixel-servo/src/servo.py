#!/usr/bin/env python

# ROS includes
import rospy
from std_msgs.msg import UInt64

# Script includes
import os
from servowrapper import Usb2Dynamixel
from servowrapper import AX12Servo
from functools import partial

class SubNode:

  def __init__(self, servos, usb_dynamixel):
    self.servos = servos
    self.usb_dynamixel = usb_dynamixel

  def callback(self, servo, data):
    servo.set_goal_position(self.usb_dynamixel, 1024 - data.data)

  def run(self):
    # Initialize the node
    rospy.init_node('servo', anonymous=True)
    offset = 4
    for id in self.servos.keys():
      rospy.Subscriber('dyna_controller_{}'.format(str(id - offset)), UInt64, partial(self.callback, self.servos[id]))

    rospy.spin()

if __name__ == "__main__":

  # Create a Usb2Dynamixel for communication
  device_name = os.environ['TTL_DEVICE']
  baud_rate = 1000000
  protocol_version = 1.0
  usb_dynamixel = Usb2Dynamixel(device_name, baud_rate, protocol_version)

  # Create a servo instance for all defined IDs
  servos = {}
  ids = [int(id) for id in os.environ['DXL_IDS'].split(",")]
  for id in ids:
    servos[id] = AX12Servo(id)

  # Create a sub node
  subNode = SubNode(servos, usb_dynamixel)

  try:
    subNode.run()
  except rospy.ROSInterruptException:
    pass
