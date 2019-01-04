#!/usr/bin/env python

# ROS includes
import rospy
from std_msgs.msg import UInt64

# Script includes
import os
from servowrapper import Usb2Dynamixel
from servowrapper import AX12Servo
from functools import partial

class PubNode():

  def __init__(self, servos, usb_dynamixel):
    self.servos = servos
    self.usb_dynamixel = usb_dynamixel

  def run(self):
    # Initialize the node
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(60) # 1hz

    # Create a publishers for each servo
    pubs = {}
    topics = {}
    for id in self.servos.keys():
      topics[id] = 'dyna_controller_{}'.format(str(id))
      pubs[id] = rospy.Publisher(topics[id], UInt64, queue_size=10)

    # Loop until shutdown
    while not rospy.is_shutdown():
      # Get data from each connected servo and publish to appropriate channel
      for id in self.servos.keys():
        data = self.servos[id].get_present_position(self.usb_dynamixel)
        rospy.loginfo("Sending to {}: {}".format(topics[id], data))
        pubs[id].publish(data)
      rate.sleep()

if __name__ == '__main__':

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

  # Create the pub node
  pubNode = PubNode(servos, usb_dynamixel)

  try:
    pubNode.run()
  except rospy.ROSInterruptException:
    pass
