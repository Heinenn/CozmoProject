# imports
import cv2
import sys
import time
import cozmo
import logging
import numpy as np
from PIL import Image
from cozmo.util import degrees, distance_mm, speed_mmps

# Logger
log = logging.getLogger('ok.linefollow')


def capture(robot: cozmo.robot.Robot):
   """ Capture an image of the path and show bounding rect of the 
      contour of the path.
   """
    
   log.info('Capture image...')
   robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
   robot.camera.image_stream_enabled = True
   robot.set_lift_height(1.0).wait_for_completed()

#   while True:
   #   img = np.array(robot.world.latest_image.raw_image)


#   for i in range(0,10): 
#      img = np.array(robot.world.latest_image.raw_image)
#      cv2.imwrite(str('test' + str(i) + '.png'), img)
#      robot.drive_straight(distance_mm(50), speed_mmps(200)).wait_for_completed()


def cozmo_program(robot: cozmo.robot.Robot):
    capture(robot)


cozmo.run_program(cozmo_program)
