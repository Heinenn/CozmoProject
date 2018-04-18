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

   #for i in range(0,10):
   #   img = np.array(robot.world.latest_image.raw_image)
   #  cv2.imwrite(str('test' + str(i) + '.png'), img)
   #   robot.drive_straight(distance_mm(50), speed_mmps(200)).wait_for_completed()

   img = cv2.imread('test0.jpg')
   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   edges = cv2.Canny(gray, 50, 150, apertureSize=3)
   lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)
   for r, theta in lines[0]:
      a = np.cos(theta)
      b = np.sin(theta)
      x0 = a * r
      y0 = b * r
      x1 = int(x0 + 1000 * (-b))
      y1 = int(y0 + 1000 * (a))
      x2 = int(x0 - 1000 * (-b))
      y2 = int(y0 - 1000 * (a))
      cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
   cv2.imwrite('linesDetected.jpg', img)

def cozmo_program(robot: cozmo.robot.Robot):
    capture(robot)


cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
