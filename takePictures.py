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

   img = cv2.imread('test0.png')

   # Convert the img to grayscale
   gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

   # Apply edge detection method on the image
   edges = cv2.Canny(gray, 50, 150, apertureSize=3)

   # This returns an array of r and theta values
   lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

   # The below for loop runs till r and theta values
   # are in the range of the 2d array
   for r, theta in lines[0]:
      # Stores the value of cos(theta) in a
      a = np.cos(theta)

      # Stores the value of sin(theta) in b
      b = np.sin(theta)

      # x0 stores the value rcos(theta)
      x0 = a * r

      # y0 stores the value rsin(theta)
      y0 = b * r

      # x1 stores the rounded off value of (rcos(theta)-1000sin(theta))
      x1 = int(x0 + 1000 * (-b))

      # y1 stores the rounded off value of (rsin(theta)+1000cos(theta))
      y1 = int(y0 + 1000 * (a))

      # x2 stores the rounded off value of (rcos(theta)+1000sin(theta))
      x2 = int(x0 - 1000 * (-b))

      # y2 stores the rounded off value of (rsin(theta)-1000cos(theta))
      y2 = int(y0 - 1000 * (a))

      # cv2.line draws a line in img from the point(x1,y1) to (x2,y2).
      # (0,0,255) denotes the colour of the line to be
      # drawn. In this case, it is red.
      cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

   # All the changes made in the input image are finally
   # written on a new image houghlines.jpg
   cv2.imwrite('linesDetected.jpg', img)

def cozmo_program(robot: cozmo.robot.Robot):
    capture(robot)


cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
