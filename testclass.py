# imports
import cozmo
import logging
import numpy as np

import cv2
import time

#import helpers.Image
from helpers.Image import Image
from helpers.Utils import *
from PIL import Image
from cozmo.util import degrees, distance_mm, speed_mmps

# Logger
log = logging.getLogger('ok.linefollow')

font = cv2.FONT_HERSHEY_SIMPLEX
direction = 0
Images=[]
N_SLICES = 4

for q in range(N_SLICES):
    Images.append(Image.Image())



def capture(robot: cozmo.robot.Robot):
    """ Capture an image of the path and show bounding rect of the
       contour of the path.
    """

    log.info('Capture image...')
    robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
    robot.camera.image_stream_enabled = True
    robot.set_lift_height(1.0).wait_for_completed()

    for i in range(0, 10):
        img = np.array(robot.world.latest_image.raw_image)

        array = np.frombuffer(img, dtype='uint8')
        img = cv2.imdecode(array, 1)
        direction = 0
        img = RemoveBackground(img, False)
        if img is not None:
            t1 = time.clock()
            SlicePart(img, Images, N_SLICES)
            for i in range(N_SLICES):
                direction += Images[i].dir

            fm = RepackImages(Images)
            t2 = time.clock()
            cv2.putText(fm, "Time: " + str((t2 - t1) * 1000) + " ms", (10, 470), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow("Vision Race", fm)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break




        #cv2.imwrite(str('test' + str(i) + '.png'), img)
        #robot.drive_straight(distance_mm(50), speed_mmps(200)).wait_for_completed()


def cozmo_program(robot: cozmo.robot.Robot):
    capture(robot)


cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)
