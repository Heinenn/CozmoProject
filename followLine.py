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
   img = np.array(robot.world.latest_image.raw_image)
   #print(img)
   #print(robot.world.latest_image.raw_image)
   #img = robot.world.latest_image.raw_image
   #im = Image.fromarray(img)
   #im.save("C:\001_DHBW\your_file.jpeg")
   cv2.imwrite('test.png', img)
   img2 = Image.open('test.png')
   img2.show()
   #img1 = robot.world.latest_image.raw_image
   #img2 = cv2.imread(img, 1)
   #cv2.imshow('img', img2)
   #cv2.waitKey(0)
   #rect_img = draw_grid(img)
   #th, b_cnt_rects=get_cnt_rects(rect_img, b_zn)
   #rect_img = draw_cnt_rects(rect_img, b_cnt_rects, (0,255,255))
   #rect_img = draw_path_rect(rect_img, get_path_rect(rect_img, b_zn),(0,255,0))
   #th, l_cnt_rects=get_cnt_rects(rect_img, l_zn)
   #rect_img = draw_cnt_rects(rect_img, l_cnt_rects, (0,255,255))
   #rect_img = draw_path_rect(rect_img, get_path_rect(rect_img, l_zn),(0,255,0))
   #th, r_cnt_rects=get_cnt_rects(rect_img, r_zn)
   #rect_img = draw_cnt_rects(rect_img, r_cnt_rects, (0,255,255))
   #rect_img = draw_path_rect(rect_img, get_path_rect(rect_img, r_zn),(0,255,0))
   #cv2.imshow('rect_img', rect_img) 


def cozmo_program(robot: cozmo.robot.Robot):
    time.sleep(2)

    print("Turning off all annotations for 2 seconds")
    robot.world.image_annotator.annotation_enabled = False
    time.sleep(2)

    print('Re-enabling all annotations')
    robot.world.image_annotator.annotation_enabled = True

    # Disable the face annotator after 10 seconds
    time.sleep(10)
    print("Disabling face annotations (light cubes still annotated)")
    robot.world.image_annotator.disable_annotator('faces')

    capture(robot)


cozmo.run_program(cozmo_program, use_viewer=True, force_viewer_on_top=True)



#cozmo.run_program(capture)
