import asyncio
import cozmo
import cv2
import getopt
import logging
import numpy as np
import sys
import threading
import time

import PIL.Image
import PIL.ImageFont
import PIL.ImageTk
from scipy.interpolate import UnivariateSpline
import tkinter as tk

from cozmo.util import degrees, distance_mm, speed_mmps

from NH_Image import Image as Image2
from NH_Utils import *

# Logger
log = logging.getLogger('ok.linefollow')

def cozmo_cli(robot: cozmo.robot.Robot):
    """ Main loop implementing simplistic CLI
    """
    robot.stop_all_motors()
    robot.set_head_light(True)
    robot.camera.image_stream_enabled = True
    robot.set_lift_height(1.0).wait_for_completed()
    robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
    while True:
        run_cmd=input('C>')
        if run_cmd == 's':
          go_Cozmo(robot)
        if run_cmd == 'e':
            robot.stop_all_motors()
            cv2.destroyAllWindows()
            print('Bye.')
            break


def go_Cozmo(robot: cozmo.robot.Robot):
    """ Capture an image of the path and show bounding rect of the
        contour of the path.
    """
    log.info('start')
    font = cv2.FONT_HERSHEY_SIMPLEX
    direction = 0
    Images = []
    N_SLICES = 4

    for q in range(N_SLICES):
         Images.append(Image())


    log.info('before true')
    while True:

         #log.info('after true')
         img = np.array(robot.world.latest_image.raw_image)
         array = np.frombuffer(img, dtype='uint8')
         img = cv2.imdecode(array, 1)
         direction = 0
         img = RemoveBackground(img, False)
         #log.info('before image')
         if img is not None:
            log.info('after image')
            t1 = time.clock()
            SlicePart(img, Images, N_SLICES)
            for i in range(N_SLICES):
                direction += Images[i].dir

            log.info(direction)
            fm = RepackImages(Images)
            t2 = time.clock()
            cv2.putText(fm, "Time: " + str((t2 - t1) * 1000) + " ms", (10, 470), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow("Vision Race", fm)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


def main(argv):
    # Set-up logging
    formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s')
    handler = logging.StreamHandler()
    handler.setLevel(logging.INFO)
    handler.setFormatter(formatter)
    log.setLevel(logging.INFO)
    log.addHandler(handler)
    # Eval command line
    usecase='cli'
    try:
        opts, args = getopt.getopt(argv,'hu:',['usecase='])
    except getopt.GetoptError:
        print('line_follow.py -u <usecase>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('line_follow.py -u <usecase>')
            sys.exit()
        elif opt in ("-u", "--usecase"):
            usecase=arg
    if usecase=='cli':
        cozmo.run_program(cozmo_cli)


#if __name__ == '__main__':
#     main(sys.argv[1:])




class Thresh:
    def __init__(self):
        self._robot = None
        self._tk_root = 0
        self._tk_label_input = 0
        self._tk_label_output = 0
        cozmo.connect(self.run)

    def on_img(self, event, *, image: cozmo.world.CameraImage, **kw):

        raw_img = image.raw_image

        raw_rgb = np.array(raw_img)

        r, g, b = cv2.split(raw_rgb)

        hsv_img = cv2.cvtColor(np.array(raw_img), cv2.COLOR_RGB2HSV)

        h, s, v = cv2.split(hsv_img)

        mer_img = cv2.merge((h, s, v))

        hsv_img = mer_img

        rgb_img2 = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)

        invGamma = 1.0 / ((6 + 1) / 50)#self._tk_ga_scale.get()#77
        new = np.zeros(256)
        ori = np.zeros(256)
        for i in range(256):
            new[i] = ((i / 255.0) ** invGamma) * 255
            ori[i] = i
        try:
            incr_ch_lut = self.create_LUT_8UC1(ori, new)
            low = np.array([0,0,0], dtype="uint8")#[self._tk_hl_scale.get(),self._tk_sl_scale.get(),self._tk_vl_scale.get()]
        except:
            sys.exit('Window Closed - Exiting')
        high = np.array([0,0,0], dtype="uint8")#255,255,252

        rgb_img = cv2.LUT(raw_rgb, incr_ch_lut).astype(np.uint8)
        rgb_img2 = cv2.LUT(rgb_img2, incr_ch_lut).astype(np.uint8)

        rgb_img = cv2.blur(rgb_img, (3, 3))

        thresh_img = cv2.inRange(rgb_img2, low, high)

        raw_rgb_conv = cv2.cvtColor(np.array(raw_img), cv2.COLOR_BGR2RGB)
        rgb_img_conv = cv2.cvtColor(np.array(rgb_img), cv2.COLOR_BGR2RGB)
        rgb_img2_conv = cv2.cvtColor(np.array(rgb_img2), cv2.COLOR_BGR2RGB)
        #cv2.imwrite('raw_img.png', raw_rgb_conv)
        #cv2.imwrite('rgb_img.png', rgb_img_conv)
        #cv2.imwrite('rgb_img2.png', rgb_img2_conv)

        pil_thresh = PIL.Image.fromarray(cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2RGB))
        rgb_img = PIL.Image.fromarray(rgb_img)
        rgb_img2 = PIL.Image.fromarray(rgb_img2)

        display_image_input = PIL.ImageTk.PhotoImage(image=pil_thresh)
        display_image_output = PIL.ImageTk.PhotoImage(image=rgb_img2)
        self._tk_label_input.imgtk = display_image_input
        self._tk_label_input.configure(image=display_image_input)
        self._tk_label_output.imgtk = display_image_output
        self._tk_label_output.configure(image=display_image_output)
        self._tk_root.update()

        log.info('start')
        font = cv2.FONT_HERSHEY_SIMPLEX
        direction = 0
        Images = []
        N_SLICES = 4

        #for q in range(N_SLICES):
        #    Images.append(Image2.Image())

        #log.info('before true')
        #while True:

            #array = np.array(rgb_img2, dtype='uint8')

            #img = np.array(rgb_img2)
            #img = img[:, :, ::-1].copy()

            #img = cv2.imdecode(open_cv_image, 1)
            #direction = 0
            #img = RemoveBackground(img, False)
            #if rgb_img_conv is not None:
                #t1 = time.clock()
                #SlicePart(rgb_img_conv, Images, N_SLICES)
                #for i in range(N_SLICES):
                    #direction += Images[i].dir
                #log.info(direction)

    def create_LUT_8UC1(self, x, y):
        spl = UnivariateSpline(x, y)
        return spl(range(256))

    async def set_up_cozmo(self, coz_conn):
        asyncio.set_event_loop(coz_conn._loop)
        self._robot = await coz_conn.wait_for_robot()
        self._robot.camera.image_stream_enabled = True
        self._robot.camera.color_image_enabled = True
        self._robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()

        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)

    async def run(self, coz_conn):
        await self.set_up_cozmo(coz_conn)

        self._tk_root = tk.Tk()
        self._tk_label_input = tk.Label(self._tk_root)
        self._tk_label_output = tk.Label(self._tk_root)
        #self._tk_ga_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='ga')
        #self._tk_hl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='hl')
        #self._tk_sl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='sl')
        #self._tk_vl_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='vl')
        #self._tk_hh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='hh')
        #self._tk_sh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='sh')
        #self._tk_vh_scale = tk.Scale(self._tk_root, from_=0, to=255, orient=tk.HORIZONTAL, length=300, label='vh')
        self._tk_label_input.pack()
        self._tk_label_output.pack()
        #self._tk_ga_scale.pack()
        #self._tk_hl_scale.pack()
        #self._tk_sl_scale.pack()
        #self._tk_vl_scale.pack()
        #self._tk_hh_scale.pack()
        #self._tk_sh_scale.pack()
        #self._tk_vh_scale.pack()

        while True:
            await asyncio.sleep(0)


if __name__ == '__main__':
    Thresh()
