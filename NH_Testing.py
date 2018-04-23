import cozmo
import cv2
import numpy as np
import logging
import asyncio

from scipy.interpolate import UnivariateSpline

from NH_Image import *
from NH_Utils import *

from cozmo.util import degrees, distance_mm, speed_mmps

# Logger
log = logging.getLogger('ok.FollowLine')


class Main:
    def __init__(self):
        # Set-up logging
        formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s')
        handler = logging.StreamHandler()
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        log.setLevel(logging.INFO)
        log.addHandler(handler)

        self._robot = None
        cozmo.connect(self.run)

        self._robot.camera.image_stream_enabled = True
        self._robot.camera.color_image_enabled = True
        self._robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()

        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)

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
        invGamma = 1.0 / ((77 + 1) / 50)#self._tk_ga_scale.get()
        new = np.zeros(256)
        ori = np.zeros(256)
        for i in range(256):
            new[i] = ((i / 255.0) ** invGamma) * 255
            ori[i] = i
        try:
            incr_ch_lut = self.create_LUT_8UC1(ori, new)
            low = np.array([0,0,0], dtype="uint8")#[self._tk_hl_scale.get(),self._tk_sl_scale.get(),self._tk_vl_scale.get()]
        except:
            #sys.exit('Window Closed - Exiting')
            log.info('ex')
        high = np.array([255,255,252], dtype="uint8")#[self._tk_hh_scale.get(), self._tk_sh_scale.get(), self._tk_vh_scale.get()]

        rgb_img = cv2.LUT(raw_rgb, incr_ch_lut).astype(np.uint8)
        rgb_img2 = cv2.LUT(rgb_img2, incr_ch_lut).astype(np.uint8)

        rgb_img = cv2.blur(rgb_img, (3, 3))

        thresh_img = cv2.inRange(rgb_img2, low, high)

        font = cv2.FONT_HERSHEY_SIMPLEX
        direction = 0
        Images = []
        N_SLICES = 4

        for q in range(N_SLICES):
            Images.append(Image())

        try:
            while True:
                direction = 0
                if rgb_img2 is not None:
                    SlicePart(rgb_img2, Images, N_SLICES)
                    for i in range(N_SLICES):
                        direction += Images[i].dir
                    log.info(direction)

                    #self._robot.turn_in_place(degrees(int(direction))).wait_for_completed()
                    #self._robot.drive_straight(distance_mm(10), speed_mmps(200)).wait_for_completed()
                else:
                    break

        finally:
            # Clean up the connection
            cv2.destroyAllWindows()

    def create_LUT_8UC1(self, x, y):
        spl = UnivariateSpline(x, y)
        return spl(range(256))

    async def set_up_cozmo(self, coz_conn):
        asyncio.set_event_loop(coz_conn._loop)
        self._robot = await coz_conn.wait_for_robot()
        self._robot.camera.image_stream_enabled = True
        self._robot.camera.color_image_enabled = True
        await self._robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()

        self._robot.set_lift_height(1.0).wait_for_completed()

        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)

    async def run(self, coz_conn):
        await self.set_up_cozmo(coz_conn)

        while True:
            await asyncio.sleep(0)



if __name__ == '__main__':
    Main()
