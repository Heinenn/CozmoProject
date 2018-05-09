import cozmo
import cv2
import numpy as np
import logging
import asyncio
import sys
import PIL.ImageTk
import tkinter as tk

from threading import Timer
from scipy.interpolate import UnivariateSpline
from cozmo.util import degrees, distance_mm, speed_mmps

# Logger
log = logging.getLogger('ok.FollowLine')

# global variables
pathTimeout = 1;  # seconds to trigger watchdog
deadEndTurns = 0;  # dead-end turns
deadEndTurnsLimit = 3;  # dead-end turns before exit

# klassen
class Watchdog:
    def __init__(self, timeout, userHandler=None):  # timeout in seconds
        self.timeout = timeout
        self.handler = userHandler if userHandler is not None else self.defaultHandler
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()
        log.info('reseted watchdog timer')

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        raise self




class Main:
    def __init__(self):
        # Set-up logging
        formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s')
        handler = logging.StreamHandler()
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        log.setLevel(logging.INFO)
        log.addHandler(handler)
        # variables
        self.endProgramm = False;
        self._robot = None
        self._tk_root = 0
        self._tk_label_input = 0
        self._tk_label_output = 0
        cozmo.connect(self.run)

        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)

    def myHandler(self):
        print("watchdog timer expired")
        self.endProgramm = True;

    def on_img(self, event, *, image: cozmo.world.CameraImage, **kw):
        raw_img = image.raw_image
        raw_rgb = np.array(raw_img)
        r, g, b = cv2.split(raw_rgb)
        hsv_img = cv2.cvtColor(np.array(raw_img), cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv_img)
        mer_img = cv2.merge((h, s, v))
        hsv_img = mer_img
        rgb_img2 = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
        rgb_img3 = rgb_img2

        # watchdog timer object
        watchdog = Watchdog(pathTimeout, self.myHandler())

        try:
            #while True:
            # Crop the image
            Slices = 10
            crop_img = self.CropImage(rgb_img3, Slices)
            #cv2.imwrite('CropImage.png', crop_img)

            # Convert to grayscale
            gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

            # Gaussian blur
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Color thresholding
            ret, thresh1 = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
            #cv2.imwrite('ThresholdImageWithAccidentalDetections.png', thresh1)

            # Erode and dilate to remove accidental line detections
            mask = cv2.erode(thresh1, None, iterations=9)
            mask = cv2.dilate(mask, None, iterations=9)

            #TODO(Bild): Zeile unten benutzten um ganzes Bild zu speichern
            #cv2.imwrite('ThresholdImageWithoutAccidentalDetections.png', mask)
            # Find the contours of the frame
            #contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
            _, contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

            # Find the biggest contour (if detected)
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
                cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
                cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)

                speed = 30
                turnspeed = 40

                maxWindow = 312
                #windowPart = int(maxWindow/3)

                if cx >= 234:
                    log.info('strong right')
                    #TODO: wenn die zeile weiter unten aktiviert ist, dann macht er das zwar, der Kopf geht aber nach oben => FIXEN
                    #self._robot.turn_in_place(degrees(int(-5))).wait_for_completed()
                    self._robot.drive_wheel_motors(int(speed), int(speed - turnspeed))
                if cx >= 192 and cx < 234:
                    log.info('lite right')
                    # TODO: wenn die zeile weiter unten aktiviert ist, dann macht er das zwar, der Kopf geht aber nach oben => FIXEN
                    # self._robot.turn_in_place(degrees(int(-5))).wait_for_completed()
                    self._robot.drive_wheel_motors(int(speed), int(speed - turnspeed + 10))
                if cx < 192 and cx > 120:
                    log.info('on track')
                    #TODO: wenn die zeile weiter unten aktiviert ist, dann macht er das zwar, der Kopf geht aber nach oben => FIXEN
                    #self._robot.drive_straight(distance_mm(10), speed_mmps(200)).wait_for_completed()
                    self._robot.drive_wheel_motors(int(speed+20), int(speed+20))
                if cx <= 120 and cx > 78:
                    log.info('lite left')
                    # TODO: wenn die zeile weiter unten aktiviert ist, dann macht er das zwar, der Kopf geht aber nach oben => FIXEN
                    # self._robot.turn_in_place(degrees(int(5))).wait_for_completed()
                    self._robot.drive_wheel_motors(int(speed - turnspeed + 10), int(speed))
                if cx <= 78:
                    log.info('strong left')
                    #TODO: wenn die zeile weiter unten aktiviert ist, dann macht er das zwar, der Kopf geht aber nach oben => FIXEN
                    #self._robot.turn_in_place(degrees(int(5))).wait_for_completed()
                    self._robot.drive_wheel_motors(int(speed - turnspeed), int(speed))
                #reset watchdog timer)
                watchdog.reset()
                self.endProgramm = False;
            else:
                log.info('nothing to see here')
                if self.endProgramm:
                 #   if int(deadEndTurns) >= int(deadEndTurnsLimit):
                 #       sys.exit("killed by watchdog")
                 #   else:
                 #       deadEndTurns = int(deadEndTurns+1)
                 #       self._robot.drive_wheel_motors(50, -50);     #TODO: 180 degree turn


            #millis = int(round(time.time() * 1000))
            #cv2.imwrite(str(millis) + '.png', crop_img)
            PilImageForTk = PIL.Image.fromarray(crop_img)
            display_image = PIL.ImageTk.PhotoImage(image=PilImageForTk)
            self._tk_label_input.imgtk = display_image
            self._tk_label_input.configure(image=display_image)
            self._tk_root.update()
        finally:
            # Clean up the connection
            cv2.destroyAllWindows()

    def CropImage(self, inputImage, Slices):
        images = []
        for i in range(Slices):
            images.append(inputImage)
        height, width = inputImage.shape[:2]
        sl = int(height / Slices)
        for i in range(Slices):
            part = sl * i
            crop_img = inputImage[part:part + sl, 0:width]
            images[i] = crop_img
        #TODO: die Zeile weiter unten bestimmt mit den zwei zahlen wie groß, und wie nach der Bereich der Kamera ist
        #TODO: 1 ist Minimum, da 0 quasi nur seine Beine sind; es geht maximal bis 9
        ImagePart = self.ConnectImages(images, 1, 5)
        # TODO: Zeile unten benutzten um Bild mit dem Bildausschnitt zu speichern, den er verwendet zum pfad suchen
        # nach "TODO(Bild)" filtern um ganzes Bild zu speichern [sollte momentan Zeile 98 sein]
        #cv2. imwrite('ImagePart.png', ImagePart)
        return ImagePart

    def ConnectImages(self, images, start, end):
        imgstart = len(images) - 1 - start
        imgend = len(images) - 1 - end
        img = images[imgend]
        #cv2. imwrite('CropImagePart0.png', img)
        for i in range(1, int(end - start + 1)):
            img = np.concatenate((img, images[imgend + i]), axis=0)
            #cv2. imwrite('CropImagePart' + str(i) + '.png', img)
        return img

    def create_LUT_8UC1(self, x, y):
        spl = UnivariateSpline(x, y)
        return spl(range(256))



    async def set_up_cozmo(self, coz_conn):
        # TODO: setzt die Parameter für Cozmo (Funktioniert, kann aber nützlich zum nachschauen von Sachen für das "Kopf hoch"-Problem sein, deshalb TODO)
        asyncio.set_event_loop(coz_conn._loop)
        self._robot = await coz_conn.wait_for_robot()
        self._robot.camera.image_stream_enabled = True
        self._robot.camera.color_image_enabled = True
        await self._robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()

        self._robot.set_lift_height(1.0).wait_for_completed()

        self._robot.add_event_handler(cozmo.world.EvtNewCameraImage, self.on_img)

    async def run(self, coz_conn):
        await self.set_up_cozmo(coz_conn)

        self._tk_root = tk.Tk()
        self._tk_label_input = tk.Label(self._tk_root)
        self._tk_label_input.pack()

        while True:
            await asyncio.sleep(0)




if __name__ == '__main__':
    Main()
