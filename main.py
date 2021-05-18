import cv2
import time
import os
import numpy as np
import math
from ctypes import cast, POINTER
# from comtypes import CLSCTX_ALL
# from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import ctypes
import pyautogui
import enum
import collections
from threading import Thread
from hand_detector import Hand_detector
LM_NAMES = Hand_detector.LM_NAMES


class Camera():
    """
        The class responsible for the window displaying the camera and for
	the hand detector.
    """
    def __init__(self, width: int = 1920, height: int = 1080):
        """
            Initializes the window displaying the camera and hand detector.
            :param width: camera width
            :param height: camera height
        """
        self.fps = 60
        self.width = width
        self.height = height
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.width)
        self.cap.set(4, self.height)
        self.detector = Hand_detector(detection_con=0.1, track_con=0.1)

    def draw_detected_fingers(self, x1: int, x2: int, y1: int, y2: int, frame):
        """
            Displays detected finger movements.
        """
        cx2, cy2 = (x2 + x1) // 2, (y2 + y1) // 2
        cv2.circle(frame, (x1, y1), 5, (255, 0, 255), cv2.FILLED)
        cv2.circle(frame, (x2, y2), 5, (255, 0, 255), cv2.FILLED)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.circle(frame, (cx2, cy2), 5, (255, 0, 255), cv2.FILLED)


class Volume():
    """
        Class for working with system sound volume.
        Stores devices, current volume.
    """
    def __init__(self, camera):
        self.camera = camera
#       self.devices = AudioUtilities.GetSpeakers()
#       self.interface = self.devices.Activate(
#               IAudioEndpointVolume._iid_, CLSCTX_ALL, None)

#        self.volume = cast(self.interface, POINTER(IAudioEndpointVolume))
#        volume_ranges = self.volume.GetVolumeRange()
#        self.min_volume = volume_ranges[0]
#        self.max_volume = volume_ranges[1]
        self.min_volume = -100
        self.max_volume = 0
        self.vol = 0  # Текущая громкость

    def set(self, lm_list: list, frame):
        """
            Setting the volume according to your finger movements.
        """
        x1, y1 = lm_list["thumb_tip"]["x"], lm_list["thumb_tip"]["y"]
        x2, y2 = lm_list["index_tip"]["x"], lm_list["index_tip"]["y"]

        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        self.camera.draw_detected_fingers(x1, x2, y1, y2, frame)

        length = math.hypot(x2 - x1, y2 - y1)

        self.vol = int(np.interp(length, [0, 150],
                                 [self.min_volume,
                                 self.max_volume]))
        if length < 150:
            cv2.circle(frame, (cx, cy), 15, (0, 255, 0), cv2.FILLED)
            print("set volume", self.vol)
            if self.vol >= -2:
                self.vol = 0
#            self.volume.SetMasterVolumeLevel(self.vol, None)


class Swipe():
    """
        The class responsible for actions associated with the movement of the
	hand along the axes.
    """
    def __init__(self):
        """
            Creating ring buffers storing the previous coordinates of the
		hand position.
        """
        self.MOTION_LIST_MAXLEN = 20
        self.motion_list_x = collections.deque(maxlen=self.MOTION_LIST_MAXLEN)
        self.motion_list_y = collections.deque(maxlen=self.MOTION_LIST_MAXLEN)
        self.last_time = time.time()

    def get_hand_move(self, lm_list: list, frame):
        """
            To observe the movement of the hand, follow the wrist (WRIST = 0)
        """
        def check_last_time_write():
            """
               Clearing the lists with the coordinates of hand movements if
			there were no records in the last 4 seconds.
            """
            if (time.time() - self.last_time) >= 4:
                self.motion_list_x.clear()
                self.motion_list_y.clear()
            self.last_time = time.time()

        check_last_time_write()

        self.motion_list_x.append(lm_list["wrist"]["x"])
        self.motion_list_y.append(lm_list["wrist"]["y"])
        print("motion_list_x:", self.motion_list_x)
        print("motion_list_y:", self.motion_list_y)
        if len(self.motion_list_x) == self.MOTION_LIST_MAXLEN:
            self.determine_direction(lm_list)

    def determine_direction(self, lm_list: list):
        """
           Determination of the direction of hand movement by the saved
		coordinates in the motion_list.
        """
        def calc_abs_x_and_y() -> int:
            """
	            Calculation of the absolute lengths of x and y (index 0 is
			taken as the origin point, the last element as the end point of
			the vector.
            """
            diff_x = self.motion_list_x[0] - self.motion_list_x[-1]
            diff_y = self.motion_list_y[0] - self.motion_list_y[-1]
            return (diff_x, diff_y)

        def calc_polar_coordinates(x: int, y: int) -> tuple:
            """
               Hand movement length radius, degree - direction.
            """
            polar_radius = np.sqrt(x**2 + y**2)  # длина
            polar_deg = np.arctan2(y, x)         # градус
            return (polar_radius, polar_deg)
        x, y = calc_abs_x_and_y()
        print("x", x)
        print("y", y)

        polar_radius, polar_deg = calc_polar_coordinates(x, y)
        print("polar_radius", polar_radius)
        print("polar_deg", polar_deg)
        if polar_radius < 100:  # Принимаются движения от 100 пикселей
            return

        def if_in_diaposon(value: int, diaposon: tuple) -> bool:
            return (value >= diaposon[0]) & (value <= diaposon[1])

        polar_degs = (0.0,             # right
                      math.pi / 4,     # right_up
                      math.pi / 2,     # up
                      3/4 * math.pi,   # up_left
                      math.pi,         # left
                      -3/4 * math.pi,  # left_bottom
                      -math.pi / 2,    # bottom
                      -math.pi / 4)    # bottom_right
        diaposones = [(i - math.pi / 8, i + math.pi / 8) for i in polar_degs]
        func_list = (self.right, self.right_up, self.up, self.up_left,
                     self.left, self.left_bottom, self.bottom,
                     self.bottom_right)

        for i in range(len(diaposones)):
            if if_in_diaposon(polar_deg, diaposones[i]):
                func_list[i](lm_list)
                return

    def up(self, lm_list: list):
        print("up")

    def right_up(self, lm_list: list):
        print("right_up")

    def right(self, lm_list: list):
        print("right")

    def bottom_right(self, lm_list: list):
        print("bottom_right")

    def bottom(self, lm_list: list):
        print("bottom")

    def left_bottom(self, lm_list: list):
        print("left_bottom")

    def left(self, lm_list: list):
        print("left")

    def up_left(self, lm_list: list):
        print("up_left")


class Controlling(Thread):
    """
        Class for controlling the program using gestures.
    """
    class Finger_id(enum.Enum):
        """
            For indexing curved fingers.
        """
        THUMB_TIP = 0
        INDEX_TIP = 1
        MIDDLE_TIP = 2
        RING_TIP = 3
        PINKY_TIP = 4

    def __init__(self, camera: Camera, volume: Volume, swipe: Swipe):
        super().__init__()
        self.camera = camera
        self.volume = volume
        self.swipe = swipe
        self.lock = True
        self.sleep_state = False
        self.is_running = True
        self.motions = {
            "volume": {
                        "function": self.volume.set,
                        "detect_function": self.volume.set,
                        "fingers": (self.Finger_id.THUMB_TIP.value,
                                    self.Finger_id.INDEX_TIP.value)
                        },
            "minimize": {
                        "function": self.minimize_all_windows,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("minimize"),
                        "fingers": (self.Finger_id.INDEX_TIP.value,
                                    self.Finger_id.MIDDLE_TIP.value),
                        },
            "lock": {
                        "function": self.lock_work_station,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("lock"),
                        "fingers": tuple()
                    },
            "switch": {
                        "function": self.switch_window,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("switch"),
                        "fingers": (self.Finger_id.INDEX_TIP.value,)
                      },
            "back": {
                        "function": self.back_window,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("back"),
                        "fingers": (self.Finger_id.INDEX_TIP.value,
                                    self.Finger_id.PINKY_TIP.value)
                    },
            "swipe": {
                        "function": self.swipe.get_hand_move,
                        "detect_function": self.swipe.get_hand_move,
                        "fingers": (self.Finger_id.INDEX_TIP.value,
                                    self.Finger_id.MIDDLE_TIP.value,
                                    self.Finger_id.RING_TIP.value,
                                    self.Finger_id.PINKY_TIP.value)
                     },
            "close": {
                        "function": self.close_active_window,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("close"),
                        "fingers": (self.Finger_id.THUMB_TIP.value,
                                    self.Finger_id.INDEX_TIP.value,
                                    self.Finger_id.PINKY_TIP.value)
                     },
            "disp": {
                        "function": self.disp,
                        "detect_function": lambda lm_list, frame:
                                        self.motion_list.append("disp"),
                        "fingers": (self.Finger_id.MIDDLE_TIP.value,
                                    self.Finger_id.RING_TIP.value,
                                    self.Finger_id.PINKY_TIP.value)
                    }
            }
        self.fingers = []
        self.MOTION_LIST_MAXLEN = 15
        self.motion_list = collections.deque(maxlen=self.MOTION_LIST_MAXLEN)

    def run(self):
        while self.is_running:
            self.wait_unlock()
            self.frame = self.camera.detector.find_hands(self.frame)

            # List of finger states.
            lm_list = self.camera.detector.find_position(self.frame,
                                                         draw=False)
            if len(lm_list) != 0:
                self.check_motion(lm_list, self.frame)
                print("motion_list = ", self.motion_list)
                self.check_motion_list(lm_list, self.frame)
            self.lock = True

    def wait_unlock(self):
        """
            Waits for self.lock to be set to False.
        """
        while(self.lock):
            time.sleep(0.0005)

    def unlock(self, frame):
        """
            Allows the class to continue its execution.
        """
        self.frame = frame
        self.lock = False

    def check_motion(self, lm_list: list, frame):
        """
            Checking the execution of gestures.
            :param lm_list: list of finger states.
        """
        def check_fingers_state(fingers_state: tuple,
                                fingers_state_needed: tuple) -> bool:
            # The number of straight fingers should be the same.
            result = sum(fingers_state) == len(fingers_state_needed)
            for i in fingers_state_needed:
                result &= fingers_state[i]
            return result

        finger_counts = self.finger_count(lm_list)
        # For each action, check the position of the fingers and perform it if
        # the position turned out to be correct.
        for motion, item in self.motions.items():
            if check_fingers_state(self.fingers, item["fingers"]):
                item["detect_function"](lm_list, self.frame)
                return

    def check_motion_list(self, lm_list: list, frame):
        """
            Checking that the action was quite MOTION_LIST_MAXLEN times.
        """
        if len(self.motion_list) != self.MOTION_LIST_MAXLEN:
            return
        for motion, item in self.motions.items():
            if all(x == motion for x in self.motion_list):
                item["function"](lm_list, frame)
                self.motion_list.clear()
                return

    def get_fingers_state(self, lm_list: list) -> list:
        """
            :return: a list of which fingers are curled up. The order of the
            indices corresponds to the Fingers_id enumeration.
        """
        fingers = []

        if lm_list["thumb_tip"]["x"] < lm_list["thumb_ip"]["x"]:
            fingers.append(False)  # палец загнут
        else:
            fingers.append(True)  # палец не загнут

        for id in range(2, 6):
            if lm_list[LM_NAMES[id * 4]]["y"] > lm_list[LM_NAMES[id * 4 - 2]]["y"]:
                fingers.append(False)  # палец загнут
            else:
                fingers.append(True)  # палец не загнут
        return fingers

    def finger_count(self, lm_list: list) -> int:
        """
            :return: the number of fingers extended, or:
                     -1 if after expecting that the state of each of the
            fingers will remain the same, it has changed
        """
#        print("LmList", lm_list)
        fingers = self.get_fingers_state(lm_list)
        print("sticky fingers", fingers)
        """
            Checking if the state of the hand has changed (which fingers are
        bent) since the last time.
        """
        if self.fingers != fingers:
            """
                If the state of the hand has changed, then it is necessary to
            make sure that this is not just a wave of the hand - a dream is
            performed, after which it is checked whether the state of the hand
            has changed. In the case of a change or absence of a hand on the
            camera, -1 is returned.

                sleep_state is required so that the main thread can identify
            sleep by this thread, since the display on the webcam should not
            be interrupted.
            """
            self.fingers = fingers
            self.sleep_state = True
            time.sleep(0.6)

            self.frame = self.camera.detector.find_hands(self.frame)
            lm_list = self.camera.detector.find_position(self.frame, draw=False)
            self.sleep_state = False
            if len(lm_list) == 0:
                return -1
            fingers = self.get_fingers_state(lm_list)
            if self.fingers != fingers:
                return -1
        self.fingers = fingers
        return self.fingers.count(False)

    def lock_work_station(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
        print("Lock_work_station")
#        ctypes.windll.user32.LockWorkStation()

    def minimize_all_windows(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
        print("Minimize_all_windows")
#        pyautogui.hotkey('win', 'd')

    def switch_window(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
#pyautogui.hotkey('ctrl', 'tab')
        print("switch window")

    def back_window(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
        print("back window")
#        pyautogui.hotkey('alt', 'tab')

    def close_active_window(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
        print("close active window")
#        pyautogui.hotkey('alt', 'f4')

    def disp(self, lm_list: list, frame):
        """
            :param lm_list: list of finger states
            :param frame: webcam image
        """
        print("disp")
#        pyautogui.hotkey('ctrl', 'shift', 'escape')


class Processing():
    """
        The main class of the application. Contains the execution cycle of
    the program.
    """
    def __init__(self):
        self.camera = Camera(1920, 1080)  # размеры [x, y]
        self.volume = Volume(self.camera)
        self.swipe = Swipe()
        self.controlling = Controlling(self.camera, self.volume, self.swipe)
        self.controlling.start()

    def loop(self):
        """
            The main execution cycle of the program.
        """
        while True:
            success, frame = self.camera.cap.read()
            self.controlling.frame = frame
            if self.controlling.sleep_state == False:
                self.controlling.unlock(frame)

            cv2.putText(frame, f'FPS:{self.camera.fps}', (40, 50),
                        cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 0), 3)

            while ((self.controlling.lock == False) &
                   (self.controlling.sleep_state == False)):
                time.sleep(0.0005)

            cv2.imshow("frame", frame)
            if cv2.waitKey(1) == 27:
                self.controlling.is_running = False
                break
            time.sleep(1 / self.camera.fps)


processing = Processing()
processing.loop()
