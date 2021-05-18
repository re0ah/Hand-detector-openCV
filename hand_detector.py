import cv2
import mediapipe as mp
import enum


class Hand_detector():
    """
        Class for detecting the position of the hands and each phalanx (list below).
    """
    class Landsmarks(enum.Enum):
        """
            Codes for which position the fingers took.
        """
        WRIST = 0
        THUMB_CMC = 1
        THUMB_MCP = 2
        THUMB_IP = 3
        THUMB_TIP = 4
        INDEX_FINGER_MCP = 5
        INDEX_FINGER_PIP = 6
        INDEX_FINGER_DIP = 7
        INDEX_FINGER_TIP = 8
        MIDDLE_FINGER_MCP = 9
        MIDDLE_FINGER_PIP = 10
        MIDDLE_FINGER_DIP = 11
        MIDDLE_FINGER_TIP = 12
        RING_FINGER_MCP = 13
        RING_FINGER_PIP = 14
        RING_FINGER_DIP = 15
        RING_FINGER_TIP = 16
        PINKY_MCP = 17
        PINKY_PIP = 18
        PINKY_DIP = 19
        PINKY_TIP = 20
    LM_NAMES = ("wrist",
                "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
                "index_mcp", "index_pip", "index_dip", "index_tip",
                "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
                "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
                "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip")

    def __init__(self,
                 mode=False,
                 max_hands=1,
                 detection_con=0.5,
                 track_con=0.5):
        self.mode = mode
        self.max_hands = max_hands
        self.detection_con = detection_con
        self.track_con = track_con

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(self.mode, self.max_hands,
                                         self.detection_con, self.track_con)
        self.mp_draw = mp.solutions.drawing_utils

    def find_hands(self, img, draw=True):
        img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_RGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(img, hand_lms,
                                                self.mp_hands.HAND_CONNECTIONS)
        return img

    def find_position(self, img, hand_no=0, draw=True) -> dict:
        self.lm_list = {}
        i = 0
        if self.results.multi_hand_landmarks:
            my_hand = self.results.multi_hand_landmarks[hand_no]
            for id, lm in enumerate(my_hand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                # print(id, cx, cy)
                self.lm_list[self.LM_NAMES[i]] = {"id": id, "x": cx, "y": cy}
                if draw:
                    cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
                i += 1
        return self.lm_list
