import cv2
import mediapipe as mp

class hands:
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    after_img = ""
    StartFlg = False
    rsp = ""
    FinishFlg = False

    def HandsLoop(self):
        cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        with self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:

            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue

                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                before_image = image.copy()

                results = hands.process(image)
                
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())

                self.after_img = cv2.flip(image - before_image, 1)

                if __name__ == '__main__':
                    cv2.imshow('MediaPipe Hands', self.after_img)
                    if cv2.waitKey(5) & 0xFF == 27:
                        break

        cap.release()

if __name__ == '__main__':
    m_hands = hands()
    m_hands.HandsLoop()
