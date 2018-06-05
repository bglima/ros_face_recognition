# -*- coding: utf-8 -*-

import face_recognition
import cv2
from time import time

class FaceDetector:
    def __init__(self):
        self.known_face_encodings = [] # Array of known face encodings
        self.known_face_names = []  # Array of known face names

        self.face_locations = []    # Array of locations of detected faces within image
        self.face_encodings = []    # Array of encodings of detected faces within image
        self.face_names = []        # Array of names of detected faces within image
        self.process_this_frame = True  # If current frame should be processed or not
        self.process_each_n = 3         # Number of frames withou processing after a processed one
        self.frame_index = -1           # Index of current frame processed since start of program
        self.scale_factor = 0.25    # Scale original image in order to increase processing speed
        self.last_frame = None

    def add_to_database(self, person_name, person_photo_path):
        person_image = face_recognition.load_image_file(person_photo_path)
        person_face_encoding = face_recognition.face_encodings(person_image)[0]
        self.known_face_encodings.append(person_face_encoding)
        self.known_face_names.append(person_name)

    def process_frame(self, frame):
        # update frame_index
        self.frame_index += 1

        self.last_frame = frame.copy()
        # Resize image to a factor of SCALE_FACTOR
        small_frame = cv2.resize(frame, (0, 0), fx=self.scale_factor, fy=self.scale_factor)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process specific frames of video to save time
        if self.process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            start = time()
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)

            self.face_names = []
            for face_encoding in self.face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = self.known_face_names[first_match_index]

                self.face_names.append(name)

            end = time()
            print('Detection took {} seconds...'.format(round(end - start, 2)))

        # Upper limit of frame_index (in order to not overflow)
        self.frame_index = self.frame_index % self.process_each_n
        # Determine if next frame will be processed
        self.process_this_frame = not self.frame_index


def main():
    detector = FaceDetector()
    detector.add_to_database("Bruno Lima", "../media/train/bruno_lima/bruno_05.png")
    detector.add_to_database("João Victor", "../media/train/joao_victor/joao_01.jpg")

    video_capture = cv2.VideoCapture('../media/video/bruno_e_joao.mp4')

    while True:
        # Grab a single frame of video
        ret, frame = video_capture.read()
        if frame is None:
            break

        detector.process_frame(frame)

        # Display the resulting image
        cv2.imshow('Video', frame)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
