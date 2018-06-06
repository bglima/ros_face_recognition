# -*- coding: utf-8 -*-
import face_recognition
import cv2
from time import time

class FaceDetector:
    def __init__(self):
        self.known_face_encodings = []   # Array of known face encodings
        self.known_face_names = []       # Array of known face names
        self.known_face_colors = []      # Array of colors in BGR format

        self.face_locations = []    # Array of locations of detected faces within image
        self.face_encodings = []    # Array of encodings of detected faces within image
        self.face_names = []        # Array of names of detected faces within image
        self.face_colors = []       # Array of colors of detected faces within image
        self.process_this_frame = True  # If current frame should be processed or not
        self.process_each_n = 1         # Number of frames withou processing after a processed one
        self.frame_index = -1           # Index of current frame processed since start of program
        self.scale_factor = 0.25    # Scale original image in order to increase processing speed
        self.last_frame = None

    def add_to_database(self, person_name, person_photo_path, person_color):
        person_image = face_recognition.load_image_file(person_photo_path)
        person_face_encoding = face_recognition.face_encodings(person_image)[0]
        self.known_face_encodings.append(person_face_encoding)
        self.known_face_names.append(person_name)
        self.known_face_colors.append(person_color)

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
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)

            self.face_names = []    # Clean data from previous frame
            self.face_colors = []
            for face_encoding in self.face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                name = "Unknown"
                color = (0, 0, 0)

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = self.known_face_names[first_match_index]
                    color = self.known_face_colors[first_match_index]

                self.face_names.append(name)
                self.face_colors.append(color)

            #print('Detection took {} seconds...'.format(round(end - start, 2)))

        # Upper limit of frame_index (in order to not overflow)
        self.frame_index = self.frame_index & self.process_each_n
        # Determine if next frame will be processed
        self.process_this_frame = not self.frame_index

    def get_results(self):
        # Init an array that will comprise: [NAME, BOUNDING_BOX, COLOR]
        results = []
        for i in range(len(self.face_names)):
            results.append( (self.face_names[i], self.face_locations[i], self.face_colors[i]) )
        return results

    def draw_results(self):
        frame = self.last_frame.copy()
        # Iterate over detected faces
        for result in self.get_results():
            (name, (top, right, bottom, left), color) = result

            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            scale = int(1.0 / self.scale_factor)
            top *= scale
            right *= scale
            bottom *= scale
            left *= scale

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), color, 2)

            # Draw a label with a name below the face
            font = cv2.FONT_HERSHEY_DUPLEX
            label_top = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(frame, name, (left, label_top), font, 1.0, (255, 255, 255), 1)
        return frame

# Use example for recognition_class
def main():
    detector = FaceDetector()
    detector.add_to_database("Bruno Lima", "../media/train/bruno_lima/bruno_05.png", (255, 0, 0))
    detector.add_to_database("Joao Victor", "../media/train/joao_victor/joao_01.jpg", (0, 0, 255))

    video_capture = cv2.VideoCapture('../media/video/bruno_e_joao.mp4')

    while True:
        # Grab a single frame of video
        ret, frame = video_capture.read()
        if frame is None:
            break

        detector.process_frame(frame)
        frame = detector.draw_results()

        # Display the resulting image
        cv2.imshow('Video', frame)

        # Hit 'q' on the keyboard to quit!
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
