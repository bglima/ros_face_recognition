#! /usr/bin/python

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
        self.scale_to_process = 0.25    # Scale original image in order to increase processing speed

    def add_to_database(self, person_name, person_photo_path):
        person_image = face_recognition.load_image_file(person_photo_path)
        person_face_encoding = face_recognition.face_encodings(person_image)[0]
        self.known_face_names.append(person_name)
        self.known_face_encodings.append[person_face_encoding]

    def process_frame(self, image):
        frame_index += 1
