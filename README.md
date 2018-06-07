## ros_face_recognition ROS Package

Final project for Computer Vision course at Federal University of Alagoas.

## Description

A ROS package that recognizes faces from video input. Through proper namespaces, allows multiples nodes to run indepently.

## Implementation Notes

In order to capture the video input, [video_stream_opencv](http://wiki.ros.org/video_stream_opencv) ROS Package was used. It allows you to used video from an external hardware (webcam) or from a stored video file.

For the recognition task, it uses the [Face Recognition](https://github.com/ageitgey/face_recognition) Python module, which is based on [Dlib](https://github.com/davisking/dlib) library.

All the recognition code is present in ```face_detector.py```. ROS related code is present in ```recognition_node.py```, which makes use of the former.

## How to Run It
You can start recognition node with following command:

```
roslaunch ros_face_recognition start.launch \
node_name:=recognition_node \
process_each_n:=3 \
camera_topic:=/camera/image_raw \
scale_factor:=0.25 \
hertz:=30
```

These are the default parameters. In the case you do not want to change one or more of them, just call

```
roslaunch ros_face_recognition start.launch
```

and it should start the same way.

In order to check which people a certain node is seeing, a service was implemented inside the node namespace. You can call it by running:

```
rosservice call /recognition_node/get_seen_faces_names
```

This service will return a [Trigger](http://docs.ros.org/kinetic/api/std_srvs/html/srv/Trigger.html) message response. If nobody is seen at the moment, it returns False and a no face names. Otherwise, it returns True and the names of seen people. If a person is unkown, it will return Unknown in the name field.


### Final Considerations

Adding new people to the known dataset is currently execute via hardcode, at the ```setup_detection()``` function from ```recognition_node.py```. In the future, this will be done via service.

### References

https://github.com/ageitgey/face_recognition

https://github.com/ros-drivers/video_stream_opencv

http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
