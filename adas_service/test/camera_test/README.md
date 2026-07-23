# OpenCV Camera Capture

Simple test to check for camera function. Prints out OpenCV debugging information as well.

# Manual Debug

For CLI debugging tools, please use either
```
nvgstcapture-1.0
```
or
```
gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=960, height=540, framerate=30/1' ! nvvidconv ! 'video/x-raw, format=BGRx' ! videoconvert ! xvimagesink
```
