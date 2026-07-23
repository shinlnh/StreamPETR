# Communications Extensions

Welcome to the Communications repository, in this folder there are classes, helper functions and example codes of GStreamer video streaming (both sending and receiving) using different methods (TCP, UDP, RTSP, ...)

## Debugging Tips
Before we start, it must be noted that GStreamer requires extensive tinkering with different pipelines, there might be some hidden rules that we must deal with regarding resolutions and bitrates, sometimes even the official examples from NVIDIA and RidgeRun don't work. Sometimes pipelines that work on one computer might not work on another, whether it is because you used not-installed pipeline components or unsupported image format. With that said, here are some useful links you can take a look for example pipelines and example codes.

[Example GStreamer pipelines](https://gist.github.com/hum4n0id/cda96fb07a34300cdb2c0e314c14df0a)

[Official NVIDIA examples](https://forums.developer.nvidia.com/t/jetson-nano-faq/82953#q-is-there-any-example-of-running-rtsp-streaming-8)

[Official RidgeRun examples](https://developer.ridgerun.com/wiki/index.php/Jetson_Nano/Gstreamer/Example_Pipelines/Capture_Display#)

### Tip: Export the Gstreamer debug flag for more information
```
export GST_DEBUG=[LEVEL]
```
You can set the GST_DEBUG flag to get more information about the pipeline, this is helpful when trying to find bugs or pipeline errors. I find LEVEL in the range of 3 to 5 the most useful.

### Tip: Check for installed components / elements
You can check for installed components using this command.
```
gst-inspect-1.0 --plugin
```
You can use this in combination with grep to find specific components. For example:
```
gst-inspect-1.0 --plugin | grep h264
```

## My test pipelines (All streaming cameras)
### UDP

Sending
```
 gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=1920, height=1080, framerate=30/1, format=NV12' ! omxh264enc insert-sps-pps=true bitrate=16000000 ! rtph264pay ! udpsink port=5000 host=192.168.240.100
```
Receiving
```
 gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

```

### RTSP
Sending
```
 ./test-launch "nvarguscamerasrc ! video/x-raw(memory:NVMM), format=NV12, width=1920, height=1080, framerate=30/1 ! nvvidconv ! video/x-raw, width=640, height=480, format=NV12, framerate=30/1 ! omxh265enc ! rtph265pay name=pay0 pt=96 config-interval=1"
```
Receiving
```
gst-launch-1.0 rtspsrc latency = 0 location=rtsp://192.168.240.110:8554/test ! decodebin ! videoconvert ! autovideosink

```

**Note**: What is test-launch? It's the executable that you get after you have compiled the source code [here](https://github.com/GStreamer/gst-rtsp-server/tree/1.16). Or you can use my compiled code at 
```
/home/share/minh.nguyen-xuan/gst-rtsp-server/build/examples
```

## Usage
You can include the header files of the codes and use them like a black box to push images in (Sender) and extract images out (Receiver). This is done through an interface:

* Sender: Use the function **updateFrame(const cv::Mat newFrame)** to push the image into the pipeline.
* Receiver: Use the function **getNextFrame()** to grab an image from the pipeline. The image is in the format cv::Mat.

## Example codes
You can check out the Examples folder for how you can integrate the classes into your code. Inside there are 4 folders along with the Cmake file. Simply compile using: 
```
cmake.
```
**Note**: In the UDP folders, send1, receive1 should be compiled on PC. send2, receive2 should be cross-compiled.

**Note**: In the RTSP folders all files should be cross-compiled. (PC compilation is not tested.)

**Note**: You MUST take care to set the IP correctly in the pipeline. Remember that in UDP we pick an IP to send to, but in RTSP we pick an IP to receive from.

