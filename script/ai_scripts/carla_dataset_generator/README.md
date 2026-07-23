# Carla dataset Generator

### Installation
Install python packages:
```
python -m pip install -r requirements.txt
```

### Procedure
#### Quick start
Make sure you have opened Carla server.
Use this command to generate carla dataset:
```
bash scripts/generate_data.sh
```

#### Detailed steps
Edit configs in **config.py** includes image size, number of image to collect, etc.

Then run this command to collect raw data:
```
source scripts/init.sh
python fast_lane_detection.py
```
During this, you have some control over the path of the car and the speed.
Also you can shake the car by adjust pitch, raw, roll to improve diversity.
You can pause/resume recording by pressing P.
**For more information, read the on screen guide during using**

When finished, run this to generate images from raw data
```
python dataset_generator.py
```

(Optional) Save visualized image into a video
```
python image_to_video.py
```
