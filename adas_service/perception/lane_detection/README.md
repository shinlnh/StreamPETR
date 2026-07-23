# Lane Detector

### Introduction
This is the source code for ADAS Service's Lane detector module.
Currently, we are using a model based on LaneATT architecture.
All of model configuration is in `laneatt_utils.h`.

Currently, we have two trained weights for this module. One with no classification and one with classification. These weights are already available as `laneatt.engine` and `laneatt_cls.engine`. To change between them, simply edit `TRT_WEIGHT_PATH` constant in `laneatt_utils.h`.

If you need to rebuild the engine (tensorrt version changed or lost data):
 - Download the onnx weight from our adas/data gitlab repo.
 - Build engine using tensorrt.

Check out `script/ai_scripts/LaneATT/README.md` for instruction and if you want to train a new weight.