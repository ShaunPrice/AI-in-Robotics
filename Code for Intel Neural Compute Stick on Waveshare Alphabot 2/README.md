# Code for the Intel Neural Compute Stick 2 with Transfer Learning using AWS SageMaker
## Overview
This code was used on the [Raspberry Pi 3b+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/) on a [Waveshare Alphabot2 robot](https://www.waveshare.com/wiki/AlphaBot2-Pi). It's runs with an [Intel (Movidius) Neural Compute Stick 2](https://software.intel.com/en-us/neural-compute-stick).

## Details on building the robot
The details of how to train the robot and use the code are in the presentation below:

[Intel Neural Compute Stick with Transfer Learning using AWS SageMaker](https://github.com/ShaunPrice/AI-in-Robotics/blob/master/Intel%20Neural%20Compute%20Stick%202%20with%20Transfer%20Learning%20using%20AWS%20SageMaker.pdf)

## Video
You can see teh robot running on my Youtube channel.

https://www.youtube.com/watch?v=QptokSX7YzY

## Notes:
1. The AlphaBot2.py and PCA9685.py files are from the AlphaBot2.
2. Inference.py runs the model by reading in JPEG (jpg) images from a subdirectory names **images**. The images must be sized to 224x224 pixels. 
3. robby-run.py is the program that run s the robot. The images are taken from the Raspberry Pi camera.
