# ATracker - Multi Target Tracker

[![ATRACKER](https://img.youtube.com/vi/dP3vMgodW80/0.jpg)](https://www.youtube.com/watch?v=dP3vMgodW80)

ATracker is a multi-target tracker based on cost computation. The costs are computed according to the Mahalanobis distance. Then, a Linear Assignment approach together with a Munkres algorithm have been used for assigning the current detections to the previous tracks.
ATracker is able to manage the groups according to the appearance of the people as well as the costs based on the Mahalanobis distance.

## Requirements
* OpenCV

## How to build

ATracker works under Linux environments. I recommend a so-called out of source build which can be achieved by the following command sequence:

* mkdir build
* cd build
* cmake ../
* make -j<number-of-cores+1>

## How to use

Go to the bin diretory and launch the program with the following command:
```bash
./atracker /path/to/the/detection_file.txt /path/to/the/image_folder ../config/kalman_param.txt
```

