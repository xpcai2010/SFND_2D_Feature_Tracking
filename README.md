# SFND 2D Feature Tracking

[//]: # (Image References)
[image0]: ./results/Images/Image0.png "MP.1"
[image1]: ./results/Images/Image1.png "MP.2"
[image2]: ./results/Images/Image2.png "MP.3"
[image3]: ./results/Images/Image3.png "MP.4"
[image4]: ./results/Images/Image4.png "MP.5"
[image5]: ./results/Images/Image5.png "MP.6"
[image6]: ./results/Images/Image6.png "MP.7"
[image7]: ./results/Images/Image7.png "MP.8"
[image8]: ./results/Images/Image8.png "MP.9a"
[image9]: ./results/Images/Image9.png "MP.9b"

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Writeup, Task MP.0
## Data Buffer
## MP.1 Data Buffer Optimization
I've implemented a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements) in <MidTermPorject_Camera_Student.cpp> Line 72 to line 85. This can be achieved by pushing in new elements on one end and removing elements on the other end. 

![alt text][image0]

## Keypoints
## MP.2 Keypoint Detection
I've implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly. You can find it in <MidTermPorject_Camera_Student.cpp> Line 88 to line 109. Please see the screen shot below

![alt text][image1]

## MP.3 Keypoint Removal
In <MidTermPorject_Camera_Student.cpp>, line 111 to line 127, all keypoints are removed outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing. 

![alt text][image2]

## Descriptors
## MP.4 Keypoint Descriptors
I've implemented descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly. You can find it in <MidTermPorject_Camera_Student.cpp>, line 175 to line 183.

![alt text][image3]

## MP.5 Descriptor Matching
I've implemented FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. 

- For descriptor type, select binary (BINARY) or histogram of gradients (HOG). BINARY descriptors include: BRISK, BRIEF, ORB, FREAK, and AKAZE. HOG descriptors include: SIFT. 
- For the matcherType, it's either "MAT_BF" or "MAT_FLANN" .
- For selectorType either "SEL_NN" (nearest neighbors) or "SEL_KNN" (k nearest neighbors)

![alt text][image4]

## MP.6 Descriptor Distance Ratio
I use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints. You can find it in matching2D_Student.cpp from line 51 to 69.

![alt text][image5]

## Performance
For the performance benchmarks (MP.7-9) below, matcherType is set to MAT_BF and selectorType was set to SEL_KNN, which implements match filtering based on the descriptor distance ratio.

## MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented. 
You can find the results in ./results/_**MP7 Performance Evaluation.csv**_ 
The top three performers in this metric were:
- BRISK
- FAST 
- SIFT

The graph below shows all the detectors that I've implenmented for all 10 images. The distribution of their neighborhood size is also calculated, including mean value and standard deviation from all detected keypoints. The summary is average of 10 images per detector.

![alt text][image6]

## MP.8 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
The total combinations of Detector type and Descriptor type are 35. **Please be aware that AKAZE descriptor works only with AKAZE detector and ORB descripotor doesn't work with SIFT detector.**

The top3 Detector type with Descriptor type that produce the largest matched points are 

- BRISK(detector) + BRIEF(descriptor)
- BRISK(detector) + SIFT(descriptor)
- FAST(detector) + SIFT(descriptor)

The graph below lists all the Detector and Descriptor types, sorted by _**the number of matched points**_. The summary is average of 10 images per detector + descriptor combination.

![alt text][image7]


## MP.9 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

The 3 fastest Detector + Descriptor types based on shortest time for keypoint detection and descriptor extraction are

- FAST(detector) + ORB(descriptor)
- FAST(detector) + BRIEF(descriptor)
- ORB(detector) + BRIEF(descriptor)

The graph below lists all the Detector and Descriptor types, sorted by _**detection and descriptor extraction time**_. The summary is average of 10 images per detector + descriptor combination.

![alt text][image8]

If we use a new metric - **ratio of matched points per time** to pick up the **top 3 Detector + Descriptor types**, the **winners** are

- **FAST(detector) + ORB(descriptor)**
- **FAST(detector) + BRIEF(descriptor)**
- **FAST(detector) + BRISK(descriptor)**

The graph below lists all the Detector and Descriptor types, sorted by _**ratio of matched points per time**_. The summary is average of 10 images per detector + descriptor combination.

![alt text][image9]

You can find summary results
 - _./results/MP7 Performance Evaluation.csv_ for task MP.7
 - _./results/MP8_MP9_Performance Evaluation.csv_ for task MP.8 and MP.9. 
 - _./results/Performance_Evaluation_byAveraging10Images.xlsx_ as metric summary based on average of 10 images for all 35 combinations of Detector and Descriptor types 