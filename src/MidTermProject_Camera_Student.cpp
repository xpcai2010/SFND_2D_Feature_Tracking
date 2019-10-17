/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"


using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // define output vector -start
    vector<int> num_detectedKeypoints;
    vector<float> mean_NeighborSize;
    vector<float> std_NeighborSize;
    vector<int> num_matchPoints;

    vector<float> t_detector;
    vector<float> t_descriptor;
    vector<float> t_match;


    // end
    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        if (dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
 
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "HARRIS"; // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        
        if (detectorType.compare("SHITOMASI") == 0)
        {
            t_detector.push_back(detKeypointsShiTomasi(keypoints, imgGray, false));
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            t_detector.push_back(detKeypointsHarris(keypoints, imgGray, false));
        }
        else
        {
            t_detector.push_back(detKeypointsModern(keypoints, imgGray, detectorType, false));
        }

            //...
            //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            for (auto it = keypoints.begin(); it < keypoints.end(); it++)
            {
                if (!vehicleRect.contains(it->pt))
                {
                    keypoints.erase(it);
                }
            }
        }

        //// EOF STUDENT ASSIGNMENT
    
        // Task MP.7 Counting keypoints
        
        cout <<"MP.7: " <<detectorType << ", detected keypoints: ";
        cout << keypoints.size() << ",  mean neighborhood size: ";
        float sum = 0.0, mean, variance = 0.0, stdDeviation;
        if (keypoints.size() > 0) {
            for (int i = 0; i < keypoints.size(); ++i)
            {
                sum += keypoints[i].size;
            }
            mean = sum / keypoints.size();

            for (int i = 0; i < keypoints.size(); ++i)
            {
                variance += pow(keypoints[i].size - mean, 2);
            }
            variance = variance / keypoints.size();
            stdDeviation = sqrt(variance);
            cout << mean << ", standard deviation of neighborhood size: ";
            cout << stdDeviation << endl;
        }
        num_detectedKeypoints.push_back(keypoints.size());
        mean_NeighborSize.push_back(mean);
        std_NeighborSize.push_back(stdDeviation);

        
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        t_descriptor.push_back(descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType));
       
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
 
            /*  For descriptor type, select binary (BINARY) or histogram of gradients (HOG) 
                BINARY descriptors include: BRISK, BRIEF, ORB, FREAK, and (A)KAZE. 
                HOG descriptors include: SIFT (and SURF and GLOH, all patented). */
            string descriptorType_HOT_or_BIN = "DES_BINARY"; // DES_BINARY, DES_HOG
            if (descriptorType.compare("SIFT") == 0)
            {
                descriptorType_HOT_or_BIN = "DES_HOG";
            }  

            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            t_match.push_back(matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_HOT_or_BIN, matcherType, selectorType));
            
            num_matchPoints.push_back(matches.size());

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    //output to txt for the summary MP.8 and MP.9
    // std::ofstream outFile("summary.csv");
    // the important part
    num_matchPoints.push_back(-1);
    std::rotate(num_matchPoints.begin(),num_matchPoints.begin()+ 9,num_matchPoints.end()); // match is between 2 images. so the first value is image2 vs image1
    for (int i = 0; i <= imgEndIndex - imgStartIndex; ++i)
    {
        // outFile << num_detectedKeypoints[i] <<","<< mean_NeighborSize[i] << ","<<std_NeighborSize[i] <<","<<num_matchPoints[i]<<"," 
        // << t_detector[i]<<"," <<t_descriptor[i]<<"," <<t_detector[i] + t_descriptor[i]<<"\n";
        cout << num_detectedKeypoints[i] <<","<< mean_NeighborSize[i] << ","<<std_NeighborSize[i] <<","<<num_matchPoints[i]<<"," 
        << t_detector[i]<<"," <<t_descriptor[i]<<"," <<t_detector[i] + t_descriptor[i]<<"\n";       
    }
    // oupt end
    return 0;
}
