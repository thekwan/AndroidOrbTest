//
// Created by HwanYoun on 2019-05-09.
//
#include <jni.h>
#include "com_example_testapp_MainActivity.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

extern "C" {
/*
 * Class:     com_example_testapp_MainActivity
 * Method:    ConvertRGBtoGray
 * Signature: (JJ)V
 */
Mat prevFrame;
int frame_count = 0;

JNIEXPORT void JNICALL Java_com_example_testapp_MainActivity_ConvertRGBtoGray
  (JNIEnv *env, jobject instance, jlong matAddrInput, jlong matAddrResult) {
    Mat &currFrame = *(Mat *)matAddrInput;
    Mat &dispFrame = *(Mat *)matAddrResult;
    Mat diffFrame, grayFrame;

    cvtColor(currFrame, grayFrame, COLOR_RGBA2GRAY);

    vector<KeyPoint> kp1;
    Ptr<ORB> orbF = ORB::create(500);
    Mat desc1;

    // Feature extraction
    orbF->detectAndCompute(grayFrame, noArray(), kp1, desc1);

    // Draw features on image
    drawKeypoints(currFrame, kp1, currFrame, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    dispFrame = currFrame;
    prevFrame = grayFrame;

#if 0
    if( frame_count != 0 ) {
        absdiff(prevFrame, currFrame, diffFrame);
        threshold(diffFrame, diffFrame, 80, 255, cv::THRESH_BINARY);
        erode(diffFrame, diffFrame, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
        dispFrame = diffFrame;
    }
    else {
        frame_count = 1;
        dispFrame = currFrame;
    }
    prevFrame = currFrame;
#endif

  }
}