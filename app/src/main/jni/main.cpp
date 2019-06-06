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
vector<KeyPoint> kp_prev;
Mat desc_prev;
int frame_count = 0;

JNIEXPORT void JNICALL Java_com_example_testapp_MainActivity_ConvertRGBtoGray
  (JNIEnv *env, jobject instance, jlong matAddrInput, jlong matAddrResult) {
    Mat &currFrame = *(Mat *)matAddrInput;
    Mat &dispFrame = *(Mat *)matAddrResult;
    Mat diffFrame, grayFrame;

    cvtColor(currFrame, grayFrame, COLOR_RGBA2GRAY);

    vector<KeyPoint> kp_curr;
    Ptr<ORB> orbF = ORB::create(100);
    Mat desc_curr;

    /*
     * Makes a keypoint & descriptor for the given current image
     */
    orbF->detectAndCompute(grayFrame, noArray(), kp_curr, desc_curr);

    /*
     * Matching descriptor vector between current and previous image
     */
    if( frame_count != 0 ) {
        vector<DMatch> matches;
        BFMatcher matcher(NORM_HAMMING);
        matcher.match(desc_curr, desc_prev, matches);

        /*
         * Find good matches such that match distance is over 4 * minimum distance.
         */
        double min_dist, max_dist;
        min_dist = matches[0].distance;

        // Find max/min distance
        for (int i = 0; i < matches.size(); i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
        }

        vector<DMatch> goodmatches;
        double dist_thr = min_dist * 2;
        for (int i = 0; i < matches.size(); i++) {
            if (matches[i].distance < dist_thr)
                goodmatches.push_back(matches[i]);
        }

        /*
         * Draw features on the image
         */
#if 1
        if (kp_prev.size() > 4 || kp_curr.size() > 4) {
            for (int i = 0; i < goodmatches.size(); i++) {
                int kp1_idx, kp2_idx;
                kp1_idx = goodmatches[i].queryIdx;
                kp2_idx = goodmatches[i].trainIdx;
                cv::circle(currFrame, kp_curr[kp1_idx].pt, 5, cv::Scalar(255, 0, 0), 1, 8, 0);
                cv::circle(currFrame, kp_prev[kp2_idx].pt, 5, cv::Scalar(0, 0, 255), 1, 8, 0);
                cv::line(currFrame, kp_curr[kp1_idx].pt, kp_prev[kp2_idx].pt, cv::Scalar(0,255,0), 1, 8, 0);
            }
        }
#endif
    }
    else {
        frame_count = 1;
    }
#if 0
    drawKeypoints(currFrame, kp_curr, currFrame, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif

    dispFrame = currFrame;
    prevFrame = grayFrame;
    kp_prev = kp_curr;
    desc_prev = desc_curr;

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