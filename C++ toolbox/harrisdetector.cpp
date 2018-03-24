#include "harrisdetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<vector>


HarrisDetector::HarrisDetector(): neighbourhood(3), aperture(3),
k(0.01), maxStrength(0.0),
threshold(0.01), nonMaxSize(3) {
setLocalMaxWindowSize(nonMaxSize);
}

void HarrisDetector::setLocalMaxWindowSize(int size) {
    nonMaxSize = size;
    kernel.create(nonMaxSize, nonMaxSize, CV_8U);
}


void HarrisDetector::detect(const cv::Mat &image)
{
    // Harris computation
    cv::cornerHarris(image,cornerStrength,
    neighbourhood,// neighborhood size
    aperture, // aperture size
    k); // Harris parameter
    // internal threshold computation
    double minStrength; // not used
    cv::minMaxLoc(cornerStrength,
    &minStrength,&maxStrength);
    // local maxima detection
    cv::Mat dilated; // temporary image
    cv::dilate(cornerStrength,dilated,cv::Mat());
    cv::compare(cornerStrength,dilated,
                localMax,cv::CMP_EQ);
}

cv::Mat HarrisDetector::getCornerMap(double qualityLevel)
{
    cv::Mat cornerMap;
    // thresholding the corner strength
    threshold= qualityLevel*maxStrength;
    cv::threshold(cornerStrength,cornerTh,
    threshold,255,cv::THRESH_BINARY);
    // convert to 8-bit image
    cornerTh.convertTo(cornerMap,CV_8U);
    // non-maxima suppression
    cv::bitwise_and(cornerMap,localMax,cornerMap);
    return cornerMap;

}

void HarrisDetector::getCorners(std::vector<cv::Point> &points, double qualityLevel)
{
    // Get the corner map
    cv::Mat cornerMap= getCornerMap(qualityLevel);
    // Get the corners
    getCorners(points, cornerMap);
}

void HarrisDetector::getCorners(std::vector<cv::Point> &points, const cv::Mat &cornerMap)
{
    // Iterate over the pixels to obtain all features
    for( int y = 0; y < cornerMap.rows; y++ ) {
    const uchar* rowPtr = cornerMap.ptr<uchar>(y);
    for( int x = 0; x < cornerMap.cols; x++ ) {
    // if it is a feature point
    if (rowPtr[x]) {
        points.push_back(cv::Point(x,y));
        }
        }
        }

}

void HarrisDetector::drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points, cv::Scalar color, int radius, int thickness)
{
    std::vector<cv::Point>::const_iterator it=
    points.begin();
    // for all corners
    while (it!=points.end()) {
    // draw a circle at each corner location
    cv::circle(image,*it,radius,color,thickness);
    ++it;
    }
}


