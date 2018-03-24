#ifndef LINEFINDER_H
#define LINEFINDER_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<math.h>

class LineFinder
{

private:

    // original image
    cv::Mat img;
    // vector containing the end points
    // of the detected lines
    std::vector<cv::Vec4i> lines;
    // accumulator resolution parameters
    double deltaRho;
    double deltaTheta;
    // minimum number of votes that a line
    // must receive before being considered
    int minVote;
    // min length for a line
    double minLength;
    // max allowed gap along the line
    double maxGap;

public:

    LineFinder(): deltaRho(1), deltaTheta(M_PI/180),
        minVote(10), minLength(0.), maxGap(0.) {}

    void setAccResolution(double dRho, double dTheta);
    void setMinVote(int minv);
    void setLineLengthAndGap(double length, double gap);
    std::vector<cv::Vec4i> findLines(cv::Mat& binary);
    void drawDetectedLines(cv::Mat &image,cv::Scalar color=cv::Scalar(255,255,255));
};

#endif // LINEFINDER_H
