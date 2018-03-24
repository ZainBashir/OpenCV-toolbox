#ifndef HARRISDETECTOR_H
#define HARRISDETECTOR_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class HarrisDetector
{

private:
    // 32-bit float image of corner strength
    cv::Mat cornerStrength;
    // 32-bit float image of thresholded corners
    cv::Mat cornerTh;
    // image of local maxima (internal)
    cv::Mat localMax;
    // size of neighborhood for derivatives smoothing
    int neighbourhood;
    // aperture for gradient computation
    int aperture;
    // Harris parameter
    double k;
    // maximum strength for threshold computation
    double maxStrength;
    // calculated threshold (internal)
    double threshold;
    // size of neighborhood for non-max suppression
    int nonMaxSize;
    // kernel for non-max suppression
    cv::Mat kernel;

public:
    HarrisDetector();

    void detect(const cv::Mat& image);
    cv::Mat getCornerMap(double qualityLevel);
    void getCorners(std::vector<cv::Point> &points, double qualityLevel);
    void getCorners(std::vector<cv::Point> &points, const cv::Mat& cornerMap);
    void drawOnImage(cv::Mat &image, const std::vector<cv::Point> &points,cv::Scalar color= cv::Scalar(255,255,255),int radius=3, int thickness=2);
    void setLocalMaxWindowSize(int size);
};

#endif // HARRISDETECTOR_H
