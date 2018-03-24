#ifndef LAPLACIAN_H
#define LAPLACIAN_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class laplacian
{
private:
private:
    // original image
    cv::Mat img;
    // 32-bit float image containing the Laplacian
    cv::Mat laplace;
    // Aperture size of the laplacian kernel
    int aperture;

public:
//   laplacian();

    laplacian() : aperture(3) {}
    // Set the aperture size of the kernel
    void setAperture(int a);
    cv::Mat computeLaplacian(const cv::Mat& image);
    cv::Mat getLaplacianImage(double scale=-1.0);

};

#endif // LAPLACIAN_H
