#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QString>
#include<QFileDialog>
#include"histogram1d.h"
#include "laplacian.h"
#include<vector>
#include"linefinder.h"
#include "harrisdetector.h"
#include<opencv2/features2d.hpp>
#include "cameracalibrator.h"
#include "opencv2/calib3d/calib3d.hpp"
#include<iostream>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


}

void MainWindow:: sharpen2D(const cv::Mat &image, cv::Mat &result) {
    // Construct kernel (all entries initialized to 0)
    cv::Mat kernel(3,3,CV_32F,cv::Scalar(0));
    // assigns kernel values
    kernel.at<float>(1,1)= 5.0;
    kernel.at<float>(0,1)= -1.0;
    kernel.at<float>(2,1)= -1.0;
    kernel.at<float>(1,0)= -1.0;
    kernel.at<float>(1,2)= -1.0;
    //filter the image
    cv::filter2D(image,result,image.depth(),kernel);
    }

void MainWindow:: displayGray(cv::Mat outImage){

    cv::resize(outImage,outImage,cvSize(ceil(ui->viewInputImage->width()),ceil(ui->viewInputImage->height())));
    ui->viewOutputImage->setPixmap(QPixmap::fromImage(QImage(outImage.data, outImage.cols, outImage.rows, outImage.step, QImage::Format_Indexed8)));

}

void MainWindow:: displayOutImage(cv::Mat outImage){

     cvtColor(outImage,outImage, CV_RGB2BGR);
    cv::resize(outImage,outImage,cvSize(ceil(ui->viewInputImage->width()),ceil(ui->viewInputImage->height())));
    ui->viewOutputImage->setPixmap(QPixmap::fromImage(QImage(outImage.data, outImage.cols, outImage.rows, outImage.step, QImage::Format_RGB888)));

}


void MainWindow::saltAndPepper (cv:: Mat &inputImage, int n){

    for (int k=0; k<n; k++) {
    // rand() is the MFC random number generator
    // try qrand() with Qt
    int i= rand()%inputImage.cols;
    int j= rand()%inputImage.rows;
    if (inputImage.channels() == 1) { // gray-level image
    inputImage.at<uchar>(j,i)= 255;
    } else if (inputImage.channels() == 3) { // color image
    inputImage.at<cv::Vec3b>(j,i)[0]= 255;
    inputImage.at<cv::Vec3b>(j,i)[1]= 255;
    inputImage.at<cv::Vec3b>(j,i)[2]= 255;
            }
        }
    displayOutImage(inputImage);
}

void MainWindow::addLogo(){



    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open Image"), ".",
    tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    cv::Mat logo= cv::imread(fileName.toStdString(),1);

    cv::cvtColor(logo, logo, CV_BGR2RGB);

    cv::resize(logo, logo, cvSize(70,70));



//    // define image ROI

    cv::Mat imageROI = processedImage(cv::Rect(processedImage.cols-70,processedImage.rows-70,logo.cols,logo.rows));
//    // add logo to image
    cv::addWeighted(imageROI,1.0,logo,0.3,0.,imageROI);

    imageROI.copyTo(processedImage(cv::Rect(processedImage.cols-70,processedImage.rows-70,imageROI.cols, imageROI.rows)));

    displayOutImage(processedImage);

 }

void MainWindow::drawCircles (cv::Mat inputImage){

    cv::GaussianBlur(inputImage,inputImage,cv::Size(5,5),1.5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(inputImage, circles, CV_HOUGH_GRADIENT,
    2, // accumulator resolution (size of the image / 2)
    50, // minimum distance between two circles
    200, // Canny high threshold
    100, // minimum number of votes
    25, 100); // min and max radius
    std::vector<cv::Vec3f>::
    const_iterator itc= circles.begin();
    while (itc!=circles.end()) {
    cv::circle(inputImage,
    cv::Point((*itc)[0], (*itc)[1]), // circle centre
    (*itc)[2], // circle radius
    cv::Scalar(0), // color
    2); // thickness
    ++itc;

    }
    cv::namedWindow("Detected Circles");
    cv::imshow("Detected Circles",inputImage);
    displayGray(inputImage);

}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_loadButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
    tr("Open Image"), ".",
    tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    image= cv::imread(fileName.toStdString(),1);
    processedImage = image;
    if(!fileName.isEmpty())
  {

    cv::cvtColor(image, image, CV_BGR2RGB);
    //resize the image to fit window size

    cv::resize(image,image,cvSize(ceil(ui->viewInputImage->width()),ceil(ui->viewInputImage->height())));
            //cv::cvtColor(image, image, CV_BGR2GRAY);
    ui->viewInputImage->setPixmap(QPixmap::fromImage(QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888)));
    ui->viewOutputImage->setPixmap(QPixmap::fromImage(QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888)));

    }

}

void MainWindow::on_applyButton_clicked()
{
        int index = ui->functionList->currentIndex();
        if (index == 1){
            saltAndPepper(processedImage, 1000);
        }

        if(index == 2){
             addLogo();
        }

        if(index == 4){
            Histogram1D h;
            cv::Mat t_processedImage;
            cvtColor(processedImage,t_processedImage, CV_BGR2GRAY);
            cv::namedWindow("Histogram");
            cv::imshow("Histogram",h.getHistogramImage(t_processedImage));
        }

        if (index==5){
            cv::Mat result;
            cvtColor(image,result, CV_BGR2GRAY);

            cv::equalizeHist(result,result);
            displayGray(result);
            cv::namedWindow("Equalized Image");
            cv::imshow("Equalized Image",result);
        }

        if (index ==6){
            cv::Mat eroded;
            cvtColor(image,eroded, CV_BGR2GRAY);

            cv::erode(eroded,eroded,cv::Mat());
            displayGray(eroded);
            cv::namedWindow("Eroded Image");
            cv::imshow("Eroded Image",eroded);

        }

        if (index ==7){
            cv::Mat dilated;
            cvtColor(image,dilated, CV_BGR2GRAY);

            cv::dilate(dilated,dilated,cv::Mat());
            displayGray(dilated);
            cv::namedWindow("Dilated Image");
            cv::imshow("Dilated Image",dilated);

        }

        if (index ==8){
            cv::Mat closed;
            cv::Mat element5(3,3,CV_8U,cv::Scalar(1));
            cvtColor(image,closed, CV_BGR2GRAY);
            cv::morphologyEx(closed,closed,cv::MORPH_CLOSE,element5);
            displayGray(closed);
            cv::namedWindow("Closed Image");
            cv::imshow("Closed Image",closed);

        }

        if (index ==9){
            cv::Mat opened;
            cv::Mat element5(3,3,CV_8U,cv::Scalar(1));
            cvtColor(image,opened, CV_BGR2GRAY);
            cv::morphologyEx(opened,opened,cv::MORPH_CLOSE,element5);
            displayGray(opened);
            cv::namedWindow("Opened Image");
            cv::imshow("Opened Image",opened);

        }

        if (index ==10){
            cv::Mat result;
            cvtColor(image,result, CV_BGR2GRAY);
            cv::blur(result,result,cv::Size(5,5));

            displayGray(result);
            cv::namedWindow("Blurred Image");
            cv::imshow("Blurred Image",result);

        }

        if (index ==11){
            cv::Mat result;
            cvtColor(image,result, CV_BGR2GRAY);
            cv::Sobel(result,result,CV_8U,1,0,3,0.4,128);
//            cv::Sobel(result,sobelY,CV_8U,1,0,3,0.4,128);

            displayGray(result);
            cv::namedWindow("Sobel Image");
            cv::imshow("Sobel Image",result);
        }

        if (index ==12){
            cv::Mat laplace;
            cvtColor(image,image, CV_BGR2GRAY);
            laplacian laplacian;
            laplacian.setAperture(7);
            cv::Mat flap= laplacian.computeLaplacian(image);
            laplace= laplacian.getLaplacianImage();

            displayGray(laplace);
            cv::namedWindow("Laplacian Image");
            cv::imshow("Laplacian Image",laplace);
        }

        if (index == 13)
        {
            cvtColor(image,image, CV_BGR2GRAY);
            sharpen2D(image, image);
            displayGray(image);
            cv::namedWindow("Sharpened Image");
            cv::imshow("Sharpened Image",image);
        }

        if (index == 14)
        {
            cv::Mat contours;
            cvtColor(image,contours, CV_BGR2GRAY);
            cv::Canny(contours, // gray-level image
            contours, // output contours
            50, // low threshold
            200); // high threshold

            displayGray(contours);
            cv::namedWindow("Canny Image");
            cv::imshow("Canny Image",contours);
        }

        if(index == 15)
        {
            cvtColor(image,image, CV_BGR2GRAY);
            cv::Mat contours;
            cv::Canny(image,contours,125,350);
            // Create LineFinder instance
           LineFinder finder;
            // Set probabilistic Hough parameters
            finder.setLineLengthAndGap(100,20);
            finder.setMinVote(80);
            // Detect lines and draw them
            std::vector<cv::Vec4i> lines= finder.findLines(contours);
            finder.drawDetectedLines(image);
            cv::namedWindow("Detected Lines with HoughP");
            cv::imshow("Detected Lines with HoughP",image);
            displayGray(image);
        }

         if(index == 16){
              cvtColor(image,image, CV_BGR2GRAY);
             drawCircles(image);

         }

         if(index == 17){

             cvtColor(image,image, CV_BGR2GRAY);
             // Create Harris detector instance
             HarrisDetector harris;
             // Compute Harris values
             harris.detect(image);
             // Detect Harris corners
             std::vector<cv::Point> pts;
             harris.getCorners(pts,0.01);
             // Draw Harris corners
             harris.drawOnImage(image,pts);

             cv::namedWindow("Harris Corners");
             cv::imshow("Harris Corners",image);
             displayGray(image);

         }

         if (index == 18){

             cv::Mat binaryImage;
             cvtColor(image,image, CV_RGB2BGR);
             cvtColor(image,binaryImage, CV_BGR2GRAY);
             cv::threshold(binaryImage, binaryImage,100,255,1);

             std::vector<std::vector<cv::Point>> contours;
             cv::findContours(binaryImage,
             contours, // a vector of contours
             CV_RETR_EXTERNAL, // retrieve the external contours
             CV_CHAIN_APPROX_NONE); // all pixels of each contours

             // Draw black contours on a white image
//             cv::Mat result(image.size(),CV_8U,cv::Scalar(255));


             //remove conotours

             int cmin= 10; // minimum contour length
             int cmax= 10000; // maximum contour length
             std::vector<std::vector<cv::Point>>::
             const_iterator itc= contours.begin();
             while (itc!=contours.end()) {
             if (itc->size() < cmin || itc->size() > cmax)
             itc= contours.erase(itc);
             else
             ++itc;
             }

             cv::drawContours(image,contours,
             -1, // draw all contours
             cv::Scalar(0), // in black
             2); // with a thickness of 2

             cv::namedWindow("Contours");

             cv::imshow("Contours",image);
         }

         if (index == 19){

             cv::Mat binaryImage;
             cvtColor(image,image, CV_RGB2BGR);
             cvtColor(image,binaryImage, CV_BGR2GRAY);
             cv::threshold(binaryImage, binaryImage,100,255,1);

             std::vector<std::vector<cv::Point>> contours;
             cv::findContours(binaryImage,
             contours, // a vector of contours
             CV_RETR_EXTERNAL, // retrieve the external contours
             CV_CHAIN_APPROX_NONE); // all pixels of each contours

              //remove smalll contours
             int cmin= 100; // minimum contour length
             int cmax= 1000; // maximum contour length
             std::vector<std::vector<cv::Point>>::
             const_iterator itc= contours.begin();
             while (itc!=contours.end()) {
             if (itc->size() < cmin || itc->size() > cmax)
             itc= contours.erase(itc);
             else
             ++itc;
             }

             cv::Rect r0= cv::boundingRect(cv::Mat(contours[0]));
             cv::rectangle(image,r0,cv::Scalar(0),2);

             cv::namedWindow("Bounding box");
             cv::imshow("Bounding box",image);
             displayOutImage(image);
         }

         if (index ==20){

                 cv::Mat binaryImage;
                 cvtColor(image,image, CV_RGB2BGR);
                 cvtColor(image,binaryImage, CV_BGR2GRAY);
                 cv::threshold(binaryImage, binaryImage,100,255,1);

                 std::vector<std::vector<cv::Point>> contours;
                 cv::findContours(binaryImage,
                 contours, // a vector of contours
                 CV_RETR_EXTERNAL, // retrieve the external contours
                 CV_CHAIN_APPROX_NONE); // all pixels of each contours

                  //remove smalll contours
                 int cmin= 100; // minimum contour length
                 int cmax= 1000; // maximum contour length
                 std::vector<std::vector<cv::Point>>::
                 const_iterator itc= contours.begin();
                 while (itc!=contours.end()) {
                 if (itc->size() < cmin || itc->size() > cmax)
                 itc= contours.erase(itc);
                 else
                 ++itc;
                 }

                 // testing the enclosing circle
                 float radius;
                 cv::Point2f center;
                 cv::minEnclosingCircle(cv::Mat(contours[1]),center,radius);
                 cv::circle(image,cv::Point(center),
                 static_cast<int>(radius),cv::Scalar(0),2);

                 cv::namedWindow("Enclosing circle");
                 cv::imshow("Enclosing circle",image);
                 displayOutImage(image);
             }

            if(index ==21){

                cvtColor(image,image, CV_RGB2GRAY);
                // vector of keypoints
                std::vector<cv::KeyPoint> keypoints;
                // Construction of the Fast feature detector object
                cv::FAST(image, keypoints, 40); // threshold for detection

                cv::drawKeypoints(image, // original image
                keypoints, // vector of keypoints
                image, // the output image
                cv::Scalar(255,255,255), // keypoint color
                cv::DrawMatchesFlags::DRAW_OVER_OUTIMG); //drawing flag

                cv::namedWindow("FAST features");
                cv::imshow("FAST features",image);
                displayGray(image);

            }

            if(index == 22){

                // Create Camera Calibration detector instance
                CameraCalibrator camcalibrator;
                // Compute Harris values
                // output vectors of image points
                std::vector<cv::Point2f> imageCorners;
                // number of corners on the chessboard
                cv::Size boardSize(13,14);
                cv::Mat imageExample;
                std::vector<std::string> filelist;
                int successes;
                int imgCounter=0;
                double calibrationError;
                // Get the chessboard corners from all images:
//
                QStringList filename = QFileDialog::getOpenFileNames(this,
                tr("Open Image"), ".",
                tr("Image Files (*.png *.jpg *.jpeg *.bmp *.tif)"));

                if (!filename.isEmpty()){
                    foreach( QString str, filename) {
                      filelist.push_back(str.toStdString());
                    }
                    imageExample = cv::imread(filelist[0],0);
                    successes=camcalibrator.addChessboardPoints(filelist,boardSize);
                    cv::Size imageSize(imageExample.cols,imageExample.rows);
                    calibrationError=camcalibrator.calibrate(imageSize);
                    std::stringstream ss;

                    foreach( QString str, filename) {
                        cv::Mat imageChess = cv::imread(str.toStdString(),0);
                        //Draw the corners
                        cv::drawChessboardCorners(imageChess,
                        boardSize, camcalibrator.imagePoints[imgCounter],
                        true); // corners have been found
                        ss<<"Chessboard points "<<imgCounter;
                        cv::namedWindow(ss.str());
                        cv::imshow(ss.str(),imageChess);
                        imgCounter++;
                    }

                }
                QString text;
                for (int i=0 ; i < camcalibrator.cameraMatrix.rows; i++)
                {
                    for (int j=0 ; j< camcalibrator.cameraMatrix.cols;j++)
                    {
                        text += QString::number(camcalibrator.cameraMatrix.at<double>(i,j)) + " ";
                    }
                    text += "\n";
                }
//                std::cout<< text;
               }




}
