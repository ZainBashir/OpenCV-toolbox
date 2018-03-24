#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <QFileDialog>

#include <QMainWindow>
//#include <QtGui/QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void displayOutImage( cv:: Mat outImage);
    void saltAndPepper (cv:: Mat &inputImage, int n);
     void addLogo();


     void displayGray(cv::Mat outImage);
     void sharpen2D(const cv::Mat &image, cv::Mat &result);

     void drawCircles(cv::Mat inputImage);
private slots:
    void on_loadButton_clicked();
    void on_applyButton_clicked();

private:
    Ui::MainWindow *ui;
    cv::Mat image; // the image variable

    cv::Mat processedImage;


};

#endif // MAINWINDOW_H
