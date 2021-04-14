#include "QtBallPlate.h"

QtBallPlate::QtBallPlate(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}

int QtBallPlate::Array2Mat(BYTE* array, cv::Mat& img, int row, int col)
{
    img = cv::Mat(row, col, CV_8UC1, (BYTE*)array);
    return 0;
}
