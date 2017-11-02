#include "qopencvview.h"

QopencvView::QopencvView(QWidget *parent) : QWidget(parent)
{
    image = new QImage(geometry().width(),geometry().height(),QImage::Format_RGB888);
    this->show();
}

QopencvView::~QopencvView()
{
    delete image;
//    delete cvImage;
    delete qImage;
    cvReleaseImage(&cvImage);
}

void QopencvView::paintEvent(QPaintEvent *e)
{
    QPainter painter(this);
    painter.drawImage(QPoint((geometry().width()-image->width())/2,(geometry().height()-image->height())/2),*image);
}

void QopencvView::OpencvPaint(IplImage* img)
{
    static bool firstUse=true;
    if(firstUse==true)
    {
        firstUse=false;
        cvImage = cvCreateImage(cvSize(img->width,img->height),8,3);
        qImage = new QImage(img->width,img->height,QImage::Format_RGB888);
        cvImage->imageData = (char*)qImage->bits();
    }
    cvCopy(img,cvImage,0);
    cvCvtColor(cvImage,cvImage,CV_BGR2RGB);
    *image = qImage->scaled(geometry().width(),geometry().height(),Qt::KeepAspectRatio);
    update();
}
