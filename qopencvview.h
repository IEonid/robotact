#ifndef QOPENCVVIEW_H
#define QOPENCVVIEW_H

#include <QWidget>
#include "cv.h"
#include "highgui.h"
#include <QImage>
#include <QPaintEvent>
#include <QPainter>

class QopencvView : public QWidget
{
    Q_OBJECT
public:
    explicit QopencvView(QWidget *parent = 0);
    void OpencvPaint(IplImage*);
    ~QopencvView();

protected:
    void paintEvent(QPaintEvent *e);

private:
    QImage *image,*qImage;
    IplImage *cvImage;
};

#endif // QOPENCVVIEW_H
