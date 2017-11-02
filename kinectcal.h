#ifndef KINECTCAL_H
#define KINECTCAL_H

#include <XnCppWrapper.h>
#include <XnModuleCppInterface.h>
#include <QVector3D>
#include <QMatrix4x4>
#include <QtMath>
#include "cv.h"
#include "highgui.h"
#include <QDebug>
#include <QLineEdit>
#include <QThread>
#include "qopencvview.h"

using namespace xn;
using namespace cv;

class kinectCal: public QThread
{
public:
    kinectCal(QopencvView *view);
    ~kinectCal();
    IplImage* collect();
    void initial();

    DepthMetaData depthMetadata;
    ImageMetaData imageMetadata;
    QLineEdit *datadisplay;
    QLineEdit *dataDebug;
    bool kinectStatus;
    bool kinectExist;
    bool kinectSave;
    QTextStream *file;

protected:
    void run();

private:
    void generaFrameAndVector(void);
    void angleDecode(void);
    void getGravity(void);

    static void XN_CALLBACK_TYPE NewUser( xn::UserGenerator& generator, XnUserID user,void* pCookie );
    static void XN_CALLBACK_TYPE LostUser( xn::UserGenerator& generator, XnUserID user,void* pCookie );
    static void XN_CALLBACK_TYPE CalibrationStart( xn::SkeletonCapability& skeleton,XnUserID user,void* pCookie );
    static void XN_CALLBACK_TYPE CalibrationEnd( xn::SkeletonCapability& skeleton,XnUserID user,XnCalibrationStatus calibrationError,void* pCookie );
    static void XN_CALLBACK_TYPE PoseDetected( xn::PoseDetectionCapability& poseDetection,const XnChar* strPose,XnUserID user,void* pCookie);   

    const int startSkelPoints[14]={0,1,5,5,11,16,5,6,11,12,16,17,20,21};
    const int endSkelPoints[14]={1,2,11,20,16,20,6,8,12,14,17,19,21,23};
    const int SkelPoints[15]={0,1,2,5,6,8,11,12,14,16,17,19,20,21,23};
    const int minAngle[21]={0,-180,-180,   0,-180,   0,   0, -30, -30, -30, -60, -20, -20, -90, -90, -60,- 60, -30, -30, -90, -45};
    const int maxAngle[21]={0, 180, 180, 180,   0, 135, 135,  30,  30,  60,  30,  90,  90,   0,   0,  90,  90,  30,  30,  90,  10};
    float  Theta[21];
    const float     Gm[21]={975.6f,  25.9f,  25.9f,
                            168.4f,  168.4f, 59.3f,
                            59.3f,   27.1f,  27.1f,
                            167.1f,  167.1f, 119.0f,
                            119.0f,  70.3f,  70.3f,
                            167.1f,  167.1f, 79.4f,
                            79.4f,   24.4f,  158.0f};
    const QVector3D Pg[21]={{3.11f,-39.44f,19.66f},  {2.48f,-5.74f,-1.39f},  {2.48f,-5.74f,-1.39f},
                            {-0.66f,-36.24f,0.73f},  {0.66f,-36.24f,0.73f},  {-6.67f,-45.84f,-13.49f},
                            {6.67f,-45.84f,-13.49f}, {0,-3.76f,0.48f},       {0,-3.76f,0.48f},
                            {-0.08f,-13.87f,-18.24f},{0.08f,-13.87f,-18.24f},{0.32f,-62.97f,0.69f},
                            {-0.32f,-62.97f,0.69f},  {-0.59f,-39.05f,6.55f}, {-0.59f,-39.05f,6.55f},
                            {0.21f,13.87f,-18.24f},  {-0.21f,13.87f,-18.24f},{-9.51f,-26.0f,-0.5f},
                            {9.51f,-26.0f,-0.5f},    {1.42f,9.4f,-0.71f},    {0.06f,18.56f,-7.66f}};
    const QVector3D Po[21]={{0,0,0},          {66.0f,0,0},           {-66.0f,0,0},
                            {16.0f,-16.0f,0}, {-16.0f,-16.0f,0},     {0,-60.0f,-16.0f},
                            {0,-60.0f,-16.0f},{37.0f,-100.0f,5.0f},  {-37.0f,-100.0f,5.0f},
                            {0,-22.2f,0},     {0,-22.2f,0},          {0,0,0},
                            {0,0,0},          {0,-93.0f,0},          {0,-93.0f,0},
                            {0,-93.0f,0},     {0,-93.0f,0},          {0,0,0},
                            {0,0,0},          {0,24.5f,0},           {0,26.0f,0}};

    QopencvView *opencvView;
    IplImage*   cameraImg;
    XnStatus    status;
    Context     context;
    DepthGenerator  depthGenerator;
    ImageGenerator  imageGenerator;
    UserGenerator   userGenerator;
    XnMapOutputMode mapMode;
    SkeletonCapability *skeletonCap;
    XnCallbackHandle userCBHandle;
    XnCallbackHandle calibCBHandle;
    XnCallbackHandle poseCBHandle;
    QVector3D Joint[15];
    QVector3D Vect[12];
};

#endif // KINECTCAL_H
