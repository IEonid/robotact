#include "kinectcal.h"

#define SQR(x) (x)*(x)

float constraint(float input,float min,float max)
{
    if(input>max)return max;
    if(input<min)return min;
    return input;
}

kinectCal::kinectCal(QopencvView *view)
{
    kinectStatus=false;
    kinectExist=false;
    opencvView = view;
}

kinectCal::~kinectCal()
{
    if(kinectExist==false)return;
    delete skeletonCap;
    context.Release();
    cvReleaseImage(&cameraImg);
}

void kinectCal::run()
{
    initial();
    qDebug()<<"kinect thread start!";
    while(1)
    {
        if(kinectStatus)opencvView->OpencvPaint(collect());
        usleep(1000);
    }
}

void kinectCal::initial()
{
    qDebug()<<"kinect init ...";
    context.Init();
    mapMode.nXRes = 640;
    mapMode.nYRes = 480;
    mapMode.nFPS = 30;

    depthGenerator.Create( context );
    depthGenerator.SetMapOutputMode( mapMode );
    imageGenerator.Create( context );
    userGenerator.Create( context );
    userGenerator.RegisterUserCallbacks( NewUser, LostUser, NULL, userCBHandle );
    skeletonCap = new SkeletonCapability(userGenerator.GetSkeletonCap());
    skeletonCap->SetSkeletonProfile( XN_SKEL_PROFILE_ALL );
    skeletonCap->RegisterToCalibrationStart( CalibrationStart,&userGenerator, calibCBHandle );
    skeletonCap->RegisterToCalibrationComplete( CalibrationEnd,&userGenerator, calibCBHandle );
    userGenerator.GetPoseDetectionCap().RegisterToPoseDetected( PoseDetected,&userGenerator, poseCBHandle );
    context.StartGeneratingAll();
    cameraImg=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

    kinectExist=true;
    kinectSave=false;
    qDebug()<<"kinect init suceess!";
}

IplImage* kinectCal::collect()
{
    context.WaitAndUpdateAll();
    imageGenerator.GetMetaData(imageMetadata);
    memcpy(cameraImg->imageData,imageMetadata.Data(),640*480*3);
    cvCvtColor(cameraImg,cameraImg,CV_RGB2BGR);
    XnUInt16 userCounts = userGenerator.GetNumberOfUsers();
    if( userCounts > 0 )
    {
        XnUserID* userID = new XnUserID[userCounts];
        userGenerator.GetUsers( userID, userCounts );
        for( int i = 0; i < userCounts; ++i )
        {
            if( skeletonCap->IsTracking( userID[i] ) )
            {
                XnPoint3D skelPointsIn[24],skelPointsOut[24];
                XnSkeletonJointTransformation mJointTran;
                for(int iter=0;iter<24;iter++)
                {
                    skeletonCap->GetSkeletonJoint( userID[i],XnSkeletonJoint(iter+1), mJointTran );
                    skelPointsIn[iter]=mJointTran.position.position;         
                }
                depthGenerator.ConvertRealWorldToProjective(24,skelPointsIn,skelPointsOut);

                for(int d=0;d<14;d++)
                {
                    CvPoint startpoint = cvPoint(skelPointsOut[startSkelPoints[d]].X,skelPointsOut[startSkelPoints[d]].Y+25);
                    CvPoint endpoint = cvPoint(skelPointsOut[endSkelPoints[d]].X,skelPointsOut[endSkelPoints[d]].Y+25);
                    cvLine(cameraImg,startpoint,endpoint,CV_RGB(0,0,255),2);
                }

                for(int d=0;d<15;d++)
                {
                    CvPoint point = cvPoint(skelPointsOut[SkelPoints[d]].X,skelPointsOut[SkelPoints[d]].Y+25);
                    cvCircle(cameraImg,point,3,CV_RGB(0,0,255),3);
                }
                for(int d=0;d<15;d++)
                {
                    Joint[d].setX(-skelPointsIn[SkelPoints[d]].X);
                    Joint[d].setY( skelPointsIn[SkelPoints[d]].Y);
                    Joint[d].setZ( skelPointsIn[SkelPoints[d]].Z);
                }
                Joint[10].setZ(skelPointsIn[SkelPoints[10]].Z-100);
                Joint[13].setZ(skelPointsIn[SkelPoints[13]].Z-100);

                generaFrameAndVector();
                angleDecode();
                Theta[15]+=5.0f;
                Theta[16]+=5.0f;
                //calibrate the joint angle, darwin real angle
                for(int i=0;i<20;i++)
                {
                    datadisplay[i].setText(QString::number((int)Theta[i+1]));
                }
                if(kinectSave==true)
                {
                    for(int i=0;i<19;i++)
                    {
                        *file<<datadisplay[i].text()+",";
                    }
                    *file<<datadisplay[19].text()<<"\r\n";
                }
            }
        }
        delete [] userID;
    }
    return cameraImg;
}

void kinectCal::generaFrameAndVector(void)
{
    QVector3D VectX,VectY,VectZ;
    VectX=Joint[3]-Joint[6];
    VectX.normalize();
    VectZ = QVector3D::normal(Joint[2]-Joint[6],VectX);
    VectY = QVector3D::normal(VectZ,VectX);

    QMatrix4x4 frameMat(VectX.x(),VectX.y(),VectX.z(),0,
                        VectY.x(),VectY.y(),VectY.z(),0,
                        VectZ.x(),VectZ.y(),VectZ.z(),0,
                        0,0,0,1);

    Vect[1]=Joint[1]-Joint[0];  //head
    Vect[2]=Joint[4]-Joint[3];  //left hand 1
    Vect[3]=Joint[5]-Joint[4];  //left hand 2
    Vect[4]=Joint[7]-Joint[6];  //right hand 1
    Vect[5]=Joint[8]-Joint[7];  //right hand 2
    Vect[6]=Joint[10]-Joint[ 9];//left leg 1
    Vect[7]=Joint[11]-Joint[10];//left leg 2
    Vect[8]=Joint[13]-Joint[12];//right leg 1
    Vect[9]=Joint[14]-Joint[13];//right leg 2

    for(int i=1;i<=9;i++)
    {
        Vect[i].normalize();
        Vect[i]=frameMat.map(Vect[i]);
    }
}

void kinectCal::angleDecode(void)
{
    Theta[3] = qAsin(Vect[2].x());
    Theta[4] = qAsin(Vect[4].x());
    Theta[5] = qAcos(constraint(QVector3D::dotProduct(Vect[2],Vect[3]),-1.f,1.f));
    Theta[6] = qAcos(constraint(QVector3D::dotProduct(Vect[4],Vect[5]),-1.f,1.f));
    Theta[1] = qAtan2(qSin(Theta[5])*Vect[3].y()-qCos(Theta[5])*qCos(Theta[3])*Vect[3].z(),
                           -qSin(Theta[5])*Vect[3].z()-qCos(Theta[5])*qCos(Theta[3])*Vect[3].y());
    Theta[2] = qAtan2(qSin(Theta[6])*Vect[5].y()-qCos(Theta[6])*qCos(Theta[4])*Vect[5].z(),
                           -qSin(Theta[6])*Vect[5].z()-qCos(Theta[6])*qCos(Theta[4])*Vect[5].y());
    Theta[7]  = 0;
    Theta[8]  = 0;
    Theta[9]  = qAtan2(Vect[6].x(),-Vect[6].y());
    Theta[10] = qAtan2(Vect[8].x(),-Vect[8].y());
    Theta[11] = -qAsin(Vect[6].z());
    Theta[12] = -qAsin(Vect[8].z());
    Theta[13] = -qAcos(constraint(QVector3D::dotProduct(Vect[6],Vect[7]),-1.f,1.f));
    Theta[14] = -qAcos(constraint(QVector3D::dotProduct(Vect[8],Vect[9]),-1.f,1.f));

    Theta[15] = 0;
    Theta[16] = 0;
    Theta[17] = 0;
    Theta[18] = 0;

    Theta[19] = 0;
    Theta[20] = qAcos(-Vect[1].y());

    for(int i=1;i<=20;i++)
    {
        Theta[i]=qRadiansToDegrees(Theta[i]);
        Theta[i]=constraint(Theta[i],minAngle[i],maxAngle[i]);
    }
    getGravity();
}

void kinectCal::getGravity(void)
{
    QMatrix4x4 Mat[21];
    QVector3D PG[21],PO[21];
    QVector3D PartPG[5]={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    QVector3D G,A,B;
    float AllGravity=0;
    float PartGm[5]={0,0,0,0,0};
    QVector3D LegOff[2]={{10,-33.5,0},{-10,-33.5,0}};

    Mat[1].setToIdentity();
    Mat[1].rotate(Theta[1],1,0,0);
    Mat[3]=Mat[1];
    Mat[3].rotate(Theta[3],0,0,1);
    Mat[5]=Mat[3];
    Mat[5].rotate(Theta[5],1,0,0);
    Mat[2].setToIdentity();
    Mat[2].rotate(Theta[2],1,0,0);
    Mat[4]=Mat[2];
    Mat[4].rotate(Theta[4],0,0,1);
    Mat[6]=Mat[4];
    Mat[6].rotate(Theta[6],1,0,0);

    Mat[7].setToIdentity();
    Mat[7].rotate(Theta[7],0,1,0);
    Mat[9]=Mat[7];
    Mat[9].rotate(Theta[9],0,0,1);
    Mat[11]=Mat[9];
    Mat[11].rotate(Theta[11],1,0,0);
    Mat[13]=Mat[11];
    Mat[13].rotate(Theta[13],1,0,0);
    Mat[15]=Mat[13];
    Mat[15].rotate(Theta[15],1,0,0);
    Mat[17]=Mat[15];
    Mat[17].rotate(Theta[17],0,0,1);

    Mat[8].setToIdentity();
    Mat[8].rotate(Theta[8],0,1,0);
    Mat[10]=Mat[8];
    Mat[10].rotate(Theta[10],0,0,1);
    Mat[12]=Mat[10];
    Mat[12].rotate(Theta[12],1,0,0);
    Mat[14]=Mat[12];
    Mat[14].rotate(Theta[14],1,0,0);
    Mat[16]=Mat[14];
    Mat[16].rotate(Theta[16],1,0,0);
    Mat[18]=Mat[16];
    Mat[18].rotate(Theta[18],0,0,1);

    Mat[19].setToIdentity();
    Mat[19].rotate(Theta[19],0,1,0);
    Mat[20]=Mat[19];
    Mat[20].rotate(Theta[20],1,0,0);
    for(int i=1;i<=20;i++)
        PG[i]=Mat[i].map(Pg[i]);
    //head
    PO[19]=Po[19];
    PO[20]=PO[19]+Mat[19].map(Po[20]);
    PartGm[0]=Gm[19]+Gm[20];
    for(int i=19;i<=20;i++)
        PartPG[0]+=(PG[i]+PO[i])*Gm[i];
    PartPG[0]/=PartGm[0];
    //right hand
    PO[1]=Po[1];
    PO[3]=PO[1]+Mat[1].map(Po[3]);
    PO[5]=PO[3]+Mat[3].map(Po[5]);
    PartGm[1]=Gm[1]+Gm[3]+Gm[5];
    for(int i=1;i<=5;i+=2)
        PartPG[1]+=(PG[i]+PO[i])*Gm[i];
    PartPG[1]/=PartGm[1];
    //left hand
    PO[2]=Po[2];
    PO[4]=PO[2]+Mat[2].map(Po[4]);
    PO[6]=PO[4]+Mat[4].map(Po[6]);
    PartGm[2]=Gm[1]+Gm[3]+Gm[5];
    for(int i=2;i<=6;i+=2)
        PartPG[2]+=(PG[i]+PO[i])*Gm[i];
    PartPG[2]/=PartGm[2];
    //right leg
    PO[7]=Po[7];
    PO[9]=PO[7]+Mat[7].map(Po[9]);
    PO[11]=PO[9]+Mat[9].map(Po[11]);
    PO[13]=PO[11]+Mat[11].map(Po[13]);
    PO[15]=PO[13]+Mat[13].map(Po[15]);
    PO[17]=PO[15]+Mat[15].map(Po[17]);
    PartGm[3]=Gm[7]+Gm[9]+Gm[11]+Gm[13]+Gm[15]+Gm[17];
    for(int i=7;i<=13;i+=2)
        PartPG[3]+=(PG[i]+PO[i])*Gm[i];
    PartPG[3]/=PartGm[3];
    //left leg
    PO[8]=Po[8];
    PO[10]=PO[8]+Mat[8].map(Po[10]);
    PO[12]=PO[10]+Mat[10].map(Po[12]);
    PO[14]=PO[12]+Mat[12].map(Po[14]);
    PO[16]=PO[14]+Mat[14].map(Po[16]);
    PO[18]=PO[16]+Mat[16].map(Po[18]);
    PartGm[4]=Gm[8]+Gm[10]+Gm[12]+Gm[14]+Gm[16]+Gm[18];
    for(int i=8;i<=14;i+=2)
        PartPG[4]+=(PG[i]+PO[i])*Gm[i];
    PartPG[4]/=PartGm[4];
    //gravity
    G=Pg[0]*Gm[0];
    AllGravity=Gm[0];
    for(int i=0;i<=5;i++)
    {
        G+=PartPG[i]*PartGm[i];
        AllGravity+=PartGm[i];
    }
    G/=AllGravity;
    A=PO[17]+Mat[17].map(LegOff[0]);
    B=PO[18]+Mat[18].map(LegOff[1]);
//    qDebug()<<"ABG"<<A<<B<<G;

//  calibration
//    QVector3D F=QVector3D::crossProduct(B-A,QVector3D::crossProduct(B-A,G-A)).normalized();
//    QMatrix4x4 MyMat[2];
//    MyMat[0].setToIdentity();
//    MyMat[0].rotate(-Theta[ 9],0,0,1);
//    MyMat[0].rotate(-Theta[11],1,0,0);
//    MyMat[0].rotate(-Theta[13],1,0,0);
//    MyMat[1].setToIdentity();
//    MyMat[1].rotate(-Theta[10],0,0,1);
//    MyMat[1].rotate(-Theta[12],1,0,0);
//    MyMat[1].rotate(-Theta[14],1,0,0);
//    Vect[10]=MyMat[0].map(F);Vect[11]=MyMat[1].map(F);
//    qDebug()<<"F"<<F<<"V10"<<Vect[10]<<"V11"<<Vect[11];
//    Theta[15]= qAtan(Vect[10].z()/Vect[10].y());
//    Theta[16]= qAtan(Vect[11].z()/Vect[11].y());
//    Theta[17]=-qAsin(Vect[10].x());
//    Theta[18]=-qAsin(Vect[11].x());

    QVector3D F=QVector3D::crossProduct(B-A,QVector3D::crossProduct(B-A,G-A)).normalized();
//    Vect[10]=(A-G).normalized();Vect[11]=(B-G).normalized();
    Vect[10]=F;Vect[11]=F;
    Vect[7]=Mat[13].map({0,-1,0});
    Vect[9]=Mat[14].map({0,-1,0});
//    qDebug()<<"F"<<F<<"V7"<<Vect[7]<<"V9"<<Vect[9];
    Theta[15]=qAtan(Vect[10].z()/Vect[10].y())+qAsin(Vect[ 7].z()/qSqrt(SQR(Vect[10].z())+SQR(Vect[10].y())));
    Theta[16]=qAtan(Vect[11].z()/Vect[11].y())+qAsin(Vect[ 9].z()/qSqrt(SQR(Vect[11].z())+SQR(Vect[11].y())));
    Theta[17]=qAtan(Vect[ 7].x()/Vect[ 7].y())+qAsin(Vect[10].x()/qSqrt(SQR(Vect[ 7].x())+SQR(Vect[ 7].y())));
    Theta[18]=qAtan(Vect[ 9].x()/Vect[ 9].y())+qAsin(Vect[11].x()/qSqrt(SQR(Vect[ 9].x())+SQR(Vect[ 9].y())));
    for(int i=15;i<=18;i++)
    {
        Theta[i]=qRadiansToDegrees(Theta[i]);
    }
}

void XN_CALLBACK_TYPE kinectCal::NewUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )
{
    qDebug() <<"New user identified:" << user;
    generator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
}

void XN_CALLBACK_TYPE kinectCal::LostUser( xn::UserGenerator& generator, XnUserID user,void* pCookie )
{
    qDebug() << "User " << user << " lost";
}

void XN_CALLBACK_TYPE kinectCal::CalibrationStart( xn::SkeletonCapability& skeleton,XnUserID user,void* pCookie )
{
    qDebug() << "Calibration start for user " <<  user;
}

void XN_CALLBACK_TYPE kinectCal::CalibrationEnd( xn::SkeletonCapability& skeleton,XnUserID user,XnCalibrationStatus calibrationError,void* pCookie )
{
    qDebug() << "Calibration complete for user " <<  user << ", ";
    if( calibrationError==XN_CALIBRATION_STATUS_OK )
    {
        qDebug() << "Success";
        skeleton.StartTracking( user );
    }
    else
    {
        qDebug() << "Failure";
        ((xn::UserGenerator*)pCookie)->GetPoseDetectionCap().StartPoseDetection( "Psi", user );
    }
}

void XN_CALLBACK_TYPE kinectCal::PoseDetected( xn::PoseDetectionCapability& poseDetection,const XnChar* strPose,XnUserID user,void* pCookie)
{
    qDebug() << "Pose " << strPose << " detected for user " <<  user;
    ((xn::UserGenerator*)pCookie)->GetSkeletonCap().RequestCalibration( user, FALSE );
    poseDetection.StopPoseDetection( user );
}
