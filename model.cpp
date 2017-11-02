#include "model.h"
#include <qmath.h>

model::model(float *robAngle)
    : m_count(0)
{
    m_data.resize(100000 * 6);

    GLfloat BodyL=0.18,BodyW=0.08,BodyH=0.2,JointsR=0.03;
    QMatrix4x4 bodyMat;
    bodyMat.setToIdentity();
    cuboid(BodyL,BodyW,BodyH,&bodyMat);
    QVector3D head(0,BodyH/2+JointsR,0),handR(BodyL/2+JointsR,BodyH/4,0),handL(-BodyL/2-JointsR,BodyH/4,0),footR(BodyL/4,-BodyH/2-JointsR,0),footL(-BodyL/4,-BodyH/2-JointsR,0);
    QVector3D InitVector(0,-0.14,0),InitVector2(0,-0.16,0);
//    GLfloat jointRad[20]={0,0,45,-45,45,45,0,0,10,-10,30,30,-60,-60,30,30,-10,10,0,0};
    GLfloat *jointRad=robAngle;
    QMatrix4x4 JointMat[11];
    for(int i=0;i<9;i++)
    {
        JointMat[i].setToIdentity();
    }

    JointMat[0].translate(head);
    JointMat[0].rotate(180, 0, 0, 1);
    JointMat[0].rotate(jointRad[18], 0, 1, 0);
    JointMat[0].rotate(jointRad[19], 1, 0, 0);
    joints(JointsR,0.08,1.5,&JointMat[0]);

    JointMat[1].translate(handR);
    JointMat[1].rotate(jointRad[0], 1, 0, 0);
    JointMat[1].rotate(jointRad[2], 0, 0, 1);
    joints(JointsR,0.08,1,&JointMat[1]);

    JointMat[2].translate(handL);
    JointMat[2].rotate(jointRad[1], 1, 0, 0);
    JointMat[2].rotate(jointRad[3], 0, 0, 1);
    joints(JointsR,0.08,1,&JointMat[2]);

    JointMat[3].translate(JointMat[1].map(InitVector));
    JointMat[3].rotate(jointRad[0], 1, 0, 0);
    JointMat[3].rotate(jointRad[2], 0, 0, 1);
    JointMat[3].rotate(jointRad[4], 1, 0, 0);
    joints(JointsR,0.1,0.9,&JointMat[3]);

    JointMat[4].translate(JointMat[2].map(InitVector));
    JointMat[4].rotate(jointRad[1], 1, 0, 0);
    JointMat[4].rotate(jointRad[3], 0, 0, 1);
    JointMat[4].rotate(jointRad[5], 1, 0, 0);
    joints(JointsR,0.1,0.9,&JointMat[4]);

    JointMat[5].translate(footR);
    JointMat[5].rotate(jointRad[6], 0, 1, 0);
    JointMat[5].rotate(jointRad[8], 0, 0, 1);
    JointMat[5].rotate(jointRad[10],1, 0, 0);
    joints(JointsR,0.08,1,&JointMat[5]);

    JointMat[6].translate(footL);
    JointMat[6].rotate(jointRad[7], 0, 1, 0);
    JointMat[6].rotate(jointRad[9], 0, 0, 1);
    JointMat[6].rotate(jointRad[11],1, 0, 0);
    joints(JointsR,0.08,1,&JointMat[6]);

    JointMat[7].translate(JointMat[5].map(InitVector));
    JointMat[7].rotate(jointRad[6], 0, 1, 0);
    JointMat[7].rotate(jointRad[8], 0, 0, 1);
    JointMat[7].rotate(jointRad[10],1, 0, 0);
    JointMat[7].rotate(jointRad[12],1, 0, 0);
    joints(JointsR,0.1,1.2,&JointMat[7]);

    JointMat[8].translate(JointMat[6].map(InitVector));
    JointMat[8].rotate(jointRad[7], 0, 1, 0);
    JointMat[8].rotate(jointRad[9], 0, 0, 1);
    JointMat[8].rotate(jointRad[11],1, 0, 0);
    JointMat[8].rotate(jointRad[13],1, 0, 0);
    joints(JointsR,0.1,1.2,&JointMat[8]);

    JointMat[9].translate(JointMat[7].map(InitVector2));
    JointMat[9].rotate(jointRad[6], 0, 1, 0);
    JointMat[9].rotate(jointRad[8], 0, 0, 1);
    JointMat[9].rotate(jointRad[10],1, 0, 0);
    JointMat[9].rotate(jointRad[12],1, 0, 0);
    JointMat[9].rotate(jointRad[14],1, 0, 0);
    JointMat[9].rotate(jointRad[16],0, 0, 1);
    joints(JointsR,0.02,1.8,&JointMat[9]);

    JointMat[10].translate(JointMat[8].map(InitVector2));
    JointMat[10].rotate(jointRad[7], 0, 1, 0);
    JointMat[10].rotate(jointRad[9], 0, 0, 1);
    JointMat[10].rotate(jointRad[11],1, 0, 0);
    JointMat[10].rotate(jointRad[13],1, 0, 0);
    JointMat[10].rotate(jointRad[15],1, 0, 0);
    JointMat[10].rotate(jointRad[17],0, 0, 1);
    joints(JointsR,0.02,1.8,&JointMat[10]);
}

void model::add(const QVector3D &v, const QVector3D &n)
{
    GLfloat *p = m_data.data() + m_count;
    *p++ = v.x();
    *p++ = v.y();
    *p++ = v.z();
    *p++ = n.x();
    *p++ = n.y();
    *p++ = n.z();
    m_count += 6;
}

void model::tri(QVector3D v1,QVector3D v2,QVector3D v3,QMatrix4x4* tran)//
{
    QVector3D n = tran->map(QVector3D::normal(v1,v2,v3));
    add(tran->map(v1),n);
    add(tran->map(v2),n);
    add(tran->map(v3),n);
}

void model::qaut(QVector3D v1,QVector3D v2,QVector3D v3,QVector3D v4,QMatrix4x4* tran)//
{
    tri(v1,v2,v3,tran);
    tri(v1,v3,v4,tran);
}

void model::cuboid(GLfloat Length,GLfloat Width,GLfloat Height,QMatrix4x4* tran)
{
    GLfloat x=Length/2.0;
    GLfloat y=Height/2.0;
    GLfloat z=Width/2.0;
    QVector3D v1(x,y,z),v2(-x,y,z),v3(-x,-y,z),v4(x,-y,z),v5(x,y,-z),v6(-x,y,-z),v7(-x,-y,-z),v8(x,-y,-z);
    qaut(v1,v2,v3,v4,tran);//front
    qaut(v6,v5,v8,v7,tran);//back
    qaut(v1,v5,v6,v2,tran);//up
    qaut(v4,v3,v7,v8,tran);//down
    qaut(v2,v6,v7,v3,tran);//left
    qaut(v1,v4,v8,v5,tran);//right
}

void model::column(GLfloat Radius,GLfloat Height,QMatrix4x4* tran)
{
    GLfloat x1,z1,x2,z2,y=Height/2.0,angle;
    int NumSectors = 40;
    for (int i = 0; i < NumSectors; ++i) //up,down
    {
        angle = (i * 2 * M_PI) / NumSectors;
        x1 = Radius * qSin(angle);
        z1 = Radius * qCos(angle);

        angle = ((i + 1) * 2 * M_PI) / NumSectors;
        x2 = Radius * qSin(angle);
        z2 = Radius * qCos(angle);

        QVector3D v1(0.0,y,0.0),v2(x1,y,z1),v3(x2,y,z2),v4(0.0,-y,0.0),v5(x1,-y,z1),v6(x2,-y,z2);
        tri(v1,v2,v3,tran);
        tri(v6,v5,v4,tran);
        qaut(v3,v2,v5,v6,tran);
    }
}

void model::ball(GLfloat Radius,QMatrix4x4* tran)
{
    int NumSectors = 20;
    GLfloat x11,x12,x21,x22,y11,y12,y21,y22,z1,z2,XY1,XY2,angleZ,angle;
    for(int i=0;i<NumSectors;++i)
    {
        angleZ = (i * 2 * M_PI) / NumSectors;
        z1 = Radius * qSin(angleZ);
        XY1 = Radius * qCos(angleZ);
        angleZ = ((i + 1) * 2 * M_PI) / NumSectors;
        z2 = Radius * qSin(angleZ);
        XY2 = Radius * qCos(angleZ);
        for(int j=0;j<NumSectors;++j)
        {
            angle = (j * 2 * M_PI) / NumSectors;
            x11 = XY1 * qSin(angle);
            y11 = XY1 * qCos(angle);
            x21 = XY2 * qSin(angle);
            y21 = XY2 * qCos(angle);
            angle = ((j + 1) * 2 * M_PI) / NumSectors;
            x12 = XY1 * qSin(angle);
            y12 = XY1 * qCos(angle);
            x22 = XY2 * qSin(angle);
            y22 = XY2 * qCos(angle);

            QVector3D v1(x11,y11,z1),v2(x12,y12,z1),v3(x22,y22,z2),v4(x21,y21,z2);

            qaut(v1,v2,v3,v4,tran);
        }
    }
}

void model::joints(GLfloat Radius,GLfloat Height,GLfloat Scale,QMatrix4x4* tran)
{
    float columnRadius=Radius*Scale;
    ball(Radius,tran);
    GLfloat x1,z1,x2,z2,y=Height+Radius,angle;
    int NumSectors = 40;
    for (int i = 0; i < NumSectors; ++i) //up,down
    {
        angle = (i * 2 * M_PI) / NumSectors;
        x1 = columnRadius * qSin(angle);
        z1 = columnRadius * qCos(angle);

        angle = ((i + 1) * 2 * M_PI) / NumSectors;
        x2 = columnRadius * qSin(angle);
        z2 = columnRadius * qCos(angle);

        QVector3D v1(0.0,-Radius,0.0),v2(x1,-Radius,z1),v3(x2,-Radius,z2),v4(0.0,-y,0.0),v5(x1,-y,z1),v6(x2,-y,z2);
        tri(v1,v2,v3,tran);
        tri(v6,v5,v4,tran);
        qaut(v3,v2,v5,v6,tran);
    }
}
