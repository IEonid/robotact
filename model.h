#ifndef MODEL_H
#define MODEL_H

#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <QMatrix4x4>

class model
{
public:
    model(float *robAngle);
    const GLfloat *constData() const { return m_data.constData(); }
    int count() const { return m_count; }
    int vertexCount() const { return m_count / 6; }

private:
    void add(const QVector3D &v, const QVector3D &n);
    void tri(QVector3D v1,QVector3D v2,QVector3D v3,QMatrix4x4* tran);
    void qaut(QVector3D v1,QVector3D v2,QVector3D v3,QVector3D v4,QMatrix4x4* tran);
    void cuboid(GLfloat Length,GLfloat Width,GLfloat Height,QMatrix4x4* tran);
    void column(GLfloat Radius,GLfloat Height,QMatrix4x4* tran);
    void ball(GLfloat Radius,QMatrix4x4* tran);
    void joints(GLfloat Radius,GLfloat Height,GLfloat Scale,QMatrix4x4* tran);

    QVector<GLfloat> m_data;
    int m_count;
};

#endif // MODEL_H
