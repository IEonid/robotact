#ifndef ROBOTACT_H
#define ROBOTACT_H

#include <QMainWindow>
#include <kinectcal.h>
#include "kinectcal.h"
#include <QtNetwork/QUdpSocket>
#include <QTimer>
#include <QTextCodec>
#include <QGridLayout>
#include <QLineEdit>
#include <QLabel>
#include <QFile>
#include <QDir>
#include <QCheckBox>
#include <QDateTime>
#include <QTextEdit>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include "glwidget.h"

namespace Ui {
class RobotAct;
}

//1、（-180 0 180）=（0 2036 4095）  +11.4
//2、（-180 0 180）=（4095 2106 0）  -11.4
//3、（180 90 0）=（3518 2526 1553）  +10.9
//4、（-180 -90 0）=（556 1518 2548） +11.0
//5、（0 90 135 极限）=（ 978 2082 2582 2818） +11.8
//6、（0 90 135 极限）=（3100 1986 1502 1270） -11.8
//7、（30 0 -30）=（1685 2063 2354） -11.2
//8、（30 0 -30）=（1685 2063 2354） -11.2
//9、（60 0 -30）=（2709 2051 1720） +11.0
//10、（30 0 -60）=（2390 2064 1418） +10.8
//11、（90 0 -20）=（ 922 1987 2354） -11.8
//12、（90 0 -20）=（3172 2076 1742） +12.1
//13、（0 -130）=（2041 3471） -11.0
//14、（0 -130）=（2075  645） +10.8
//15、（-60 -45 0 90）=（1409 1572 1993 2945） +10.6
//16、（-60 -45 0 90）=（2672 2533 2092 1160） -10.4
//17、（30 0 -30）=（1748 2031 2352） -10.1
//18、（30 0 -30）=（1730 2069 2354） -10.4
//19、（-90 0 90）=（ 999 2046 3043） +11.4
//20、（10 0 -45）=（2582 2405 1848） +12.3

const QString jointName[22]={"SHOULDER-RIGHT","SHOULDER-LEFT","ARM-RIGHT","ARM-LEFT","ELBOW-RGIHT","ELBOW-LEFT",
                             "BUTTOCK-RIGHT","BUTTOCK-LEFT","HIP-RIGHT","HIP-LEFT",
                             "THIGH-RIGHT","THIGH-LEFT", "KNEE-RIGHT","KNEE-LEFT",
                             "ANKEL-RIGHT","ANKEL-LEFT","FOOT-RIGHT","FOOT-LEFT","NECK","HEAD","PARA1","PARA2"};
//const QString jointName[22]={"右肩","左肩","右臂","左臂","右肘","左肘","右臀","左臀","右胯","左胯",
//                             "右腿","左腿", "右膝","左膝","右足","左足","右掌","左掌","颈部","头部","滚转","俯仰"};

const int angleTable[20]={2036,2062,1553,2557,1017,3032,2059,2063,2051,2064,
2042,2076,2041,2075,2055,2062,2031,2069,2046,2405};

const int angleDir[20]={1,-1,1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,-1,1};

const int angleUnit = 11;

class RobotAct : public QMainWindow
{
    Q_OBJECT

public:
    explicit RobotAct(QWidget *parent = 0);
    ~RobotAct();

private slots:
    void kinectOpenOrClose();
    void kinectDataSave();
    void kinectView();
    void kinectEnable3D();
    void robotisLinkOrUnlink();
    void robotisUdpRecv();
    void robotisReadJoins();
    void robotisSaveJoins();
    void robotisSendJoins();
    void robotisTimeSend();
    void robotisFileOpen();
    void robotisFileSend();
    void robotisCmdSend();
    void robotisOrder1();
    void robotisOrder2();
    void robotisOrder3();
    void robotisOrder4();
    void robotisOrder5();
    void statusInfoTimerUpdate();
    void statusDateTimerUpdate();

private:
    void robotisInit();

    Ui::RobotAct *ui;
    kinectCal* Kinect;

    bool udpStatus;
    QUdpSocket *udpSocketRobotRecv;
    QUdpSocket *udpSocketRobotSend;
    QHostAddress udpIPRobot;
    int udpPortRobotRecv;
    int udpPortRobotSend;

    QTimer timerSend;

    QTimer statusInfoTimer;
    QTimer statusDateTimer;
    QLabel statusInfo;
    QLabel statusDate;
    //widget
    GLWidget *glWidget;
    QVBoxLayout *vboxLayoutGl;
    float robotisAngle[20];
    QVector3D xyzView;
    QTimer viewTimer;

    QWidget *widgetSend;
    QGridLayout *gridLayoutSend;
    QLabel *labelSend;
    QLineEdit *lineEditSend;
    QCheckBox *checkBoxSend;

    QWidget *widgetRead;
    QGridLayout *gridLayoutRead;
    QLabel *labelRead;
    QLineEdit *lineEditRead;

    QFile *kinectFile;
    QTextStream *kinectTextStream;
    QFile *robotFile;
    QTextStream *robotTextStream;
};

#endif // ROBOTACT_H
