#include "robotact.h"
#include "ui_robotact.h"

RobotAct::RobotAct(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RobotAct)
{
    ui->setupUi(this);
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("GB18030"));

    robotisInit();
    udpStatus = false;

    Kinect=new kinectCal(ui->Kinect);
    Kinect->datadisplay=lineEditSend;
    Kinect->dataDebug=lineEditRead;

    xyzView={20,195,0};
    for(int i=0;i<20;i++)
    {
        robotisAngle[i]=lineEditSend[i].text().toFloat();
    }
    vboxLayoutGl = new QVBoxLayout;
    glWidget = new GLWidget(robotisAngle,xyzView);
    vboxLayoutGl->addWidget(glWidget);
    ui->Visual3D->setLayout(vboxLayoutGl);

    statusInfo.setText("welcome to skyrobot!");
    ui->statusBar->addWidget(&statusInfo);
    ui->statusBar->addPermanentWidget(&statusDate);

    connect(ui->pushButtonKinectOpen,SIGNAL(clicked(bool)),this,SLOT(kinectOpenOrClose()));
    connect(ui->pushButtonRobotisLink,SIGNAL(clicked(bool)),this,SLOT(robotisLinkOrUnlink()));
    connect(&statusInfoTimer,SIGNAL(timeout()),this,SLOT(statusInfoTimerUpdate()));
    connect(&statusDateTimer,SIGNAL(timeout()),this,SLOT(statusDateTimerUpdate()));
    connect(ui->checkBoxSaveKinectJoins,SIGNAL(clicked(bool)),this,SLOT(kinectDataSave()));
    connect(ui->checkBoxSendKinectJoins,SIGNAL(clicked(bool)),this,SLOT(robotisTimeSend()));
    connect(ui->checkBox3D,SIGNAL(clicked(bool)),this,SLOT(kinectEnable3D()));
    connect(ui->pushButtonSaveJoins,SIGNAL(clicked(bool)),this,SLOT(robotisSaveJoins()));
    connect(ui->pushButtonReadJoins,SIGNAL(clicked(bool)),this,SLOT(robotisReadJoins()));
    connect(ui->pushButtonSendJoins,SIGNAL(clicked()),this,SLOT(robotisSendJoins()));
    connect(ui->pushButtonOpenFile,SIGNAL(clicked(bool)),this,SLOT(robotisFileOpen()));
    connect(ui->pushButtonSendFile,SIGNAL(clicked(bool)),this,SLOT(robotisFileSend()));
    connect(ui->pushButtonOrder_1,SIGNAL(clicked(bool)),this,SLOT(robotisOrder1()));
    connect(ui->pushButtonOrder_2,SIGNAL(clicked(bool)),this,SLOT(robotisOrder2()));
    connect(ui->pushButtonOrder_3,SIGNAL(clicked(bool)),this,SLOT(robotisOrder3()));
    connect(ui->pushButtonOrder_4,SIGNAL(clicked(bool)),this,SLOT(robotisOrder4()));
    connect(ui->pushButtonOrder_5,SIGNAL(clicked(bool)),this,SLOT(robotisOrder5()));
    connect(&viewTimer,SIGNAL(timeout()),this,SLOT(kinectView()));
    statusDateTimer.start(1000);
}

RobotAct::~RobotAct()
{
    delete Kinect;
    delete ui;
    delete [] labelSend;
    delete [] lineEditSend;
    delete [] checkBoxSend;
    delete [] labelRead;
    delete [] lineEditRead;
}

void RobotAct::kinectOpenOrClose()
{
    static bool firstOpen = true;
    if(firstOpen)
    {
        firstOpen=false;
        Kinect->start();
    }
    Kinect->kinectStatus=!Kinect->kinectStatus;
    if(Kinect->kinectStatus)
    {
        ui->pushButtonKinectOpen->setText(tr("kinect(close)"));
        ui->checkBoxSaveKinectJoins->setEnabled(true);
    }
    else
    {
        ui->pushButtonKinectOpen->setText(tr("kinect(open)"));
        ui->checkBoxSaveKinectJoins->setEnabled(false);
    }
}

void RobotAct::kinectDataSave()
{
    if(ui->checkBoxSaveKinectJoins->isChecked())
    {
        kinectFile = new QFile((QString)"kinect data"+QDir::separator()+statusDate.text());
        kinectTextStream=new QTextStream(kinectFile);
        if(!kinectFile->open(QFile::ReadWrite|QIODevice::Text))
        {
            QMessageBox::warning(this,tr("error"),tr("open motion file fail!"),tr("OK"));
            return;
        }
        Kinect->file=kinectTextStream;
        Kinect->kinectSave=true;
    }
    else
    {
        Kinect->kinectSave=false;
        kinectFile->close();
        delete kinectFile;
    }
}

void RobotAct::kinectView()
{
    if(ui->tabWidgetKinect->currentIndex()==1)
    {

        glWidget->cleanup();
        glWidget->close();
        delete glWidget;
        for(int i=0;i<20;i++)
        {
            robotisAngle[i]=lineEditSend[i].text().toFloat();
        }
        glWidget = new GLWidget(robotisAngle,xyzView);
        vboxLayoutGl->addWidget(glWidget);
    }
}

void RobotAct::kinectEnable3D()
{
    xyzView.setX(glWidget->m_xRot);
    xyzView.setY(glWidget->m_yRot);
    xyzView.setZ(glWidget->m_zRot);
    if(ui->checkBox3D->isChecked())
        viewTimer.start(500);
    else
        viewTimer.stop();
}

void RobotAct::robotisLinkOrUnlink()
{
    if(!udpStatus)
    {
        udpPortRobotRecv =  ui->lineEditPortRobotRecv->text().toInt();
        udpPortRobotSend =  ui->lineEditPortRobotSend->text().toInt();
        if(!udpIPRobot.setAddress(ui->lineEditIPRobot->text()))
        {
            QMessageBox::information(this,tr("error"),tr("server ip address error!"));
            return;
        }
        //udpIPRobot=QHostAddress::Any;
        udpSocketRobotRecv = new QUdpSocket();
        udpSocketRobotSend = new QUdpSocket();
        udpSocketRobotRecv->bind(QHostAddress::Any, udpPortRobotRecv);
        connect(udpSocketRobotRecv, SIGNAL(readyRead()),this, SLOT(robotisUdpRecv()));
        ui->pushButtonOrder_1->setEnabled(true);
        ui->pushButtonOrder_2->setEnabled(true);
        ui->pushButtonOrder_3->setEnabled(true);
        ui->pushButtonOrder_4->setEnabled(true);
        ui->pushButtonOrder_5->setEnabled(true);
        ui->pushButtonReadJoins->setEnabled(true);
        ui->pushButtonSaveJoins->setEnabled(true);
        ui->pushButtonSendJoins->setEnabled(true);
        ui->pushButtonSendFile->setEnabled(true);
        ui->pushButtonCmd->setEnabled(true);
        ui->checkBoxSendKinectJoins->setEnabled(true);
        ui->pushButtonRobotisLink->setText(tr("unlink"));
        udpStatus=true;
    }
    else
    {
        ui->pushButtonOrder_1->setEnabled(false);
        ui->pushButtonOrder_2->setEnabled(false);
        ui->pushButtonOrder_3->setEnabled(false);
        ui->pushButtonOrder_4->setEnabled(false);
        ui->pushButtonOrder_5->setEnabled(false);
        ui->pushButtonReadJoins->setEnabled(false);
        ui->pushButtonSaveJoins->setEnabled(false);
        ui->pushButtonSendJoins->setEnabled(false);
        ui->pushButtonSendFile->setEnabled(false);
        ui->pushButtonCmd->setEnabled(false);
        ui->checkBoxSendKinectJoins->setEnabled(false);
        ui->pushButtonRobotisLink->setText(tr("link"));
        udpStatus=false;
        delete udpSocketRobotRecv;
        delete udpSocketRobotSend;
    }
}

void RobotAct::robotisUdpRecv()
{
    QByteArray datagram;
    QHostAddress udpIpSendFrom;
    quint16 udpIpSendPort;
    datagram.resize(udpSocketRobotRecv->pendingDatagramSize());
    udpSocketRobotRecv->readDatagram(datagram.data(), datagram.size(),&udpIpSendFrom,&udpIpSendPort);

    if(udpIpSendFrom.toString().contains("192.168.1.107"))
    {
        if(datagram.left(1)!="$"||datagram.right(2)!="\r\n")//$DAT,[1],[2]\r\n
        {
            statusInfo.setText("udp lose some data");
            statusInfoTimer.start(1000);
            return;
        }
        QByteArray dataType=datagram.mid(1,3);
        datagram=datagram.mid(5,datagram.size()-7);
        qDebug()<<datagram;
        if(dataType=="DAT")
        {
            QList<QByteArray> list=datagram.split(',');
            lineEditSend[20].setText(list.at(0));
            lineEditSend[21].setText(list.at(1));
        }
    }
    if(udpIpSendFrom.toString().contains(udpIPRobot.toString()))
    {
        if(datagram.left(1)!="$"||datagram.right(2)!="\r\n")//$DAT,[ID],[1]...[20],20\r\n
        {
            statusInfo.setText("udp lose some data");
            statusInfoTimer.start(1000);
            return;
        }
        QByteArray dataType=datagram.mid(1,3);
        datagram=datagram.mid(5,datagram.size()-10);
        qDebug()<<datagram;
        if(dataType=="DAT")
        {
            QList<QByteArray> list=datagram.split(',');
            for(int i=0;i<20;i++)
            {
                lineEditRead[i].setText(list.at(i));
            }
            if(ui->checkBoxCopyJoinsToKinect->isChecked())
            {
                for(int i=0;i<20;i++)
                {
                    lineEditSend[i].setText(lineEditRead[i].text());
                }
            }
        }
        if(dataType=="TIP")
        {
            statusInfo.setText(datagram.data());
            statusInfoTimer.start(1000);
        }
    }
}

void RobotAct::robotisReadJoins()
{
    QString msg="$READ,";
    for(int i=0;i<20;i++)
    {
        if(checkBoxSend[i].isChecked()==true)
            msg+="1";
        else
            msg+="0";
    }
    msg+=",1\r\n";
    qDebug()<<msg;

    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisSaveJoins()
{
    static bool firstUse=true;
    if(firstUse)
    {
        robotFile = new QFile((QString)"robotis data"+QDir::separator()+statusDate.text());
        robotTextStream=new QTextStream(robotFile);
        if(!robotFile->open(QFile::ReadWrite|QIODevice::Text))
        {
            QMessageBox::warning(this,tr("error"),tr("open motion file fail!"),tr("OK"));
            return;
        }
        firstUse=false;
    }
    for(int i=0;i<19;i++)
    {
        *robotTextStream<<lineEditRead[i].text()+",";
    }
    *robotTextStream<<lineEditRead[19].text()<<"\r\n";
    robotTextStream->flush();
}

void RobotAct::robotisSendJoins()
{
    QString msg="$DATA,";
    for(int i=0;i<20;i++)
    {
       msg+=(lineEditSend[i].text()+",");
    }
    msg+=ui->spinBoxTime->text();
    msg+=",21\r\n";
    qDebug()<<msg;

    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisTimeSend()
{
    if(ui->checkBoxSendKinectJoins->isChecked())
    {
        timerSend.start(ui->spinBoxTime->value());
        connect(&timerSend,SIGNAL(timeout()),this,SLOT(robotisSendJoins()));
    }
    else
    {
        timerSend.stop();
        disconnect(&timerSend,SIGNAL(timeout()),this,SLOT(robotisSendJoins()));
    }
}

void RobotAct::robotisFileOpen()
{
    QString filename=QFileDialog::getOpenFileName(this,tr("select a file to send!"),"");
    if(filename.isEmpty())
    {
        QMessageBox::warning(this,tr("warning"),tr("no file has been selected"),tr("ok"));
        return;
    }
    ui->lineEditFileName->setText(filename);
}

void RobotAct::robotisFileSend()
{
    QFile *fileSend = new QFile(ui->lineEditFileName->text());
    if(!fileSend->open(QIODevice::ReadOnly))
    {
        QMessageBox::warning(this,tr("error"),tr("open file failed"),tr("ok"));
        return;
    }
    QTextStream in(fileSend);
    QString text=in.readAll();
    fileSend->close();
    delete fileSend;
    QString fileID=QDateTime::currentDateTime().toString("hhmmss");
    QStringList textlist=text.split("\r\n",QString::SkipEmptyParts);
    for(int i=0;i<textlist.size();i++)
    {
        QString msg="$FILE,"+fileID+","+textlist.at(i)+"\r\n";
        qDebug()<<msg;
        QByteArray datagram = msg.toLatin1();
        udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
    }
}

void RobotAct::robotisCmdSend()
{
    QString msg=ui->lineEditCmd->text();
    if(ui->checkBoxNewline->isChecked())msg+="\r\n";
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisOrder1()
{
    QString msg="$INIT\r\n";
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisOrder2()
{
    QString msg="$DOWN\r\n";
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisOrder3()
{
    QString msg="$ORDER3";
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisOrder4()
{
    QString msg="$PLAY\r\n";//confirm
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisOrder5()
{
    QString msg="$ORDER5";
    qDebug()<<msg;
    QByteArray datagram = msg.toLatin1();
    udpSocketRobotSend->writeDatagram(datagram, datagram.size(), udpIPRobot, udpPortRobotSend);
}

void RobotAct::robotisInit()
{
    //按配置信息生成参数界面
    widgetSend = new QWidget(ui->groupBoxSend);
    widgetSend->setGeometry(QRect(0, 20, ui->groupBoxSend->geometry().width(), ui->groupBoxSend->geometry().height()-20));
    gridLayoutSend = new QGridLayout(widgetSend);
    labelSend=new QLabel[22];
    lineEditSend=new QLineEdit[22];
    checkBoxSend=new QCheckBox[22];
    for(int i=0;i<22;i++)
    {
        gridLayoutSend->addWidget(&labelSend[i],i/2,i%2*3,1,1);
        gridLayoutSend->addWidget(&lineEditSend[i],i/2,i%2*3+1,1,1);
        gridLayoutSend->addWidget(&checkBoxSend[i],i/2,i%2*3+2,1,1);
        labelSend[i].setText(jointName[i]);
        checkBoxSend[i].setChecked(true);
    }
    widgetRead = new QWidget(ui->groupBoxRead);
    widgetRead->setGeometry(QRect(0, 20, ui->groupBoxRead->geometry().width(), ui->groupBoxRead->geometry().height()-20));
    gridLayoutRead = new QGridLayout(widgetRead);
    labelRead=new QLabel[22];
    lineEditRead=new QLineEdit[22];
    for(int i=0;i<22;i++)
    {
        gridLayoutRead->addWidget(&labelRead[i],i/2,i%2*2,1,1);
        gridLayoutRead->addWidget(&lineEditRead[i],i/2,i%2*2+1,1,1);
        labelRead[i].setText(jointName[i]);
    }
    //initial value
    for(int i=0;i<22;i++)
    {
        lineEditSend[i].setText("0");
        lineEditRead[i].setText("0");
    }
    //touch motion direction
    QDir dir;
    if(dir.mkpath(QDir::currentPath()+QDir::separator()+"kinect data"+QDir::separator())==false)
    {
        QMessageBox::warning(this,tr("error"),tr("touch kinect data direction fail!"),tr("OK"));
        return;
    }
    if(dir.mkpath(QDir::currentPath()+QDir::separator()+"robotis data"+QDir::separator())==false)
    {
        QMessageBox::warning(this,tr("error"),tr("touch robotis data direction fail!"),tr("OK"));
        return;
    }
}

void RobotAct::statusDateTimerUpdate()
{
    statusDate.setText(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
}

void RobotAct::statusInfoTimerUpdate()
{
    statusInfo.setText("welcome to skyrobot!");
}

