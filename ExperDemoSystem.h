#pragma once
#ifndef EXPERDEMOSYSTEM_H
#define EXPERDEMOSYSTEM_H

#include <QtWidgets/QWidget>
#include "ui_ExperDemoSystem.h"

//Windows Header
#include <windows.h>

//C++ header
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>

//OpenCV Headers
#include <opencv2\highgui\highgui_c.h>
#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv.hpp>

//Andy's Personal Headers
#include "MyLibs/AndysOpenCVLib.h"
#include "Mylibs/Talk2MomentCamera.h"
#include "Mylibs/Talk2TLDC4104.h"
#include "MyLibs/AndysComputations.h"
#include "MyLibs/NeuronAnalysis.h"
#include "MyLibs/experiment.h"

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QTime>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QBitmap>
#include <QKeyEvent>
#include <QIcon>
#include <QFileDialog>
#include <QTcpSocket>
#include <QTcpServer>
#include <QTextEdit>
#include <QRandomGenerator>
#include <QDebug>

//主线程类
class ExperDemoSystem : public QWidget
{
    Q_OBJECT

public:
    ExperDemoSystem(QWidget *parent = nullptr);
    ~ExperDemoSystem();

    int PreCamOperation();

public:
    Experiment* exp;
    int e;

    double totalDistance;   //线虫移动的总距离
    double curVelocity;   //线虫当前的速度
    float moveStep;    //位移台移动一步的距离 单位mm
    double nematodePosition_x;   
    double nematodePosition_y;    //线虫的位置显示
    int Amp;    //电压输出幅值
    std::unordered_map<int, int> RealVoltage;  //真实对照的电压值
    int Fre;    //电压输出频率
    int DutyCycle;   //方波占空比

    //定时器事件 显示图像
    void timerEvent(QTimerEvent* e);

    void ImageLabel();   //用于有关线虫图像的显示操作都放在这里

    void KeyTimerConnect(); //用于按键的连续触发

    void m_SpinBoxFinished(); //连接SpinBox完成信号

    void SynchroControlPanel(); //同步控制界面参数

    void TcpSocketSlotConnection(); //Tcp槽函数

    QImage MatToImage(const cv::Mat& mat);

	// 电场刺激计时器相关
	QTimer* m_efTimer;       // 用于定时触发UI更新
	QTime m_efElapsedTime;   // 用于计算经过的时间
    
private slots:

    /* 图像显示参数控件 */
    void on_ExpBegin_clicked();  //开始实验按钮
    void on_ContrastImp_clicked(); //增强对比度按钮
    void m_Diameter_valueChanged(); //圆形Mask半径按钮
    void m_UpBoundary_valueChanged();  //矩形上边界
    void on_UpBoundarySlider_valueChanged(int value);
    void m_RectHeight_valueChanged();  //矩形高度
    void on_RectHeightSlider_valueChanged(int value);
    void m_BinThreshBox_valueChanged();  //二值化阈值
    //void m_DilateThresh_valueChanged();  //膨胀参数
    //void on_DilateSlider_valueChanged(int value);
    void m_ErodesThresh_valueChanged();   //腐蚀参数
    void on_ErodesSlider_valueChanged(int value);
    void on_BinThreshSlider_valueChanged(int value);   //二值化阈值滑条
    void on_CircleRbt_clicked();  //Mask选择
    void on_RectRbt_clicked();

    /* 位移台参数控件 */
    void on_Up_clicked();
    void on_Down_clicked();
    void on_Right_clicked();
    void on_Left_clicked();   //位移台方向控制
    void m_StageSpeed_valueChanged();  //位移台速度
    void on_axisRadioButton_toggled(bool checked);//跟踪轴锁定
    void on_Track_clicked();    //位移台跟踪开关
    void on_OriginPoint_clicked();   //原点设定按钮
    void m_AccSet_valueChanged();  //加速度设定

    /* 光遗传刺激模块控件 */
    void on_LedSlider_valueChanged(int value);  //Led亮度滑条
    void m_LedSpinBox_valueChanged();  //Led亮度
    void on_LedOnOff_clicked();   //Led开关

    /* 电场刺激模块控价 */
    void on_voltageSlider_valueChanged(int value);
    void on_voltageBox_valueChanged();    //电压幅值
    void on_waveformBox_currentIndexChanged(int);
    void on_Electric1_Stop_clicked();
    void on_Electric1_Right_clicked();
    void on_Electric1_Down_clicked();
    void on_Electric1_Left_clicked();
    void on_Electric1_Up_clicked();
    void updateTimerDisplay();  //更新计时器显示的槽函数

    /* 远程连接模块 */
    void on_Random_Stimulus_clicked();
    //void on_operation1_clicked();
    void on_operation2_clicked();
    void on_operation3_clicked();
    void on_RemoteConnect_clicked();

    /* 储存模块控件 */
    void on_SaveData_clicked();  //储存数据
    void on_ChoseFolder_clicked();  //选择文件夹

    /* 用于按键的连续触发 */
    void onUpButtonPressed();
    void onUpButtonReleased();
    void onDownButtonPressed();
    void onDownButtonReleased();
    void onLeftButtonPressed();
    void onLeftButtonReleased();
    void onRightButtonPressed();
    void onRightButtonReleased();

    void onUpButtonTimeout();
    void onDownButtonTimeout();
    void onLeftButtonTimeout();
    void onRightButtonTimeout();

    //远程连接槽函数
    void onConnected();
    void onDisconnected();
    void onError();
    void on_SendButton_clicked();
    void on_SendButton_2_clicked();
    void on_SendButton_3_clicked();
    void on_RemoteExpBegin_clicked();
    void on_Remote_Left_clicked(); 
    void on_Remote_Right_clicked(); 
    void onSocketError(QAbstractSocket::SocketError socketError);
    void handleNewConnection();

public slots:
    void recv_data();

protected:
    //重载键盘触发事件 - 将按键与PushButton连接
    void keyPressEvent(QKeyEvent* event) override;

public:
    Ui::ExperDemoSystemClass ui;

    //Worker* m_worker;

    QThread* m_cameraThread;  //相机线程
    QThread* m_stageThread;   //位移台线程
    QThread* m_recordThread;  //文件记录线程

    bool m_cam_running;  //相机运行标记
    bool m_stage_running;  //位移台运行标记
    bool m_histogram_running;  //直方图线程运行标记
    bool m_record_running;   //文件记录线程

    bool Gui_Debug;   //调试
    bool ExpCanStart;

    //用于按键的连续触发
    QTimer* m_upButtonTimer;
    QTimer* m_downButtonTimer;
    QTimer* m_leftButtonTimer;
    QTimer* m_rightButtonTimer;

    //用于记录线程的循环
    QTimer* m_recordTimer;

    //储存数据的路径
    QString savePath;

    //tcp Socket
    QTcpSocket* m_socket;
    QTcpServer* m_server;
    bool socketStatus;
    void readFromServer(QString line);

    void captureImage();     //线程1
    void moveStage();     //线程2
    void recordData();    //线程3  记录实验数据

    /*char vChannel;
    char vDir;
    char vNum;*/
};

#endif // EXPERDEMOSYSTEM_H