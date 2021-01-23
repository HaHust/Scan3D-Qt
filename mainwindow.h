#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <calib.h>
#include <capture.h>
#include <QSerialPort>
#include <QCameraInfo>
#include <QSerialPortInfo>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <point_cloud.h>
#include <exportfile.h>
#include <QTimer>
#include <QtDataVisualization>
#include <QMetaType>
#include <QJsonObject>


using namespace QtDataVisualization;
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
Q_DECLARE_METATYPE(std::vector<cv::Point3f>)

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    Calib *calib;
    Capture *capture;
    Exportfile *exportfile;
    QSerialPort *m_serial;
    QTimer *timer;
    QTimer *timer1;
    cv::Mat image;
    Camera cam;
    std::vector<cv::Point3f> pts;
    QString dir;
    // point cloud opengl

    QWidget *graphContainer;
    QScatterDataProxy *proxy;
    Q3DScatter *scatter;

    QScatter3DSeries *ser;

    void initGraph3D();

    float brightness, contrast, saturation;


private slots:
    void on_Calib_clicked();

    void on_port_clicked();

    void update_window();

    void update_window1();

    void on_camera_clicked();

    void on_export_file_clicked();

    void on_capture_clicked();

    void addPoint(std::vector<cv::Point3f>);

    void on_test_clicked();

    void on_actionData_camera_triggered();

    void on_scan_fast_clicked();

    void on_Brightness_valueChanged(int value);

    void on_saturation_valueChanged(int value);

    void on_Contrast_valueChanged(int value);

    void on_exposure_valueChanged(int value);

    void on_checkBox_3_stateChanged(int arg1);

    void on_checkBox_stateChanged(int arg1);

    void on_checkBox_2_stateChanged(int arg1);

    void on_actionLaser_triggered();

public slots:
    void slot_begin_camera();
    void slot_control9();
    void slot_remain_camera(int);
    void slot_complete_camera();

    void motor5L();
    void slot_begin_laser();
    void slot_laser_off();
    void slot_laser_on();
    void slot_complete_laser();

    void slot_begin_turntable();
    void slot_complete_turntable();
    void slot_control_3turntable();


    void slot_capture();

    void read();
signals:

private:
    Ui::MainWindow *ui;
};



#endif // MAINWINDOW_H
