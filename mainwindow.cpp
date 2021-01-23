#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QGridLayout>

QByteArray data_camera(cv::Mat intrinsic_matrix,cv::Mat R, cv::Mat n,float d, cv::Mat T){
    QJsonObject dataObject;
    QJsonArray intrinsic_matrixArray;
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(0,0)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(0,1)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(0,2)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(1,0)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(1,1)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(1,2)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(2,0)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(2,1)));
    intrinsic_matrixArray.push_back (QJsonValue::fromVariant(intrinsic_matrix.at<double>(2,2)));

    dataObject.insert ("intrinsic_matrix",intrinsic_matrixArray);
    //

    QJsonArray RArray;
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(0,0)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(0,1)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(0,2)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(1,0)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(1,1)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(1,2)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(2,0)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(2,1)));
    RArray.push_back (QJsonValue::fromVariant(R.at<double>(2,2)));

    dataObject.insert ("R",RArray);

    //

    QJsonArray TArray;
    TArray.push_back (QJsonValue::fromVariant(T.at<double>(0,0)));
    TArray.push_back (QJsonValue::fromVariant(T.at<double>(1,0)));
    TArray.push_back (QJsonValue::fromVariant(T.at<double>(2,0)));

    dataObject.insert ("T",TArray);

    QJsonArray nArray;
    nArray.push_back (QJsonValue::fromVariant(n.at<double>(0,0)));
    nArray.push_back (QJsonValue::fromVariant(n.at<double>(0,1)));
    nArray.push_back (QJsonValue::fromVariant(n.at<double>(0,2)));

    dataObject.insert ("n",nArray);


    dataObject.insert ("d",QJsonValue::fromVariant(d));


    QJsonDocument doc(dataObject);
    return doc.toJson ();
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)

{
    ui->setupUi(this);

    calib = new Calib();

    capture = new Capture();

    initGraph3D();


    for (const QCameraInfo &cameraInfo : QCameraInfo::availableCameras()) {
        ui->listCamera->addItem (cameraInfo.description ());
    }

    for(const QSerialPortInfo &info : QSerialPortInfo::availablePorts ())
        ui->listSerial->addItem(info.portName ());

    ui->checkBox->setEnabled (false);
    ui->checkBox_2->setEnabled (false);
    ui->checkBox_3->setEnabled (false);

}

MainWindow::~MainWindow()
{
    delete calib;
    delete capture;
    delete ui;
}

void MainWindow::on_export_file_clicked()
{
    QFileDialog dialog(this);
    dialog.setViewMode(QFileDialog::Detail);
    dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                            "/",
                                            QFileDialog::ShowDirsOnly
                                                | QFileDialog::DontResolveSymlinks);
    QDir address;
    QString fileAdd = dir;
    QString fileName = ui->lineEdit->text (),format = ".ply";
    fileName.append (format);
    fileAdd.append ("/");
    fileAdd.append (fileName);
    qDebug() << fileAdd;
    std::ofstream file(fileAdd.toLocal8Bit().constData());
    exportfile = new Exportfile(capture->points_final,fileAdd);
    connect (exportfile,&Exportfile::finished,exportfile,&QObject::deleteLater);
    exportfile->start ();
}

void MainWindow::on_port_clicked()
{
    m_serial = new QSerialPort;
    if(!m_serial->isOpen ()){
        m_serial->setPortName (ui->listSerial->currentText ());
        m_serial->setBaudRate (QSerialPort::Baud9600);
        m_serial->setDataBits (QSerialPort::Data8);
        m_serial->setStopBits (QSerialPort::OneStop);
        m_serial->setParity (QSerialPort::NoParity);
        m_serial->setFlowControl (QSerialPort::NoFlowControl);
        m_serial->open(QIODevice::ReadWrite);
        ui->label_2->setText ("Kết nối");
        ui->checkBox->setChecked (true);
        connect (m_serial, &QSerialPort::readyRead,this,&MainWindow::read);
    }
    else{
        ui->label_2->setText ("Chưa kết nối");
    }

}

void MainWindow::on_camera_clicked()
{
    calib->cap.open (ui->listCamera->currentIndex ());
    calib->cap.set(3,1920);
    calib->cap.set(4,1080);
    calib->cap.set(CV_CAP_PROP_EXPOSURE,ui->exposure->value ()/10.0);
    calib->cap.set (CV_CAP_PROP_AUTOFOCUS,false);
    calib->cap.set (CV_CAP_PROP_FOCUS,16);
    ui->checkBox_2->setChecked (true);
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
    timer->start(30);
}

void MainWindow::update_window()
{
    calib->cap.read (image);
    QImage qt_image = QImage((const unsigned char*) (image.data), image.cols, image.rows, QImage::Format_BGR888);
    qt_image = qt_image.scaled(ui->label_pic->width(),ui->label_pic->height (),Qt::IgnoreAspectRatio);
    ui->label_pic->setPixmap(QPixmap::fromImage(qt_image));
}

void MainWindow::on_Calib_clicked()
{
    connect (calib,&Calib::remain_camera,this,&MainWindow::slot_remain_camera);
    connect (calib,&Calib::complete_camera,this,&MainWindow::slot_complete_camera);
    connect (calib,&Calib::begin_camera,this,&MainWindow::slot_begin_camera);
    connect (calib,&Calib::control9,this,&MainWindow::slot_control9);



    connect (calib,SIGNAL(control5L()),this,SLOT(motor5L()));
    connect (calib,&Calib::begin_laser,this,&MainWindow::slot_begin_laser);
    connect (calib,SIGNAL(complete_laser()),this,SLOT(slot_complete_laser()));
    connect (calib,SIGNAL(laser_on()),this,SLOT(slot_laser_on()));
    connect (calib,SIGNAL(laser_off()),this,SLOT(slot_laser_off()));



    connect (calib,SIGNAL(begin_turntable()),this,SLOT(slot_begin_turntable()));
    connect (calib,SIGNAL(control_3turntable()),this,SLOT(slot_control_3turntable()));
    connect (calib,SIGNAL(complete_turntable()),this,SLOT(slot_complete_turntable()));

    connect (calib,&Calib::finished,calib,&Calib::deleteLater);
    calib->start ();
}







void MainWindow::on_capture_clicked()
{
    connect (capture,SIGNAL(c_laser_on()),this,SLOT(slot_laser_on()));
    connect (capture,SIGNAL(c_laser_off()),this,SLOT(slot_laser_off()));
    connect (capture,SIGNAL(control45_capture()),this,SLOT(slot_capture()));
    connect (capture,&Capture::remain_capture,this,&MainWindow::slot_remain_camera);

    //connect (capture,SIGNAL(complete_capture()),this,SLOT(()));
    connect (capture,&Capture::transmit_point_cloud,this,&MainWindow::addPoint);

    capture->start ();
}

void MainWindow::update_window1()
{
    capture->cap.read (image);
    cv::Point2i p1(ui->horizontal_left->value(),-ui->vertical_bottom->value()),
                p2(1919-ui->horizontal_left->value(),-ui->vertical_bottom->value()),
                p3(ui->horizontal_left->value(),-ui->vertical_top->value()),
                p4(1919-ui->horizontal_left->value(),-ui->vertical_top->value());
    std::vector<cv::Point2i> points{p1,p3,p4,p2};
    capture->area_points = points;

    cv::rectangle (image,p1,p4,cv::Scalar(0,255,255),4);

    QImage qt_image = QImage((const unsigned char*) (image.data), image.cols, image.rows, QImage::Format_BGR888);
    qt_image = qt_image.scaled(ui->label_pic->width(),ui->label_pic->height (),Qt::IgnoreAspectRatio);
    ui->label_pic->setPixmap(QPixmap::fromImage(qt_image));
}



void MainWindow::on_test_clicked()
{
    capture->cap.open (ui->listCamera->currentIndex ());
    capture->cap.set(3,1920);
    capture->cap.set(4,1080);
    capture->cap.set(CV_CAP_PROP_EXPOSURE,ui->exposure->value ()/10.0);
    capture->cap.set (CV_CAP_PROP_AUTOFOCUS,false);
    capture->cap.set (CV_CAP_PROP_FOCUS,16);
    timer1 = new QTimer();
    connect(timer1, SIGNAL(timeout()), this, SLOT(update_window1()));
    timer1->start(30);
}




/*
 *
 *
 * ----------------------  Connected Port control ----------------------------------------------------
 *
 *
 *
 */

// --------------------------  Camera  -----------------------------------------------------

void MainWindow::slot_remain_camera(int count)
{
    if(count==0){
        ui->label_calib->setText ("Done");
    }
    else{
        ui->label_calib->setText (QString::number (count));
    }
}
void MainWindow::slot_complete_camera()
{
    capture->intrinsic_matrix = calib->intrinsic_matrix;
    capture->distortion_coefficients = calib->distortion_coefficients;
}

void MainWindow::slot_begin_camera()
{
    m_serial->write ("6");
}

void MainWindow::slot_control9()
{
    m_serial->write ("9");
}
// -------------------------- Laser ----------------------------------------------------------

void MainWindow::motor5L()
{
    m_serial->write ("3");
}

void MainWindow::slot_begin_laser()
{
    m_serial->write ("8");
}

void MainWindow::slot_laser_off()
{
    m_serial->write ("0");
}

void MainWindow::slot_laser_on()
{
    m_serial->write ("1");
}

void MainWindow::slot_complete_laser()
{
    capture->d = calib->d;
    capture->n = calib->n;
}


// ------------------------------- Turntable ---------------------------------------------------
void MainWindow::slot_begin_turntable()
{
    m_serial->write ("4");
}

void MainWindow::slot_complete_turntable()
{
    ui->checkBox_3->setChecked (true);
    capture->R = calib->R;
    capture->T = calib->T;
    delete timer;
    calib->cap.release ();
    QPixmap nullPix(0,0);
    ui->label_pic->setPixmap(nullPix);
    //calib->quit ();
}

void MainWindow::slot_control_3turntable()
{
    m_serial->write ("5");
}

// ----------------------------------- Capture --------------------------------------------------

void MainWindow::slot_capture()
{
    m_serial->write ("2");
}

void MainWindow::read()
{

}




// OpenGL


void MainWindow::initGraph3D()
{
    ui->widget->setLayout (new QVBoxLayout);
    scatter = new Q3DScatter();

    graphContainer = QWidget::createWindowContainer(scatter);
    ui->widget->layout()->addWidget(graphContainer);
    //´´½¨×ø±êÖá

    scatter->axisX()->setTitle("axis X");
    scatter->axisX()->setTitleVisible(true);

    scatter->axisY()->setTitle("axis Y");
    scatter->axisY()->setTitleVisible(true);

    scatter->axisZ()->setTitle("axis Z");
    scatter->axisZ()->setTitleVisible(true);

    scatter->activeTheme()->setLabelBackgroundEnabled(false);
    scatter->setAspectRatio(1.0);
    scatter->setHorizontalAspectRatio(1.0);
    scatter->setShadowQuality (QAbstract3DGraph::ShadowQualityNone);
    //
    proxy = new QScatterDataProxy();
    ser = new QScatter3DSeries(proxy);
    ser->setItemLabelFormat("(@xLabel, @yLabel @zLabel)");
    ser->setMesh(QAbstract3DSeries::MeshPoint);
    ser->setItemSize(0.05);
    scatter->addSeries(ser);
    std::vector<cv::Point3f> pts;
    pts.push_back (cv::Point3f(0,0,0));
    addPoint (pts);
}

void MainWindow::addPoint(std::vector<cv::Point3f> point)
{
    int itemCount = point.size ();

    QScatterDataArray *da = new QScatterDataArray();
    da->resize(itemCount);
    QScatterDataItem *item = &da->first();

    for(auto it:point){
        item->setPosition(QVector3D(it.x, it.y, it.z));
        item++;
    }
    proxy->resetArray(da);

}


void MainWindow::on_actionData_camera_triggered()
{
    QFileDialog dialog(this);
    dialog.setViewMode(QFileDialog::Detail);
    dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/",
                                                    QFileDialog::ShowDirsOnly
                                                        | QFileDialog::DontResolveSymlinks);
    QString namefile{"/data.json"};
    QString nameDir{dir};
    nameDir.append (namefile);
    std::ofstream createFile(nameDir.toLocal8Bit().constData());
    QFile file(nameDir);
    if(!file.open (QFile::WriteOnly | QFile::Text)){

    }
    QTextStream out(&file);
    out << data_camera (capture->intrinsic_matrix,capture->R,capture->n, capture->d,capture->T);
    file.flush ();
    file.close ();

}

void MainWindow::on_scan_fast_clicked()
{
    QFileDialog dialog(this);
    dialog.setViewMode(QFileDialog::Detail);
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Data file"), "/",tr("Images (*.json *.yaml)"));
    QFile file;
    file.setFileName (fileName);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QByteArray val;

    val = file.readAll ();

    QJsonDocument doc = QJsonDocument::fromJson(val);
    QJsonObject sett2 = doc.object();
    QJsonArray intrinsic_matrix = sett2["intrinsic_matrix"].toArray();
    QJsonArray R = sett2["R"].toArray();
    QJsonArray n = sett2["n"].toArray();
    QJsonArray T = sett2["T"].toArray();
    QJsonValue d = sett2["d"];

    capture->intrinsic_matrix = (cv::Mat_<double>(3,3)<< intrinsic_matrix[0].toDouble (),
                                                         intrinsic_matrix[1].toDouble (),
                                                         intrinsic_matrix[2].toDouble (),
                                                         intrinsic_matrix[3].toDouble (),
                                                         intrinsic_matrix[4].toDouble (),
                                                         intrinsic_matrix[5].toDouble (),
                                                         intrinsic_matrix[6].toDouble (),
                                                         intrinsic_matrix[7].toDouble (),
                                                         intrinsic_matrix[8].toDouble () );
    capture->R = (cv::Mat_<double>(3,3)<< R[0].toDouble (),
                                 R[1].toDouble (),
                                 R[2].toDouble (),
                                 R[3].toDouble (),
                                 R[4].toDouble (),
                                 R[5].toDouble (),
                                 R[6].toDouble (),
                                 R[7].toDouble (),
                                 R[8].toDouble () );
    capture->n = (cv::Mat_<double>(1,3)<< n[0].toDouble (),
                                 n[1].toDouble (),
                                 n[2].toDouble ());

    capture->T = (cv::Mat_<double>(3,1)<< T[0].toDouble (),
                  T[1].toDouble (),
                  T[2].toDouble ());
    capture->d = d.toDouble ();
    ui->checkBox_3->setChecked (true);
}

void MainWindow::on_Brightness_valueChanged(int value)
{
    calib->cap.set(CV_CAP_PROP_BRIGHTNESS,value);
    capture->cap.set(CV_CAP_PROP_BRIGHTNESS,value);
}

void MainWindow::on_saturation_valueChanged(int value)
{
    capture->cap.set(CV_CAP_PROP_CONTRAST,value);
    calib->cap.set(CV_CAP_PROP_CONTRAST,value);
}

void MainWindow::on_Contrast_valueChanged(int value)
{
    capture->cap.set(CV_CAP_PROP_SATURATION,value);
    calib->cap.set(CV_CAP_PROP_SATURATION,value);
}

void MainWindow::on_exposure_valueChanged(int value)
{
    double valueTemp = value/10.0;
    capture->cap.set(CV_CAP_PROP_EXPOSURE,valueTemp);
    calib->cap.set(CV_CAP_PROP_EXPOSURE,valueTemp);
}



void MainWindow::on_checkBox_3_stateChanged(int arg1)
{
    if(ui->checkBox_3->isChecked ()&&ui->checkBox_2->isChecked ()&&ui->checkBox->isChecked ()){
        ui->isOK->setText ("OK");
    }
    else{
        ui->isOK->setText("NOT OK");
    }
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if(ui->checkBox_3->isChecked ()&&ui->checkBox_2->isChecked ()&&ui->checkBox->isChecked ()){
        ui->isOK->setText ("OK");
    }
    else{
        ui->isOK->setText("NOT OK");
    }
}

void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    if(ui->checkBox_3->isChecked ()&&ui->checkBox_2->isChecked ()&&ui->checkBox->isChecked ()){
        ui->isOK->setText ("OK");
    }
    else{
        ui->isOK->setText("NOT OK");
    }
}
bool status {false};
void MainWindow::on_actionLaser_triggered()
{
    status = !status;
    if(status){
        m_serial->write ("1");
    }
    else{
        m_serial->write ("0");
    }
}
