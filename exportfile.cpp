#include "exportfile.h"
#include <QDebug>

Exportfile::Exportfile(std::vector<cv::Point3f> points_cloud,QString address):points_cloud{points_cloud},fileAdress{address}
{

}

void Exportfile::save_points_cloud(std::vector<cv::Point3f> points_cloud)
{
    QString lenPoint = QString::number (points_cloud.size());
    header = "ply\n";
    header += "format ascii 1.0\n";
    header += "comment Code by Ha\n";
    header += "element vertex ";
    header += lenPoint;
    header += "\n";
    header += "property float32 x\n";
    header += "property float32 y\n";
    header += "property float32 z\n";
    header += "element face 0\n";
    header += "property list uchar int vertex_indices\n";
    header += "end_header\n";
    for(cv::Point3f &it:points_cloud){
        header += QString::number(it.x) + " " + QString::number(it.y) + " " + QString::number(it.z) + "\n";

    }
}

void Exportfile::run()
{
    QFile file(fileAdress);
    if(!file.open (QFile::WriteOnly | QFile::Text)){

    }
    QTextStream out(&file);
    save_points_cloud(points_cloud);
    out << header;
    file.flush ();
    file.close ();
}
