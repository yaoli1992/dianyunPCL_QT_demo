#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkVersion.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <QtWidgets/QMainWindow>
#include "ui_Point_Gui.h"

#include <iostream>

// Qt
#include <QMainWindow>
#include <QFileDialog>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Point_Gui : public QMainWindow {
    Q_OBJECT

public:
    Point_Gui(QWidget* parent = Q_NULLPTR);
    /** @brief Destructor */
    ~Point_Gui();

public Q_SLOTS:
    void  randomButtonPressed();
    void  openfileButtonPressed();
protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;

    unsigned int red;
    unsigned int green;
    unsigned int blue;
    QString filepath;
private:
    Ui::Point_GuiClass ui;
};
