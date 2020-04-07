#include "Point_Gui.h"
#include "vtkAutoInit.h"


Point_Gui::Point_Gui(QWidget* parent)
    : QMainWindow(parent) {
    ui.setupUi(this);
    this->setWindowTitle("|| dianyunPCL || PCL QT_demo");

    // Setup the cloud pointer
    cloud.reset(new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize(5000);

    // The default color
    red = 128;
    green = 128;
    blue = 128;

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }

    //Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
    ui.qvtkWidget->update();

    // Connect "random" button and the function
    connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(randomButtonPressed()));
    connect(ui.pushButton_2, SIGNAL(clicked()), this, SLOT(openfileButtonPressed()));

    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();
    ui.qvtkWidget->update();
}

Point_Gui::~Point_Gui() {
    delete ui.centralWidget;
}


void Point_Gui::randomButtonPressed() {
    printf("Random button was pressed\n");
    printf("Random button was pressed\n");

    // Set the new color
    for (size_t i = 0; i < cloud->size(); i++) {
        cloud->points[i].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
        cloud->points[i].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
        cloud->points[i].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
    }

    viewer->updatePointCloud(cloud, "cloud");
    ui.qvtkWidget->update();
}

void
Point_Gui::openfileButtonPressed() {
    viewer->removePointCloud("cloud");
    // You might want to change "/home/" if you're not on an *nix platform
    QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"),
                       "D://", tr("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
    std::cout << filename.toStdString().c_str() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);

    if (filename.isEmpty()) {
        return;
    }

    int return_status;

    if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
        return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
    } else {
        return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);
    }

    if (return_status != 0) {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
        return;
    }

    //// If point cloud contains NaN values, remove them before updating the visualizer point cloud
    //if (cloud_tmp->is_dense) {
    //    pcl::copyPointCloud(*cloud_tmp, *cloud_);
    //} else {
    //    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    //    std::vector<int> vec;
    //    pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
    //}

    /*
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_);
    sor.setLeafSize (0.05f,0.05f,0.05f);
    sor.filter (*cloud_);
    */
    pcl::copyPointCloud(*cloud_tmp, *cloud_);
    std::cout << cloud_tmp->points.size() << std::endl;
    viewer->addPointCloud(cloud_, "cloud");
    viewer->resetCamera();
    ui.qvtkWidget->update();
}