

#ifndef qt_filter_tester_PCLFILTERS_H
#define qt_filter_tester_PCLFILTERS_H

#include <QObject>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>



namespace qt_filter_tester {

class PclFilters : public QObject{
Q_OBJECT

public:
    PclFilters(QObject *parent = 0);
    ~PclFilters();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> passthrough_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> median_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> voxelgrid_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> shadowpoint_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> statistical_outlier_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_cloud();
    pcl::PointCloud<pcl::Normal>::Ptr get_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr median (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);
    pcl::PointCloud<pcl::PointXYZ>::Ptr shadowpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);


private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
    pcl::PassThrough<pcl::PointXYZ> passfilter;
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;
    pcl::MedianFilter<pcl::PointXYZ> medianfilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_Outlier_filter;

public Q_SLOTS:

Q_SIGNALS:

};
}
#endif
