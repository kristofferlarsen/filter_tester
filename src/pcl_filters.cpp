
#include "../include/qt_filter_tester/pcl_filters.hpp"

namespace qt_filter_tester{

PclFilters::PclFilters(QObject *parent):
    QObject(parent)
{

}

PclFilters::~PclFilters() {}

/*
 * Returns a visualizer based on the input pointcloud
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->initCameraParameters ();
    return (viewer);
}

/*
 * returns a visualizer based on the input pointcloud and normals cloud
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::normalsVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, get_normals(cloud), 10, 0.05, "normals");
    viewer->initCameraParameters ();
    filteredCloud = cloud;
    return (viewer);
}

/*
 * Return a viewer with a passthrough filtered pointcloud
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::passthrough_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis)
{
    return(visualize(passthrough(cloud,min,max,axis)));
}

/*
 * Returns the last filtered pointcloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::get_filtered_cloud()
{
    return filteredCloud;
}

/*
 * Returns a pointcloud of normals corresponding to the input cloud
 */
pcl::PointCloud<pcl::Normal>::Ptr PclFilters::get_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr)
{

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    passfilter.setInputCloud(cloud);
    passfilter.setFilterFieldName(axis);
    passfilter.setFilterLimits(min,max);
    //passfilter.setKeepOrganized(true);
    passfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}
}


