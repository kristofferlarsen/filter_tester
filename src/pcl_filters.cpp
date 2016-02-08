
#include "../include/qt_filter_tester/pcl_filters.hpp"

namespace qt_filter_tester{

PclFilters::PclFilters(QObject *parent):
    QObject(parent)
{

}

PclFilters::~PclFilters() {}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::normalsVis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int numOfNormals)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, get_normals(cloud,radius), numOfNormals, 0.05, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
    viewer->initCameraParameters ();
    filteredCloud = cloud;
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::passthrough_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis)
{
    return (visualize(passthrough(cloud,min,max,axis)));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::median_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement)
{
    return (visualize(median(cloud,window_size,max_allowed_movement)));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::voxelgrid_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz)
{
    return (visualize(voxelgrid(cloud,lx,ly,lz)));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::shadowpoint_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius)
{
    return (visualize(shadowpoint(cloud, threshold, radius)));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::statistical_outlier_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold)
{
    return (visualize(statistical_outlier(cloud,meanK,std_deviation_threshold)));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::get_filtered_cloud()
{
    return filteredCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr PclFilters::get_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal>);  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch (radius);
    norm_est.setInputCloud (cloud);
    //norm_est.setSearchSurface (cloud);
    norm_est.compute (*normals_out);
    return (normals_out);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    passfilter.setInputCloud(cloud);
    passfilter.setFilterFieldName(axis);
    passfilter.setFilterLimits(min,max);
    passfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::voxelgrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    voxelfilter.setInputCloud(cloud);
    voxelfilter.setLeafSize(lx,ly,lz);
    voxelfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::median(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    medianfilter.setInputCloud(cloud);
    medianfilter.setWindowSize(window_size);
    medianfilter.setMaxAllowedMovement(max_allowed_movement);
    medianfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::shadowpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    shadowpoint_filter.setInputCloud(cloud);
    shadowpoint_filter.setThreshold(threshold);
    shadowpoint_filter.setNormals(get_normals(cloud, radius));
    shadowpoint_filter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::statistical_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //statistical_outlier_filter.setKeepOrganized(true);
    statistical_outlier_filter.setInputCloud(cloud);
    statistical_outlier_filter.setMeanK(meanK);
    statistical_outlier_filter.setStddevMulThresh(std_deviation_threshold);
    statistical_outlier_filter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}
}


