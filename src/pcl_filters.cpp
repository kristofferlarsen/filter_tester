
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

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::visualize_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
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

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclFilters::cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>), outcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr incloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*incloud);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (distance);

    int i=0, nr_points = (int) incloud->points.size ();
    while (incloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (incloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (incloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *incloud = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (incloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (incloud);
    ec.extract (cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    for(int i = 0; i< cluster_indices.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*incloud,cluster_indices[i],*tmpcloud);
        clusters.push_back(tmpcloud);
    }
    filteredCloud = combine_clouds(clusters);
    return (clusters);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr incloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud,*incloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (incloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    filteredCloud = cloud;
    return (cloud_plane);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::get_filtered_cloud()
{
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PclFilters::color_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud,*rgb_cloud);

    //color all points white
    for (int i = 0; i< rgb_cloud->points.size(); i++)
    {
        rgb_cloud->points[i].r = r;
        rgb_cloud->points[i].g = g;
        rgb_cloud->points[i].b = b;
    }
    return (rgb_cloud);
}

pcl::PointCloud<pcl::Normal>::Ptr PclFilters::get_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal>);  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::combine_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cluster_cloud = *input.at(0);
    for (unsigned i=0; i<input.size(); i++){
        *cluster_cloud += *(pcl::PointCloud<pcl::PointXYZ>::Ptr) input.at(i);
    }
    return (cluster_cloud);

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr PclFilters::compute_cvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(object);
    cvfh.setInputNormals(normals);
    cvfh.setSearchMethod(kdtree);
    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);
    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(false);
    cvfh.compute(*descriptors);
    return (descriptors);
}
}


