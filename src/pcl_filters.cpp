
#include "../include/qt_filter_tester/pcl_filters.hpp"

namespace qt_filter_tester{

PclFilters::PclFilters(QObject *parent):
    QObject(parent)
{

}

PclFilters::~PclFilters() {}




int PclFilters::recognizePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    ObjectModel inputModel;

    inputModel.points = input;
    inputModel.normals = get_normals(inputModel.points,0.05);

    inputModel.global_descriptors = calculate_vfh_descriptors(inputModel.points,inputModel.normals);
    std::vector<int> nn_index(1);
    std::vector<float> nn_sqr_distance(1);
    kdtree_->nearestKSearch (inputModel.global_descriptors->points[0],1,nn_index,nn_sqr_distance);
    int best_match = nn_index[0];
    std::cout << "nn_sqr_distance: " << nn_sqr_distance[0] << std::endl;
    return (best_match);
}

std::vector<ObjectModel> PclFilters::populate_models(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds)
{
    std::vector<ObjectModel> models;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_(new pcl::PointCloud<pcl::VFHSignature308>);

    for(int i = 0; i< clouds.size(); i++){
        ObjectModel tmpModel;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud = clouds.at(i);
        tmpModel.points = tmpcloud;
        std::cout << "Cloud nr: " << i << std::endl;
        std::cout << "Calculating normals" << std::endl;
        tmpModel.normals = get_normals(tmpModel.points,0.05);
        //std::cout << "Calculating keypoints" <<  std::endl;
        //tmpModel.keypoints = calculate_keypoints(tmpModel.points,0.01,10,8,0.0);
        //std::cout << "Calculating local descriptors" << std::endl;
        //tmpModel.local_descriptors = calculate_local_descritor(tmpModel.points,tmpModel.normals,tmpModel.keypoints,0.1);
        std::cout << "Calculating global descriptors" << std::endl;
        tmpModel.global_descriptors = calculate_vfh_descriptors(tmpModel.points,tmpModel.normals);
        std::cout << "pushing model to array" << std::endl;
        models.push_back(tmpModel);
        *descriptors_ += *(tmpModel.global_descriptors);
    }

    kdtree_ = pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr (new pcl::KdTreeFLANN<pcl::VFHSignature308>);
    kdtree_->setInputCloud (descriptors_);
    return (models);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::calculate_keypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                    float min_scale,
                                                                    int nr_octaves,
                                                                    int nr_scales_per_octave,
                                                                    float min_contrast)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud,*rgbcloud);
    for(int i = 0; i< rgbcloud->size(); i++){
        rgbcloud->points[i].r = 255;
        rgbcloud->points[i].g = 255;
        rgbcloud->points[i].b = 255;
    }
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
    sift_detect.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast (min_contrast);
    sift_detect.setInputCloud (rgbcloud);
    pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
    sift_detect.compute (keypoints_temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (keypoints_temp, *keypoints);

    return (keypoints);
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr PclFilters::calculate_local_descritor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                                                 pcl::PointCloud<pcl::Normal>::Ptr normal,
                                                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                                                                                 float feature_radius)
{
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setNumberOfThreads(8);
    fpfh_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    fpfh_estimation.setRadiusSearch (feature_radius);
    fpfh_estimation.setSearchSurface (cloud);
    fpfh_estimation.setInputNormals (normal);
    fpfh_estimation.setInputCloud (keypoints);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_estimation.compute (*local_descriptors);

    return (local_descriptors);

}

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

boost::shared_ptr<pcl::visualization::PCLVisualizer> PclFilters::visualize_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int numOfNormals)
{
    viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, get_normals(cloud,radius), numOfNormals, 0.05, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
    viewer->initCameraParameters ();
    filteredCloud = cloud;
    return (viewer);
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
    ec.setMinClusterSize (500);
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
    norm_est.setNumberOfThreads(8);
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch (radius);
    norm_est.setInputCloud (cloud);
    //norm_est.setSearchSurface (cloud);
    norm_est.compute (*normals_out);
    return (normals_out);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    passfilter.setKeepOrganized(true);
    passfilter.setInputCloud(cloud);
    passfilter.setFilterFieldName(axis);
    passfilter.setFilterLimits(min,max);
    passfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    voxelfilter.setInputCloud(cloud);
    voxelfilter.setLeafSize(lx,ly,lz);
    voxelfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::median_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    medianfilter.setInputCloud(cloud);
    medianfilter.setWindowSize(window_size);
    medianfilter.setMaxAllowedMovement(max_allowed_movement);
    medianfilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::shadowpoint_removal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    shadowpoint_filter.setInputCloud(cloud);
    shadowpoint_filter.setKeepOrganized(true);
    shadowpoint_filter.setThreshold(threshold);
    shadowpoint_filter.setNormals(get_normals(cloud, radius));
    shadowpoint_filter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    statistical_outlier.setKeepOrganized(true);
    statistical_outlier.setInputCloud(cloud);
    statistical_outlier.setMeanK(meanK);
    statistical_outlier.setStddevMulThresh(std_deviation_threshold);
    statistical_outlier.filter(*cloud_filtered);
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

pcl::PointCloud<pcl::VFHSignature308>::Ptr PclFilters::compute_cvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals = get_normals(cloud,0.01);
    // CVFH estimation object.

    pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;

    cvfh.setInputCloud(cloud);

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

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilters::bilateral_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sigmaS, double sigmaR)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::FastBilateralFilterOMP<pcl::PointXYZ> bifilter;
    bifilter.setInputCloud(cloud);
    bifilter.setSigmaR(sigmaR);
    bifilter.setSigmaS(sigmaS);
    bifilter.filter(*cloud_filtered);
    filteredCloud = cloud_filtered;
    return (cloud_filtered);
}

std::vector<RayTracedCloud_descriptors> PclFilters::get_descriptors(std::vector<RayTraceCloud> inclouds)
{
    //ray_trace_loader->setCloudResolution(200);
    std::vector<RayTracedCloud_descriptors> defined_clouds;

    for(int i = 0; i< inclouds.size(); i++){
        RayTracedCloud_descriptors tmp;
        RayTraceCloud current = inclouds.at(i);
        tmp.cloud = current.cloud;
        tmp.enthropy = current.enthropy;
        tmp.pose = current.pose;
        tmp.descriptor = compute_cvfh_descriptors(tmp.cloud);
        defined_clouds.push_back(tmp);
    }
    return (defined_clouds);
}

pcl::PointCloud<pcl::ESFSignature640>::Ptr PclFilters::calculate_esf_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud);
    esf.compute(*descriptor);
    return (descriptor);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr PclFilters::calculate_ourcvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal){

    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;

    ourcvfh.setInputCloud(cloud);

    ourcvfh.setInputNormals(normal);

    ourcvfh.setSearchMethod(kdtree);

    ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.

    ourcvfh.setCurvatureThreshold(0.1);

    ourcvfh.setNormalizeBins(false);

    // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
    // this will decide if additional Reference Frames need to be created, if ambiguous.
    ourcvfh.setAxisRatio(0.8);

    ourcvfh.compute(*descriptors);

    return (descriptors);
}

pcl::PointCloud<pcl::GFPFHSignature16>::Ptr PclFilters::calculate_gfpfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZL>::Ptr object(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::GFPFHSignature16>::Ptr descriptor(new pcl::PointCloud<pcl::GFPFHSignature16>);

    pcl::copyPointCloud(*cloud,*object);

    // Note: you should now perform classification on the cloud's points. See the
    // original paper for more details. For this example, we will now consider 4
    // different classes, and randomly label each point as one of them.
    for (size_t i = 0; i < object->points.size(); ++i)
    {
        object->points[i].label = 1 + i % 4;
    }

    pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
    gfpfh.setInputCloud(object);
    gfpfh.setInputLabels(object);
    gfpfh.setOctreeLeafSize(0.01);
    gfpfh.setNumberOfClasses(4);
    gfpfh.compute(*descriptor);

    return (descriptor);
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr PclFilters::calculate_vfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh_estimation;
    vfh_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    vfh_estimation.setInputCloud (points);
    vfh_estimation.setInputNormals (normals);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor (new pcl::PointCloud<pcl::VFHSignature308>);
    vfh_estimation.compute (*global_descriptor);

    return (global_descriptor);
}

}


