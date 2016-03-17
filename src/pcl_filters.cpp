
#include "../include/qt_filter_tester/pcl_filters.hpp"

namespace qt_filter_tester{

PclFilters::PclFilters(QObject *parent):
    QObject(parent)
{

}

PclFilters::~PclFilters() {}


int PclFilters::search_for_model(std::vector<RayTraceCloud> clusters,
                                 std::vector<RayTraceCloud> model){

    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree = generate_search_tree(model);
    float min_distance = 10000;
    int correct_cluster;
    std::vector<float> search_result;
    for (int i = 0; i < clusters.size(); i++){
        search_result = match_cloud(clusters.at(i),search_tree);
        if(search_result[1] < min_distance){
            min_distance = search_result[1];
            correct_cluster = i;
        }
    }
    std::cout << "Minimum distance found: " << min_distance << std::endl;
    return correct_cluster;
}

void PclFilters::ransac_recognition(std::vector<RayTraceCloud> models,
                                    RayTraceCloud object)
{
    pcl::recognition::ObjRecRANSAC recognition(40.0,5.0);
    std::list<pcl::recognition::ObjRecRANSAC::Output> matchingList;
    for(int i = 0; i< models.size(); i++){
        QString name = "model_";
        name.append(QString::number(i));
        recognition.addModel(*models.at(i).cloud,*models.at(i).normals,name.toStdString());
    }
    recognition.recognize(*models.at(0).cloud,*models.at(0).normals,matchingList,0.99);

    std::cout << "size of list: " << matchingList.size() << std::endl;
}

Eigen::Matrix4f PclFilters::calculateInitialAlignment(RayTraceCloud source, RayTraceCloud target, float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac_ia;
    sac_ia.setMinSampleDistance(min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia.setMaximumIterations(nr_iterations);

    sac_ia.setInputSource(source.keypoints);
    sac_ia.setSourceFeatures(source.local_descriptors);

    sac_ia.setInputTarget(target.keypoints);
    sac_ia.setTargetFeatures(target.local_descriptors);

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia.align(registration_output);

    return (sac_ia.getFinalTransformation());
}

Eigen::Matrix4f PclFilters::calculateRefinedAlignment(RayTraceCloud source, RayTraceCloud target, Eigen::Matrix4f initial_alignment, float max_correspondence_distance, float outlier_rejection_threshold, float transformation_epsilon, float eucledian_fitness_epsilon, int max_iterations)
{
   pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
   icp.setMaxCorrespondenceDistance(max_correspondence_distance);
   icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
   icp.setTransformationEpsilon(transformation_epsilon);
   icp.setEuclideanFitnessEpsilon(eucledian_fitness_epsilon);
   icp.setMaximumIterations(max_iterations);

   pcl::PointCloud<pcl::PointXYZ>::Ptr source_transformed (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::transformPointCloud(*source.cloud,*source_transformed,initial_alignment);

   icp.setInputSource(source_transformed);
   icp.setInputTarget(target.cloud);

   pcl::PointCloud<pcl::PointXYZ> registration_output;
   icp.align(registration_output);
   return (icp.getFinalTransformation() * initial_alignment);
}



RayTraceCloud PclFilters::calculate_features(RayTraceCloud inputcloud)
{
    //Calculate normals, keypoints, local and global descriptor and return the model

    inputcloud.normals = get_normals(inputcloud.cloud,0.05);
    inputcloud.keypoints = calculate_keypoints(inputcloud.cloud,0.001,3,3,0.0);
    inputcloud.local_descriptors = calculate_local_descritor(inputcloud.cloud,inputcloud.normals,inputcloud.keypoints,0.15);
    inputcloud.global_descriptors = calculate_vfh_descriptors(inputcloud.cloud,inputcloud.normals);
    return (inputcloud);
}

pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr PclFilters::generate_search_tree(std::vector<RayTraceCloud> models)
{
    //take global descriptors and put them in a kdtree
    pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor(new pcl::PointCloud<pcl::VFHSignature308>);
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree(new pcl::KdTreeFLANN<pcl::VFHSignature308>);
    for(int i = 0; i< models.size(); i++){

        RayTraceCloud model = models.at(i);
        *global_descriptor += *(model.global_descriptors);
    }
    search_tree->setInputCloud(global_descriptor);
    //std::cout << "Size of tree: " << global_descriptor->size() << std::endl;
    return (search_tree);
}

std::vector<float> PclFilters::match_cloud(RayTraceCloud object_model,
                            pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree){
    std::vector<float> returnvalues;
    std::vector<int> best_match(1);
    std::vector<float> square_distance(1);
    //search_tree->nearestKSearch (object_model.global_descriptors->points[0],1,best_match,square_distance);
    search_tree->nearestKSearch (object_model.global_descriptors->points[0],1,best_match,square_distance);
    returnvalues.push_back(best_match[0]);
    returnvalues.push_back(square_distance[0]);
    return (returnvalues);
}

std::vector<float> PclFilters::temp_matching_cvfh(RayTraceCloud object_model,
                            pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr search_tree){
    std::vector<float> returnvalues;
    std::vector<int> best_match(1);
    std::vector<float> square_distance(1);
    //search_tree->nearestKSearch (object_model.global_descriptors->points[0],1,best_match,square_distance);
    int nr_of_descriptors = object_model.global_descriptors->points.size();
    for(int i = 0; i< nr_of_descriptors; i++){
        search_tree->nearestKSearch (object_model.global_descriptors->points[i],1,best_match,square_distance);
        std::cout << "loop nr: " << i << ", best match: " << best_match.at(0) << ", confidence level: " << square_distance.at(0) << std::endl;
    }
    //search_tree->nearestKSearch (object_model.global_descriptors->points[0],1,best_match,square_distance);
    returnvalues.push_back(best_match[0]);
    returnvalues.push_back(square_distance[0]);
    return (returnvalues);
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
    ec.setClusterTolerance (0.01); // 2cm
    ec.setMinClusterSize (200);
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


