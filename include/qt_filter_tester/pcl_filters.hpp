

#ifndef qt_filter_tester_PCLFILTERS_H
#define qt_filter_tester_PCLFILTERS_H

#include <QObject>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "point_cloud_ray_trace.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral_omp.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/esf.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>



namespace qt_filter_tester {


/*!
 * A structure containing all information about a ray trace
 */
struct RayTracedCloud_descriptors {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; /*!< The point cloud of the ray trace */
    Eigen::Matrix4f pose; /*!< The pose transformation from the camera to the mesh when the ray trace was generated */
    float enthropy; /*!< The amount of the whole mesh seen in the camera */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor; /*!< The CVFH descriptor for the cloud */
};

class PclFilters : public QObject{
Q_OBJECT

public:

    PclFilters(QObject *parent = 0);
    ~PclFilters();

    /*!
     * \brief Creates a PCL Visualizer containing the input cloud
     * \param cloud Input point cloud
     * \return A pcl visualizer containing the input cloud
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Creates a PCL Visualizer to visualize a PointXYZRGB point cloud
     * \param cloud Input point cloud
     * \return A pcl visualizer containing the input cloud
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /*!
     * \brief Creates a PCL Visualizer used to visualize a point clouds normals
     * \param cloud Input point cloud
     * \param radius Double value for the search radius for the normal estimation of a point cloud
     * \param numOfNormals Integer value for the number of normals to display in the visualizer
     * \return A pcl visualizer containing the input cloud and normals as defined by the input parameters
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int numOfNormals);

    /*!
     * \brief Segments out a plane for the input point cloud
     * \param cloud Input point cloud
     * \param distance Double value for the maximum distance between points in a plane
     * \return A pcl point cloud containing the points of the segmented plane
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance);

    /*!
     * \brief Creates a PCL Visualizer containing a point cloud of the clusters extracted from the input cloud using Eucledian cluster extraction
     * \param cloud Input point cloud
     * \param distance Double value for the maximum distance between points in a plane
     * \return A pcl visualizer containing the extracted clusters from the input cloud
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_extraction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double distance);

    /*!
     * \brief Returns the most recent point cloud handled by the class
     * \return The most recent point cloud handled by the class
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_cloud();

    /*!
     * \brief Colors all the points in a point cloud
     * \param cloud Input point cloud
     * \param r Integer value for the red component of the color
     * \param g Integer value for the green component of the color
     * \param b Integer value for the blue component of the color
     * \return A RGB point cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b);

    /*!
     * \brief Returns the normals of a Point cloud
     * \param cloud Input point cloud
     * \param radius Double value for the search radius of the normal estimation
     * \return The normals of a point cloud
     */
    pcl::PointCloud<pcl::Normal>::Ptr get_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);

    /*!
     * \brief Returns the a point cloud filtered using passthrough filtering
     * \param cloud Input point cloud
     * \param min Double value for the minimum value of the filter
     * \param max Double value for the maximum value of the filter
     * \param axis std::string value for the axis of the filter (lower case)
     * \return A point cloud filtered using passthrough filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);

    /*!
     * \brief Returns the a point cloud filtered using voxel grid filtering
     * \param cloud Input point cloud
     * \param lx Double value for the voxel size in the "x" axis of the filter
     * \param ly Double value for the voxel size in the "y" axis of the filter
     * \param lz Double value for the voxel size in the "z" axis of the filter
     * \return A point cloud filtered using voxel grid filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);

    /*!
     * \brief Returns the a point cloud filtered using median filtering
     * \param cloud Input point cloud
     * \param window_size Integer value for the window size of the filter
     * \param max_allowed_movement Double value for the maximum allowed movenet of the filter
     * \return A point cloud filtered using median filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr median_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);

    /*!
     * \brief Returns the a point cloud filtered using shadow point removal filtering
     * \param cloud Input point cloud
     * \param threshold Double value for the filter threshold
     * \param radius Double value for the filter search radius
     * \return A point cloud filtered using shadow point removal filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr shadowpoint_removal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius);

    /*!
     * \brief Returns the a point cloud filtered using statistical outlier removal filtering
     * \param cloud Input point cloud
     * \param meanK Integer value for the number of nearest neighbors to use for mean distance estimation
     * \param std_deviation_threshold Double value for the standard deviation multiplier for the distance threshold calculation
     * \return A point cloud filtered using statistical outlier removal filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);

    /*!
     * \brief Combines all clouds in an vector to one cloud
     * \param input std::vector containing all clouds to be combined
     * \return A point cloud containing all clouds in the input vector
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr combine_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input);

    /*!
     * \brief Generates the CVFH descriptors for an object (cluster)
     * \param object Input point cloud, cluster of the object
     * \param normals Input normal cloud of the object
     * \return A CVFH Descriptor
     */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr compute_cvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr object);

    /*!
     * \brief Returns a pcl point cloud filtered using a bilateral filter
     * \param cloud Input point cloud
     * \param sigmaS Double value for the half size of the gaussian bilateral filter window
     * \param sigmaR Double value for the standard deviation parameter
     * \return A point cloud filtered using a bilateral filter
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr bilateral_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sigmaS, double sigmaR);

    /*!
     * \brief Calculates the CVFH descriptors for an array of point clouds
     * \param inclouds Array of point clouds
     * \return Array of point cloud structs with descriptors
     */
    std::vector<RayTracedCloud_descriptors> get_descriptors(std::vector<RayTraceCloud> inclouds);

    /*!
     * \brief Returns the esf descriptor for a pcl point cloud
     * \param cloud Input point cloud
     * \return ESDSignature640 descriptor for the input point cloud
     */
    pcl::PointCloud<pcl::ESFSignature640>::Ptr compute_esf_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Returns the ourcvfh descriptor for a pcl point cloud
     * \param cloud Input point cloud
     * \return OurCVFH descriptor for the input point cloud
     */
    pcl::PointCloud<pcl::VFHSignature308>::Ptr compute_ourcvfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /*!
     * \brief Returns the gfpfh descriptor for a pcl point cloud
     * \param cloud Input point cloud
     * \return GFPFH descriptor for the input point cloud
     */
    pcl::PointCloud<pcl::GFPFHSignature16>::Ptr compute_gfpfh_descriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; //!< A pcl viewer used to visualize pcl point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud; //!< The product of a filter operation.
    pcl::PassThrough<pcl::PointXYZ> passfilter; //!< A pcl passthrough filter object.
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter; //!< A pcl voxelgrid filter object.
    pcl::MedianFilter<pcl::PointXYZ> medianfilter; //!< A pcl median filter object.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier; //!< A pcl statistical outlier removal filter object.
    pcl::ShadowPoints<pcl::PointXYZ, pcl::Normal> shadowpoint_filter; //!< A pcl shadowpoints removal filter object.


public Q_SLOTS:

Q_SIGNALS:

};
}
#endif
