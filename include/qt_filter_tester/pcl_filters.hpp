

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
#include <pcl/filters/shadowpoints.h>



namespace qt_filter_tester {

class PclFilters : public QObject{
Q_OBJECT

public:

    PclFilters(QObject *parent = 0);
    ~PclFilters();

    //! Creates a PCL Visualizer containing the input cloud
    /*!
     * \param cloud Input point cloud
     * \return A pcl visualizer containing the input cloud
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


    //! Creates a PCL Visualizer used to visualize a point clouds normals
    /*!
     * \param cloud Input point cloud
     * \param radius Double value for the search radius for the normal estimation of a point cloud
     * \param numOfNormals Integer value for the number of normals to display in the visualizer
     * \return A pcl visualizer containing the input cloud and normals as defined by the input parameters
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int numOfNormals);


    //! Creates a PCL Visualizer used to visualize a point cloud after passthrough filtering
    /*!
     * \param cloud Input point cloud
     * \param min Double value for the minimum value of the filter
     * \param max Double value for the maximum value of the filter
     * \param axis std::string value for the axis of the filter (lower case)
     * \return A pcl visualizer containig the input cloud filtered using a passthrough filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> passthrough_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);


    //! Creates a PCL Visualizer used to visualize a point cloud after median filtering
    /*!
     * \param cloud Input point cloud
     * \param window_size Integer value for the window size of the filter
     * \param max_allowed_movement Double value for the maximum allowed movenet of the filter
     * \return A pcl visualizer containig the input cloud filtered using a median filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> median_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);


    //! Creates a PCL Visualizer used to visualize a point cloud after voxel grid filtering
    /*!
     * \param cloud Input point cloud
     * \param lx Double value for the voxel size in the "x" axis of the filter
     * \param ly Double value for the voxel size in the "y" axis of the filter
     * \param lz Double value for the voxel size in the "z" axis of the filter
     * \return A pcl visualizer containing the input cloud filtered using a voxel grid filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> voxelgrid_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);


    //! Creates a PCL Visualizer used to visualize a point cloud after shadow point removal filtering
    /*!
     * \param cloud Input point cloud
     * \param threshold Double value for the filter threshold
     * \param radius Double value for the filter search radius
     * \return A pcl visualizer containing the input cloud filtered using a shadow point removal filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> shadowpoint_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius);


    //! Creates a PCL Visualizer used to visualize a point cloud after statistical outlier removal filtering
    /*!
     * \param cloud Input point cloud
     * \param meanK Integer value for the number of nearest neighbors to use for mean distance estimation
     * \param std_deviation_threshold Double value for the standard deviation multiplier for the distance threshold calculation
     * \return A pcl visualizer containing the input cloud filtered using a statistical outlier removal filter
     */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> statistical_outlier_vis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);


    //! Returns the most recent point cloud handled by the class
    /*!
     * \return The most recent point cloud handled by the class
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_cloud();


    //! Returns the normals of a Point cloud
    /*!
     * \param cloud Input point cloud
     * \param radius Double value for the search radius of the normal estimation
     * \return The normals of a point cloud
     */
    pcl::PointCloud<pcl::Normal>::Ptr get_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);


    //! Returns the a point cloud filtered using passthrough filtering
    /*!
     * \param cloud Input point cloud
     * \param min Double value for the minimum value of the filter
     * \param max Double value for the maximum value of the filter
     * \param axis std::string value for the axis of the filter (lower case)
     * \return A point cloud filtered using passthrough filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double min, double max, std::string axis);


    //! Returns the a point cloud filtered using voxel grid filtering
    /*!
     * \param cloud Input point cloud
     * \param lx Double value for the voxel size in the "x" axis of the filter
     * \param ly Double value for the voxel size in the "y" axis of the filter
     * \param lz Double value for the voxel size in the "z" axis of the filter
     * \return A point cloud filtered using voxel grid filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lx, double ly, double lz);


    //! Returns the a point cloud filtered using median filtering
    /*!
     * \param cloud Input point cloud
     * \param window_size Integer value for the window size of the filter
     * \param max_allowed_movement Double value for the maximum allowed movenet of the filter
     * \return A point cloud filtered using median filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr median (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int window_size, double max_allowed_movement);


    //! Returns the a point cloud filtered using shadow point removal filtering
    /*!
     * \param cloud Input point cloud
     * \param threshold Double value for the filter threshold
     * \param radius Double value for the filter search radius
     * \return A point cloud filtered using shadow point removal filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr shadowpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, double radius);


    //! Returns the a point cloud filtered using statistical outlier removal filtering
    /*!
     * \param cloud Input point cloud
     * \param meanK Integer value for the number of nearest neighbors to use for mean distance estimation
     * \param std_deviation_threshold Double value for the standard deviation multiplier for the distance threshold calculation
     * \return A point cloud filtered using statistical outlier removal filtering
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meanK, double std_deviation_threshold);


private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
    pcl::PassThrough<pcl::PointXYZ> passfilter;
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;
    pcl::MedianFilter<pcl::PointXYZ> medianfilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_filter;
    pcl::ShadowPoints<pcl::PointXYZ, pcl::Normal> shadowpoint_filter;

public Q_SLOTS:

Q_SIGNALS:

};
}
#endif
