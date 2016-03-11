//#ifndef MODELLOADER_HPP
//#define MODELLOADER_HPP

#ifndef qt_filter_tester_MODELLOADER_H
#define qt_filter_tester_MODELLOADER_H

#include <QObject>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <string.h>
#include <sstream>
#include <iomanip>

#include "pcl_filters.hpp"

namespace qt_filter_tester {

class ModelLoader : public QObject
{
    Q_OBJECT

public:


    ModelLoader(pcl::PolygonMesh mesh, std::string mesh_name);

    ModelLoader(std::string mesh_name);


    ~ModelLoader();

    std::vector<RayTraceCloud> getModels(bool load = false);

    void populateLoader();

    void setMesh(pcl::PolygonMesh mesh){
        ModelLoader::mesh = mesh;
    }

    void setMeshName(std::string mesh_name){
        ModelLoader::mesh_name = mesh_name;
    }

    void setTesselation_level(int tesselation_level){
        ModelLoader::tesselation_level = tesselation_level;
    }

    void setCloudResolution(int cloud_resolution){
        ModelLoader::cloud_resolution = cloud_resolution;
    }

    void setPath(const std::string &path){
        ModelLoader::path = path;
    }

Q_SIGNALS:

public Q_SLOTS:

private:
    //! Generate the point clouds from a mesh
    /*!
     * This function will generate the traces from a mesh and populate the ray_trace_clouds variable
     */
    void generatePointClouds();

    //! Loads the ray trace point clouds from files
    /*!
     * This function will load and populate the ray_trace_clouds variable from the given path
     */
    bool loadPointClouds();

    //! Saves the ray trace point clouds to files
    /*!
     * This function will save all teh infromation from the ray_trace_clouds variable to the given path
     */
    bool savePointClouds();

    std::string path; //!< The path for saving and loading files.
    std::string mesh_name; //!< The name of the mesh. Used for saving and loading file names.
    std::vector<RayTraceCloud> ray_trace_clouds; //!< List of ray trace clouds.
    pcl::PolygonMesh mesh; //!< The mesh which is used for generation
    int cloud_resolution; //!< The resolution camera when generating clouds
    int tesselation_level; //!< The tesselation level of the sphere for the camera
    PclFilters *filters;
};

}

#endif // MODELLOADER_HPP
