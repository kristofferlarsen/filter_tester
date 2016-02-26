/**
 * @file /include/qt_filter_tester/main_window.hpp
 *
 * @brief Qt based gui for QT_test.
 *
 * @date November 2010
 **/
#ifndef qt_filter_tester_MAIN_WINDOW_H
#define qt_filter_tester_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "point_cloud_ray_trace.hpp"
#include "modelloader.hpp"

#include <vtkRenderWindow.h>
#include <QVTKWidget.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

#include <pcl/io/vtk_lib_io.h>

#include <eigen3/Eigen/Core>

#include <cmath>
#include <iostream>
#include <stdlib.h>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_filter_tester {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();
    void print_to_logg(QString msg);
    void display_viewer_1(QString url);
    void hide_all_filter_inputs();
    void display_viewer_2(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void print_filter_values();
    void init_ui_elemets();
    void reset_sliders();
    void render_raytrace(std::string partName, std::string directory);

public Q_SLOTS:
    //auto connection
    void on_load_file_button_clicked(bool check);
    void on_run_action_button_clicked(bool check);
    void on_save_cloud_button_clicked(bool check);
    void on_reload_button_clicked(bool check);
    void on_test_button_clicked(bool check);
    void on_create_database_part_clicked(bool check);
    void on_filter_selection_box_currentIndexChanged(int i);
    void on_filter_slider_1_valueChanged(int i);
    void on_filter_slider_2_valueChanged(int i);
    void on_filter_slider_3_valueChanged(int i);
    void on_filter_input_1_valueChanged(double d);
    void on_filter_input_2_valueChanged(double d);
    void on_filter_input_3_valueChanged(double d);
    //manual connection

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;
    std::vector<pcl::visualization::Camera> cam;
    bool filter_changed_flag;
    bool continously_update_filter_flag;

    QVTKWidget *w1;
    QVTKWidget *w2;
    QStringListModel *loggingModel;
    QString loggstring;
    PclFilters *filters;
};

}

#endif
