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

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <QVTKWidget.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/file_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <cmath>

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
    void load_and_display_cloud(QString url);
    void hide_all_filter_inputs();
    void display_output_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud);
    void print_filter_values();
    void init_ui_elemets();

public Q_SLOTS:
    void on_load_file_button_clicked(bool check);
    void on_run_action_button_clicked(bool check);
    void on_save_cloud_button_clicked(bool check);
    void on_reload_button_clicked(bool check);
    void on_filter_selection_box_currentIndexChanged(int i);
    void on_filter_slider_1_valueChanged(int i);
    void on_filter_slider_2_valueChanged(int i);
    void on_filter_slider_3_valueChanged(int i);
    void on_filter_input_1_valueChanged(double d);
    void on_filter_input_2_valueChanged(double d);
    void on_filter_input_3_valueChanged(double d);

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
    pcl::PassThrough<pcl::PointXYZ> passfilter;
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;
    pcl::MedianFilter<pcl::PointXYZ> medianfilter;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_Outlier_filter;
    QVTKWidget *w1;
    QVTKWidget *w2;
    QStringListModel *loggingModel;
    QString loggstring;
};

}

#endif
