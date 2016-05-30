//
// Original author Kristoffer Larsen. Latest change date 01.05.2016
// main_window.cpp is responsible for all user interaction and information flow. Because of this, the majority
// of the methods implemented are used for user interaction.
//
// Created as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//

#ifndef qt_filter_tester_MAIN_WINDOW_H
#define qt_filter_tester_MAIN_WINDOW_H

#include <QtGui>
#include <QMessageBox>

#include "ui_main_window.h"
#include "qnode.hpp"
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

namespace qt_filter_tester {

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:

    /*!
     * \brief Constructor for the MainWindow class.
     * \param argc Initial argument.
     * \param argv Initial argument.
     * \param parent Parent arguemnt, default = 0.
     */
	MainWindow(int argc, char** argv, QWidget *parent = 0);

    ~MainWindow();

    /*!
     * \brief Window event handler for the closeing window event.
     * \param event The closing event.
     */
    void closeEvent(QCloseEvent *event);

    /*!
     * \brief Displays a message to the user through the graphical user interface.
     * \param msg The message to display.
     */
    void print_to_logg(QString msg);

    /*!
     * \brief Loads a .PCD file and displays it in the PCL visualizer 1.
     * \param url Path to the file.
     */
    void display_viewer_1(QString url);

    /*!
     * \brief Hides all user controlable filter inputs in the graphical user interface.
     */
    void hide_all_filter_inputs();

    /*!
     * \brief Displays a pcl vislualizer in the graphical user interface.
     * \param viewer The PCL visualizer to display.
     */
    void display_viewer_2(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

    /*!
     * \brief Prints the current filter values used to the user in the feedback section of the
     *        graphical user interface.
     */
    void print_filter_values();

    /*!
     * \brief Initializing method to populatae the grahical user interface.
     */
    void init_ui_elemets();

    /*!
     * \brief Resets the current value of the sliders to the default value.
     */
    void reset_sliders();

    /*!
     * \brief Creates a training set of a user specified .STL part.
     * \param partName The name of the training set.
     * \param directory Path to the .STL part.
     */
    void render_raytrace(std::string partName, std::string directory);

public Q_SLOTS:

    /*!
     * \brief Event handler for the "Load file" button. A open file dialog is presented to the user,
     *        and the selected .PCD file is loaded and visualizer.
     * \param check Button status for the event.
     */
    void on_load_file_button_clicked(bool check);

    /*!
     * \brief Event handler for the "Run action" button. The selected filter action and parameters are performed.
     * \param check Button status for the event.
     */
    void on_run_action_button_clicked(bool check);

    /*!
     * \brief Event handler for the "Save cloud" button. The point cloud currently displayed in
     *        visualizer 2 is saved to a user specified location.
     * \param check Button status for the event.
     */
    void on_save_cloud_button_clicked(bool check);

    /*!
     * \brief Event handler for the "Reload" button. Saves the current point cloud, and reloads it (usefull for performing
     *        different filters in a sequence).
     * \param check Button status for the event.
     */
    void on_reload_button_clicked(bool check);

    /*!
     * \brief Event handler for the "Create database" button. Creates a training set for a user specified .STL CAD model.
     * \param check Button status for the event.
     */
    void on_create_database_part_clicked(bool check);

    /*!
     * \brief Index changed event for "filter selection combobox"
     * \param i Current index.
     */
    void on_filter_selection_box_currentIndexChanged(int i);

    /*!
     * \brief Value changed event for the "filter slider 1" slider.
     * \param i Current value.
     */
    void on_filter_slider_1_valueChanged(int i);

    /*!
     * \brief Value changed event for the "filter slider 2" slider.
     * \param i Current value.
     */
    void on_filter_slider_2_valueChanged(int i);

    /*!
     * \brief Value changed event for the "filter slider 3" slider.
     * \param i Current value.
     */
    void on_filter_slider_3_valueChanged(int i);

    /*!
     * \brief Value changed event for the "Filter input 1" spinbox.
     * \param d Current value.
     */
    void on_filter_input_1_valueChanged(double d);

    /*!
     * \brief Value changed event for the "Filter input 2" spinbox.
     * \param d Current value.
     */
    void on_filter_input_2_valueChanged(double d);

    /*!
     * \brief Value changed event for the "FIlter input 3" spinbox.
     * \param d Current value.
     */
    void on_filter_input_3_valueChanged(double d);

private:
    Ui::MainWindowDesign ui; //!< Object of the user interface defention XML (this XML defines the graphical apperance of the application).
    QNode qnode; //!< Local object of the QNode class. Used for ROS communication (no ROS communication is implemented).
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1; //!< PCL visualizer 1.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2; //!< PCL visualizer 2.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud; //!< Point cloud displayed in visualizer 1.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud; //!< Point cloud displayed in visualizer 2.
    std::vector<pcl::visualization::Camera> cam; //!< PCL camera object, used to maintain the same camera view in the visualizer when applying a filter.
    bool filter_changed_flag; //!< Flag used to check if the selected filter has changed.
    bool continously_update_filter_flag; //!< Flag used to indicate continously display update, or single image update.

    QVTKWidget *w1; //!< QVTK Widget used to display a PCL visualizer embedded in the graphical user interface.
    QVTKWidget *w2; //!< QVTK Widget used to display a PCL visualizer embedded in the graphical user interface.
    QStringListModel *loggingModel; //!< StringListModel used to give feedback to the user.
    QString loggstring; //!< String holding the feedback message displayed to the user.
    PclFilters *filters; //!< Local object of the PclFilters class. Used for 3D point cloud processing.
    ModelLoader *freakthing; //!< Modelloader used to load part A.
    ModelLoader *box; //!< Modelloader used to load a test part (a box).
    ModelLoader *cone; //!< Modelloader used to load part B.
};
}

#endif
