/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/qt_filter_tester/main_window.hpp"


namespace qt_filter_tester {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    qnode.init();
    this->showMaximized();
    loggingModel = new QStringListModel(this);
    filters = new PclFilters();
    filter_changed_flag = true;
    init_ui_elemets();
}

MainWindow::~MainWindow() {}


void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::init_ui_elemets(){
    ui.filter_selection_box->clear();
    ui.filter_selection_box->addItem("PassThrough");
    ui.filter_selection_box->addItem("VoxelGrid");
    ui.filter_selection_box->addItem("Median");
    ui.filter_selection_box->addItem("ShadowPoints");
    ui.filter_selection_box->addItem("Normal Estimation");
    ui.filter_selection_box->addItem("Statistical outlied removal");
    ui.passthrough_input_3->addItem("x");
    ui.passthrough_input_3->addItem("y");
    ui.passthrough_input_3->addItem("z");
    ui.run_action_button->setEnabled(false);
    ui.reload_button->setEnabled(false);
    ui.save_cloud_button->setEnabled(false);
    ui.listView->setModel(loggingModel);

    w1 = new QVTKWidget();
    w2 = new QVTKWidget();
    viewer1.reset(new pcl::visualization::PCLVisualizer ("3D Viewer", false));
    viewer2.reset(new pcl::visualization::PCLVisualizer ("3D Viewer", false));
    w1->SetRenderWindow(viewer1->getRenderWindow());
    w2->SetRenderWindow(viewer2->getRenderWindow());
    viewer1->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    viewer2->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
    ui.verticalLayout->addWidget(w1);
    ui.verticalLayout_2->addWidget(w2);
    ui.filter_selection_box->setEnabled(false);
    hide_all_filter_inputs();
}

void MainWindow::display_viewer_1(QString url)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(url.toUtf8().constData(), *displayCloud) == -1)
    {
        print_to_logg("Could not load file");
        return;
    }
    input_cloud = displayCloud;
    viewer1 = filters->visualize(displayCloud);
    w1->SetRenderWindow(viewer1->getRenderWindow());
    viewer1->setCameraPosition( 0.146598, 0.0941454, -4.95334, -0.0857047, -0.0396425, 0.600109, -0.00146821, -0.999707, -0.0241453, 0);
}

void MainWindow::display_viewer_2(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer){
    output_cloud = filters->get_filtered_cloud();
    viewer2->getCameras(cam);
    viewer2 = viewer;
    w2->SetRenderWindow(viewer2->getRenderWindow());  
    viewer2->setCameraPosition(cam[0].pos[0],cam[0].pos[1],cam[0].pos[2],cam[0].view[0],cam[0].view[1],cam[0].view[2]);
    filter_changed_flag = false;
    ui.save_cloud_button->setEnabled(true);
    ui.reload_button->setEnabled(true);
}


void MainWindow::print_filter_values(){
    loggstring = "";
    switch (ui.filter_selection_box->currentIndex()){
    case 0:
        // passthrough
        loggstring.append("Passthrough parameters: ");
        loggstring.append("Axis: ");
        loggstring.append(ui.passthrough_input_3->currentText());
        loggstring.append(", Min: ");
        loggstring.append(QString::number(ui.filter_input_1->value()));
        loggstring.append(", Max: ");
        loggstring.append(QString::number(ui.filter_input_2->value()));
        print_to_logg(loggstring);
        break;
    case 1:
        // voxel grid
        loggstring.append("Voxelgrid parameters: ");
        loggstring.append("Voxel size: ");
        loggstring.append(QString::number(ui.filter_input_1->value()));
        print_to_logg(loggstring);
        break;
    case 2:
        //median
        loggstring.append("Median paramters: ");
        loggstring.append("Window size: ");
        loggstring.append(QString::number(round(ui.filter_input_1->value())));
        loggstring.append(", Maximum allowed movement: ");
        loggstring.append(QString::number(ui.filter_input_2->value()));
        print_to_logg(loggstring);
        break;
    case 3:
        //shadow point
        break;
    default:
        break;
    }
}

void MainWindow::print_to_logg(QString msg){
    QString tmp = "Logg: ";
    tmp.append(msg);
    loggingModel->insertRows(loggingModel->rowCount(),1);
    loggingModel->setData(loggingModel->index(loggingModel->rowCount()-1), tmp);
    ui.listView->scrollToBottom();
}

void MainWindow::on_load_file_button_clicked(bool check){
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/home/minions",tr("PointCload (*.pcd)"));
    if(fileName.length() == 0)
    {
        //no file selected
        loggstring = "No file selected";
        print_to_logg(loggstring);
        return;
    }
    loggstring = "Loading file from: ";
    loggstring.append(fileName);
    print_to_logg(loggstring);
    ui.run_action_button->setEnabled(true);
    ui.filter_selection_box->setEnabled(true);
    display_viewer_1(fileName);
    ui.filter_selection_box->setCurrentIndex(0);
    Q_EMIT on_filter_selection_box_currentIndexChanged(0);
}

void MainWindow::on_run_action_button_clicked(bool check){
    loggstring = "";
    switch(ui.filter_selection_box->currentIndex()){
    case 0:
        // passthrough
        if(filter_changed_flag){
            display_viewer_2(filters->passthrough_vis(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value(),ui.passthrough_input_3->currentText().toStdString()));
        }
        else{
            //update pointcloud in viewer2
            viewer2->updatePointCloud(filters->passthrough(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value(),ui.passthrough_input_3->currentText().toStdString()),"sample cloud");
            output_cloud = filters->get_filtered_cloud();
            w2->update();
        }

        break;
    case 1:
        //voxelGrid
        if(filter_changed_flag){
            display_viewer_2(filters->voxelgrid_vis(input_cloud,ui.filter_input_1->value(),ui.filter_input_1->value(),ui.filter_input_1->value()));
        }
        else{
            viewer2->updatePointCloud(filters->voxelgrid(input_cloud,ui.filter_input_1->value(),ui.filter_input_1->value(),ui.filter_input_1->value()),"sample cloud");
            output_cloud = filters->get_filtered_cloud();
            w2->update();
        }
        break;
    case 2:
        //median
        display_viewer_2(filters->median_vis(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value()));
        break;
    //case 3:
        //shadow points
        // do filter operations here
    case 4:
        //normal estimation
        display_viewer_2(filters->normalsVis(input_cloud,ui.filter_input_1->value()));
        break;
    //case 5:
        // radius outlier removal
        //statistical_Outlier_filter.setKeepOrganized(true);
        //statistical_Outlier_filter.setInputCloud(input_cloud);
        //statistical_Outlier_filter.setMeanK(100);
        //statistical_Outlier_filter.setStddevMulThresh(ui.filter_input_1->value());
        //display_viewer_2(cloud_filtered);
        //break;
    default:
        loggstring.append("Not implemented");
        print_to_logg(loggstring);
        break;
    }
}

void MainWindow::on_save_cloud_button_clicked(bool check){
    QString savefile = QFileDialog::getSaveFileName(this,tr("Save cloud"),"/home/minions",tr("PointCloud (*.pcd)"));
    if(savefile.length() != 0){
        loggstring = "Saving file to: ";
        loggstring.append(savefile);
        pcl::io::savePCDFileASCII(savefile.toUtf8().constData(), *output_cloud);
        print_to_logg(loggstring);
    }
    else{
        print_to_logg("No file selected");
    }
}

void MainWindow::on_reload_button_clicked(bool check){
    QString savefile = "/home/minions/temp_point_cloud.pcd";
    print_to_logg("Reloading file...");
    loggstring.append(savefile);
    pcl::io::savePCDFileASCII(savefile.toUtf8().constData(), *output_cloud);
    QString tmp = "Backup is located at: ";
    tmp.append(savefile);
    print_to_logg(tmp);
    display_viewer_1(savefile);
    print_filter_values();
}

void MainWindow::on_filter_selection_box_currentIndexChanged(int i){
    hide_all_filter_inputs();

    filter_changed_flag = true;
    switch (i){
    case 0:
        // passthrough
        ui.filter_input_1->setRange(-5,5);
        ui.filter_input_2->setRange(-5,5);
        ui.filter_input_1->setSingleStep(0.1);
        ui.filter_input_2->setSingleStep(0.1);
        ui.filter_input_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        ui.passthrough_input_3->setVisible(true);
        ui.input_label_1->setText("Min:");
        ui.input_label_2->setText("Max:");
        ui.input_label_3->setText("Axis:");
        continously_update_filter_flag = true;
        break;
    case 1:
        // voxel grid
        ui.input_label_1->setText("Voxel size");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_input_1->setRange(0.001,0.2);
        continously_update_filter_flag = true;
        break;
    case 2:
        //median
        ui.input_label_1->setText("Window size");
        ui.input_label_2->setText("Movement");
        ui.filter_input_1->setRange(0,80);
        ui.filter_input_2->setRange(0.001,1);
        ui.filter_input_1->setSingleStep(1);
        ui.filter_input_2->setSingleStep(0.001);
        ui.filter_input_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        continously_update_filter_flag = false;
        break;
    case 3:
        //shadow point
        ui.input_label_1->setText("Threshold");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_1->setRange(0,1);
        ui.filter_input_1->setSingleStep(0.001);
        continously_update_filter_flag = false;
        break;
    case 4:
        //normal estimation
        ui.input_label_1->setText("Radius");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_1->setRange(0.001,0.5);
        ui.filter_input_1->setSingleStep(0.001);
        continously_update_filter_flag = false;
        //all blank
        break;
    case 5:
        //statistical outlier removal
        ui.input_label_1->setText("MeanK");
        ui.input_label_2->setText("StddevMulThresh");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        ui.filter_input_1->setRange(1,100);
        ui.filter_input_1->setSingleStep(1);
        ui.filter_input_2->setRange(0.001,1);
        ui.filter_input_2->setSingleStep(0.001);
        continously_update_filter_flag = false;
        break;
    default:
        break;
    }
}

void MainWindow::on_filter_slider_1_valueChanged(int i){
    double tmp = i/100.0;
    if(ui.filter_input_1->minimum() < 0){
        ui.filter_input_1->setValue(ui.filter_input_1->minimum()+(tmp*(abs(ui.filter_input_1->minimum())+abs(ui.filter_input_1->maximum()))));
    }
    else{
        ui.filter_input_1->setValue(ui.filter_input_1->minimum()+(tmp*(ui.filter_input_1->maximum()-ui.filter_input_1->minimum())));
    }
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}
void MainWindow::on_filter_slider_2_valueChanged(int i){
    double tmp = i/100.0;
    if(ui.filter_input_2->minimum() < 0){
        ui.filter_input_2->setValue(ui.filter_input_2->minimum()+(tmp*(abs(ui.filter_input_2->minimum())+abs(ui.filter_input_2->maximum()))));
    }
    else{
        ui.filter_input_2->setValue(ui.filter_input_2->minimum()+(tmp*(ui.filter_input_2->maximum()-ui.filter_input_2->minimum())));
    }
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}
void MainWindow::on_filter_slider_3_valueChanged(int i){
    double tmp = i/100.0;
    if(ui.filter_input_3->minimum() < 0){
        ui.filter_input_3->setValue(ui.filter_input_3->minimum()+(tmp*(abs(ui.filter_input_3->minimum())+abs(ui.filter_input_3->maximum()))));
    }
    else{
        ui.filter_input_3->setValue(ui.filter_input_3->minimum()+(tmp*(ui.filter_input_3->maximum()-ui.filter_input_3->minimum())));
    }
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}

void MainWindow::on_filter_input_1_valueChanged(double d){
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}

void MainWindow::on_filter_input_2_valueChanged(double d){
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}

void MainWindow::on_filter_input_3_valueChanged(double d){
    if(continously_update_filter_flag){
        Q_EMIT on_run_action_button_clicked(true);
    }
}

void MainWindow::hide_all_filter_inputs(){
    ui.filter_input_1->setVisible(false);
    ui.filter_input_2->setVisible(false);
    ui.filter_input_3->setVisible(false);
    ui.filter_slider_1->setVisible(false);
    ui.filter_slider_2->setVisible(false);
    ui.filter_slider_3->setVisible(false);
    ui.passthrough_input_3->setVisible(false);
    ui.input_label_1->setText("");
    ui.input_label_2->setText("");
    ui.input_label_3->setText("");
}
}

