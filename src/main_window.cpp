//
// Original author Kristoffer Larsen. Latest change date 01.05.2016
// main_window.cpp is responsible for all user interaction and information flow. Because of this, the majority
// of the methods implemented are used for user interaction.
//
// Created as part of the software solution for a Master's Thesis in Production Technology at NTNU Trondheim.
//

#include "../include/qt_filter_tester/main_window.hpp"
#include <ctime>

namespace qt_filter_tester {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv){
    ui.setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    qnode.init();
    this->showMaximized();
    loggingModel = new QStringListModel(this);
    filters = new PclFilters();
    filter_changed_flag = true;
    init_ui_elemets();
    freakthing = new ModelLoader("freakthing-42-200");
    cone = new ModelLoader("cone-42-200");
    box = new ModelLoader("nuc-42-100");
    freakthing->populateLoader();
    box->populateLoader();
    cone->populateLoader();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event){
    QMainWindow::closeEvent(event);
}

void MainWindow::init_ui_elemets(){
    ui.filter_selection_box->clear();
    ui.filter_selection_box->addItem("PassThrough");
    ui.filter_selection_box->addItem("VoxelGrid");
    ui.filter_selection_box->addItem("Median");
    ui.filter_selection_box->addItem("ShadowPoints");
    ui.filter_selection_box->addItem("Normal Estimation");
    ui.filter_selection_box->addItem("Statistical outlier removal");
    ui.filter_selection_box->addItem("Plane model segmentation");
    ui.filter_selection_box->addItem("ClusterExtraction");
    ui.filter_selection_box->addItem("Bilateral filter");
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

void MainWindow::reset_sliders(){
    bool tmp = continously_update_filter_flag;
    continously_update_filter_flag = false;
    Q_EMIT on_filter_slider_1_valueChanged(50);
    Q_EMIT on_filter_slider_2_valueChanged(50);
    Q_EMIT on_filter_slider_3_valueChanged(50);
    ui.filter_slider_1->setValue(50);
    ui.filter_slider_2->setValue(50);
    ui.filter_slider_3->setValue(50);
    continously_update_filter_flag = tmp;
}

void MainWindow::render_raytrace(std::string partName, std::string directory){
    QString msg = "";
    msg.append("Rendering file: ");
    msg.append(directory.data());
    msg.append(", as partname: ");
    msg.append(partName.data());
    print_to_logg(msg);
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL(directory,mesh);

    pcl::PointCloud<pcl::PointXYZ> scaled_mesh;
    Eigen::Matrix4f scaleMatrix = Eigen::Matrix4f::Identity();
    scaleMatrix(0,0)=0.001f;
    scaleMatrix(1,1)=0.001f;
    scaleMatrix(2,2)=0.001f;

    pcl::fromPCLPointCloud2(mesh.cloud,scaled_mesh);
    pcl::transformPointCloud(scaled_mesh,scaled_mesh,scaleMatrix);
    pcl::toPCLPointCloud2(scaled_mesh, mesh.cloud);

    ModelLoader *render = new ModelLoader(mesh, partName);
    render->setCloudResolution(200);
    render->setTesselation_level(1);
    render->populateLoader();
}

void MainWindow::display_viewer_1(QString url){
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
        loggstring.append("Shadow point removal paramters: ");
        loggstring.append("Threshold: ");
        loggstring.append(QString::number(round(ui.filter_input_1->value())));
        loggstring.append(", Radius (normal search): ");
        loggstring.append(QString::number(ui.filter_input_2->value()));
        print_to_logg(loggstring);
        break;
    case 4:
        //Normal estimation
        loggstring.append("Normal estimation paramters: ");
        loggstring.append("Radius (normal search): ");
        loggstring.append(QString::number(ui.filter_input_1->value()));
        loggstring.append(", Number of normals displayed: ");
        loggstring.append(QString::number(ui.filter_input_2->value()));
        print_to_logg(loggstring);
        break;
    case 5:
        //statistical outlier removal
        loggstring.append("Statistical outlier removal paramters: ");
        loggstring.append("MeanK: ");
        loggstring.append(QString::number(round(ui.filter_input_1->value())));
        loggstring.append(", Standard deviation threshold: ");
        loggstring.append(QString::number(ui.filter_input_2->value()));
        print_to_logg(loggstring);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_rgb,plane_rgb;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud, plane;
    switch(ui.filter_selection_box->currentIndex()){
    case 0:
        // passthrough
        if(filter_changed_flag){
            display_viewer_2(filters->visualize(filters->passthrough_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value(),ui.passthrough_input_3->currentText().toStdString())));
        }
        else{
            //update pointcloud in viewer2
            viewer2->updatePointCloud(filters->passthrough_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value(),ui.passthrough_input_3->currentText().toStdString()),"sample cloud");
            output_cloud = filters->get_filtered_cloud();
            w2->update();
        }
        break;
    case 1:
        //voxelGrid
        if(filter_changed_flag){
            display_viewer_2(filters->visualize(filters->voxel_grid_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_1->value(),ui.filter_input_1->value())));
        }
        else{
            viewer2->updatePointCloud(filters->voxel_grid_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_1->value(),ui.filter_input_1->value()),"sample cloud");
            output_cloud = filters->get_filtered_cloud();
            w2->update();
        }
        break;
    case 2:
        //median
        display_viewer_2(filters->visualize(filters->median_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value())));
        break;
    case 3:
        //shadow points
        display_viewer_2(filters->visualize(filters->shadowpoint_removal_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value())));
        break;
    case 4:
        //normal estimation
        display_viewer_2(filters->visualize_normals(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value()));
        break;
    case 5:
        //statistical outlier removal
        display_viewer_2(filters->visualize(filters->statistical_outlier_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value())));
        break;
    case 6:
        //segmentation
        plane = filters->plane_segmentation(input_cloud,ui.filter_input_1->value());
        plane_rgb = filters->color_cloud(plane,255,0,0);
        if(!ui.filter_checkbox_1->isChecked()){
            //display plane and cloud
            cluster_cloud_rgb = filters->color_cloud(input_cloud,255,255,255);
            *cluster_cloud_rgb += *plane_rgb;
            plane_rgb = cluster_cloud_rgb;
        }
        display_viewer_2(filters->visualize_rgb(plane_rgb));
        break;
    case 7:
        //cluster extraction
        plane = filters->plane_segmentation(input_cloud,ui.filter_input_1->value());
        cluster_cloud = filters->combine_clouds(filters->cluster_extraction(input_cloud,ui.filter_input_1->value()));
        plane_rgb = filters->color_cloud(plane,255,0,0);
        cluster_cloud_rgb = filters->color_cloud(cluster_cloud,0,0,255);
        if(ui.filter_checkbox_1->isChecked()){
            //add plane to the point cloud
            *cluster_cloud_rgb += *plane_rgb;
        }
        display_viewer_2(filters->visualize_rgb(cluster_cloud_rgb));
        break;
    case 8:
        //bilateral filtering
        display_viewer_2(filters->visualize(filters->bilateral_filter(input_cloud,ui.filter_input_1->value(),ui.filter_input_2->value())));
        break;
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

void MainWindow::on_create_database_part_clicked(bool check){
    QString filePath = QFileDialog::getOpenFileName(this, tr("Open STL File"),"/home/minions",tr("STL cad model (*.stl)"));
    if(!filePath.isEmpty()){
        bool ok;
        QString partName = QInputDialog::getText(this, tr("Name the part"), tr("Part name"), QLineEdit::Normal, tr(""), &ok);
        if(!partName.isEmpty() && ok){
            std::cout << "Part directory: " << filePath.toStdString() << std::endl;
            std::cout << "Part name: " << partName.toStdString() << std::endl;
            render_raytrace(partName.toStdString(),filePath.toStdString());
        }
    }
}

void MainWindow::on_filter_selection_box_currentIndexChanged(int i){
    hide_all_filter_inputs();
    filter_changed_flag = true;
    switch (i){
    case 0:
        // passthrough
        continously_update_filter_flag = true;
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
        break;
    case 1:
        // voxel grid
        continously_update_filter_flag = true;
        ui.input_label_1->setText("Voxel size");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_input_1->setRange(0.001,0.2);

        break;
    case 2:
        //median
        continously_update_filter_flag = false;
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
        break;
    case 3:
        //shadow point
        continously_update_filter_flag = false;
        ui.input_label_1->setText("Threshold");
        ui.input_label_2->setText("Radius");
        ui.filter_input_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        ui.filter_input_1->setRange(0,1);
        ui.filter_input_2->setRange(0.001,0.5);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_input_2->setSingleStep(0.001);
        break;
    case 4:
        //normal estimation
        continously_update_filter_flag = false;
        ui.input_label_1->setText("Radius");
        ui.input_label_2->setText("NumNormals");
        ui.filter_input_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        ui.filter_input_1->setRange(0.001,0.5);
        ui.filter_input_2->setRange(1,10);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_input_2->setSingleStep(1);
        break;
    case 5:
        //statistical outlier removal
        continously_update_filter_flag = false;
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
        break;
    case 6:
        //segmentation
        continously_update_filter_flag = false;
        ui.input_label_1->setText("Point distance");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_input_1->setRange(0.001,0.02);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_checkbox_1->setText("Show only plane");
        ui.filter_checkbox_1->setVisible(true);
        break;
    case 7:
        continously_update_filter_flag = false;
        ui.input_label_1->setText("Point distance");
        ui.filter_input_1->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_checkbox_1->setVisible(true);
        ui.filter_checkbox_1->setText("Show plane");
        ui.filter_input_1->setRange(0.001,0.02);
        ui.filter_input_1->setSingleStep(0.001);
        //cluster extraction
        break;
    case 8:
        //bilateral filtering
        continously_update_filter_flag = false;
        ui.input_label_1->setText("SigmaS");
        ui.input_label_2->setText("SigmaR");
        ui.filter_input_1->setVisible(true);
        ui.filter_input_2->setVisible(true);
        ui.filter_slider_1->setVisible(true);
        ui.filter_slider_2->setVisible(true);
        ui.filter_input_1->setRange(0.001,5);
        ui.filter_input_2->setRange(0.001,5);
        ui.filter_input_1->setSingleStep(0.001);
        ui.filter_input_2->setSingleStep(0.001);
        break;
    default:
        break;
    }
    reset_sliders();
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
    ui.filter_checkbox_1->setVisible(false);
}
}

