#include <iostream>
#include "ros/ros.h"
#include <string>
#include "grasp_modification_utility.h"
#include "GMU_gui.h"
#include <QApplication>
#include <thread>
#include "ros/package.h"
#include <csignal>

int main(int argc, char** argv)
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "grasp_modification_utility", ros::init_options::AnonymousName );
    }
    
    ROS_INFO_STREAM("This is a utility to modify one serialized grasp (post grasp pose is with the blue object - grasp trajectory with the green one)");
    
    ros::AsyncSpinner spin(1);
    spin.start();
    GMU gmu;

    QApplication app(argc,argv);
    gmu_gui gui(gmu);

    struct sigaction sigint;
    sigint.sa_handler = gmu_gui::intSignalHandler;
    sigemptyset(&sigint.sa_mask);
    sigint.sa_flags |= SA_RESTART;
    if (sigaction(SIGINT, &sigint, 0) > 0) ROS_WARN_STREAM("Something wrong with unix interrupt signal handling");

    gui.setWindowTitle("GMU");
    gui.setWindowIcon(QIcon(QString::fromStdString(ros::package::getPath("dual_manipulation_grasp_db")) + "/grasp_modification_utility/gmu.png"));
    gui.setWindowFlags(Qt::WindowStaysOnTopHint);
    gui.show();
    app.exec();

    ROS_INFO_STREAM("Bye! Thanks for using this terrific program.");
    return 0;
}

