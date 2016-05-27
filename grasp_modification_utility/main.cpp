/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Alessandro Settimi, Hamal Marino, Mirko Ferrati, Centro di Ricerca "E. Piaggio", University of Pisa
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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

