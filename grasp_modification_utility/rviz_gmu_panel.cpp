#include "ros/ros.h"
#include "grasp_modification_utility.h"
#include "GMU_gui.h"
#include <rviz/panel.h>

class rviz_gmu_panel: public rviz::Panel
{
public:
    rviz_gmu_panel(QWidget* parent = 0);
private:
    GMU gmu;
    gmu_gui gui;
};

rviz_gmu_panel::rviz_gmu_panel(QWidget* parent): Panel(parent), gui(gmu)
{
    setLayout(&gui.main_layout);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gmu_panel,rviz::Panel )