#ifndef GMU_GUI_H
#define GMU_GUI_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSignalMapper>
#include <map>
#include "grasp_modification_utility.h"
#include "ros/ros.h"

class gmu_gui: public QWidget
{
Q_OBJECT
public:
    gmu_gui(GMU& gmu_);
    ~gmu_gui();

private Q_SLOTS:
    void on_waypoint_selection_changed();
    void on_delete_button_clicked();
    void on_add_before_button_clicked();
    void on_add_last_button_clicked();
    void on_edit_button_clicked();
    void on_copy_button_clicked();
    void on_save_button_clicked();
    void on_abort_button_clicked();
    void on_text_changed(const int& id);

private:
    GMU& gmu;

    void toggle_top_layout(bool enable);
    void toggle_middle_layouts(bool enable);
    void toggle_bottom_layout(bool enable);
    void starting_mode(bool starting);
    bool initialize_gmu();
    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
    void update_coords(geometry_msgs::Pose wp);

    int current_wp=0;
    bool editing; //false means copying
    dual_manipulation_shared::grasp_trajectory grasp_msg;
    int obj_id;
    int grasp_id;
    std::string file_name;

    QVBoxLayout main_layout;

    QHBoxLayout layout1;
    QLabel waypoint_label;
    QComboBox waypoint_selection;
    QLabel object_label;
    QLineEdit object_text;
    QLabel grasp_id_label;
    QLineEdit grasp_id_text;
    QPushButton edit;
    QPushButton copy;

    QHBoxLayout layout2;
    QPushButton delete_button;
    QPushButton add_before_button;
    QPushButton add_last_button;

    QGridLayout layout3;
    QLabel pos_label, or_label;
    std::map<int, QLabel*> coord_label;
    std::map<int, QLineEdit*> coord_text;

    QHBoxLayout layout4;
    QLabel synergy_label;
    QSlider synergy_slider;

    QHBoxLayout layout5;
    QPushButton save;
    QPushButton abort;

    ros::NodeHandle node;
    ros::Subscriber sub;

    QSignalMapper signalMapper;
};

#endif //GMU_GUI_H