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
#include <QCheckBox>
#include <map>
#include "grasp_modification_utility.h"
#include "ros/ros.h"
#include <QSocketNotifier>

class gmu_gui: public QWidget
{
Q_OBJECT
public:
    gmu_gui(GMU& gmu_);
    ~gmu_gui();
    
    //Unix signal handlers
    static void intSignalHandler(int);
public Q_SLOTS:
    void handleSigInt();
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
    void on_check_box_changed();
    void on_slider_moved(const int& id);
    void on_time_from_start_changed();

private:
    GMU& gmu;

    void toggle_top_layout(bool enable);
    void toggle_middle_layouts(bool enable);
    void toggle_bottom_layout(bool enable);
    void starting_mode(bool starting);
    bool initialize_gmu();
    void im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback);
    void update_coords(geometry_msgs::Pose wp);
    void update_sliders(int number_of_joints);
    void update_joint_names();
    void publish_joint_state();
    void update_joints_info();
    void delete_joint_wp();
    void add_joint_wp();
    std::string common_prefix(const std::string& a, const std::string& b);

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
    QSignalMapper coord_mapper;

    QVBoxLayout layout4;
    std::vector<QLabel*> synergy_label;
    std::vector<QSlider*> synergy_slider;
    QSignalMapper slider_mapper;
    QLabel time_from_start_label;
    QLineEdit time_from_start_text;

    QHBoxLayout layout4a;
    QLabel check_label;
    QCheckBox check_box;

    QHBoxLayout layout5;
    QPushButton save;
    QPushButton abort;

    ros::NodeHandle node;
    ros::Subscriber sub;
    ros::Publisher joint_pub;
    sensor_msgs::JointState joint_msg;

    static int sigintFd[2];
    QSocketNotifier* snInt;
};

#endif //GMU_GUI_H