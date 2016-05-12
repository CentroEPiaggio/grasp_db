#include "GMU_gui.h"
#include "normalization_utils.cpp"
#include "dual_manipulation_shared/serialization_utils.h"
#include "tf_conversions/tf_kdl.h"
#include "tf/tf.h"
#include <QLayoutItem>
#include "ros/duration.h"
#include <algorithm>

void insertSeparator(QVBoxLayout* tab)
{
    QFrame* myFrame = new QFrame();
    QPalette palette;
    myFrame->setFrameShape(QFrame::HLine);
    myFrame->setLineWidth(2);
    palette.setColor(myFrame->foregroundRole(),Qt::gray);
    myFrame->setPalette(palette);
    tab->addWidget(myFrame);
    tab->setStretchFactor(myFrame,1);
}

gmu_gui::gmu_gui(GMU& gmu_): QWidget(), gmu(gmu_)
{
    sub = node.subscribe("grasp_modification_utility_interactive_marker/feedback",1,&gmu_gui::im_callback,this);
    joint_pub = node.advertise<sensor_msgs::JointState>("gui_joint_state_publisher",1);

    object_label.setText("obj_id");
    grasp_id_label.setText("grasp_id");
    edit.setText("EDIT");
    copy.setText("COPY");
    layout1.addWidget(&object_label);
    layout1.addWidget(&object_text);
    layout1.addWidget(&grasp_id_label);
    layout1.addWidget(&grasp_id_text);
    layout1.addWidget(&edit);
    layout1.addWidget(&copy);

    waypoint_label.setText("wp#");
    delete_button.setText("delete");
    add_before_button.setText("add before");
    add_last_button.setText("add last");
    layout2.addWidget(&waypoint_label);
    layout2.addWidget(&waypoint_selection);
    layout2.addWidget(&delete_button);
    layout2.addWidget(&add_before_button);
    layout2.addWidget(&add_last_button);

    pos_label.setText("pos");
    or_label.setText("or");
    coord_label[0]=new QLabel("x");
    coord_label[1]=new QLabel("y");
    coord_label[2]=new QLabel("z");
    coord_label[3]=new QLabel("r");
    coord_label[4]=new QLabel("p");
    coord_label[5]=new QLabel("y");
    
    for(int i=0;i<6;i++)
    {
	coord_text[i]=new QLineEdit();
	layout3.addWidget(coord_label.at(i),i%3,1+3*(i/3));
	layout3.addWidget(coord_text.at(i),i%3,2+3*(i/3));

	connect(coord_text.at(i), SIGNAL(textEdited(QString)),&coord_mapper, SLOT(map())) ;
	coord_mapper.setMapping(coord_text.at(i), i) ;
    }
    connect(&coord_mapper, SIGNAL(mapped(int)), this, SLOT(on_text_changed(int))) ; 

    layout3.addWidget(&pos_label,0,0);
    layout3.addWidget(&or_label,0,3);

    time_from_start_label.setText("time from start [s]: ");
    time_from_start_text.setText(QString::number(0, 'f', 2));
    connect(&time_from_start_text, SIGNAL(textEdited(QString)),this, SLOT(on_time_from_start_changed())) ;

    check_label.setText("object interaction");
    check_box.setCheckState(Qt::Checked);
    layout4a.addWidget(&check_label);
    layout4a.addWidget(&check_box,0,Qt::AlignLeft);

    save.setText("SAVE");
    abort.setText("ABORT");
    layout5.addWidget(&save);
    layout5.addWidget(&abort);

    main_layout.addLayout(&layout1);
    insertSeparator(&main_layout);
    main_layout.addLayout(&layout2);
    main_layout.addLayout(&layout3);
    main_layout.addLayout(&layout4);
    main_layout.addLayout(&layout4a);
    insertSeparator(&main_layout);
    main_layout.addLayout(&layout5);

    setLayout(&main_layout);

    connect(&edit,SIGNAL(clicked(bool)),this,SLOT(on_edit_button_clicked()));
    connect(&copy,SIGNAL(clicked(bool)),this,SLOT(on_copy_button_clicked()));

    connect(&waypoint_selection,SIGNAL(currentIndexChanged(int)),this,SLOT(on_waypoint_selection_changed()));
    connect(&delete_button,SIGNAL(clicked(bool)),this,SLOT(on_delete_button_clicked()));
    connect(&add_before_button,SIGNAL(clicked(bool)),this,SLOT(on_add_before_button_clicked()));
    connect(&add_last_button,SIGNAL(clicked(bool)),this,SLOT(on_add_last_button_clicked()));

    connect(&check_box,SIGNAL(stateChanged(int)),this,SLOT(on_check_box_changed()));

    connect(&save,SIGNAL(clicked(bool)),this,SLOT(on_save_button_clicked()));
    connect(&abort,SIGNAL(clicked(bool)),this,SLOT(on_abort_button_clicked()));

    starting_mode(true);
    gmu.clear();
}

std::string gmu_gui::common_prefix( const std::string& a, const std::string& b )
{
    if( a.size() <= b.size() )
        return std::string( a.begin(), std::mismatch( a.begin(), a.end(), b.begin() ).first ) ;
    else
        return std::string( b.begin(), std::mismatch( b.begin(), b.end(), a.begin() ).first ) ;
}

void gmu_gui::update_joint_names()
{
    joint_msg.name.clear();
    
    // find prefix
    std::string prefix;
    std::string new_prefix("gmu_hand_");
    int number_of_joints = grasp_msg.grasp_trajectory.joint_names.size();
    if(number_of_joints == 0)
        return;
    if(number_of_joints == 1)
    {
        std::string hand("hand_");
        std::size_t end_prefix = grasp_msg.grasp_trajectory.joint_names.at(0).find(hand);
        end_prefix += hand.length();
        prefix = grasp_msg.grasp_trajectory.joint_names.at(0).substr(0,end_prefix);
    }
    else
    {
        prefix = grasp_msg.grasp_trajectory.joint_names.at(0);
        for(int i=1;i<number_of_joints;i++)
            prefix = common_prefix(prefix,grasp_msg.grasp_trajectory.joint_names.at(i));
    }
    
    for(int i=0;i<number_of_joints;i++)
    {
        const std::string& lab = grasp_msg.grasp_trajectory.joint_names.at(i);
        
        // HACK: replace the prefix - still not fully general, but working better
        std::string tmp(lab);
        tmp.replace(0,prefix.length(),new_prefix);
        
        joint_msg.name.push_back(tmp);
    }
}

void gmu_gui::update_sliders(int number_of_joints)
{
    QLayoutItem* child;
    while((child = layout4.takeAt(0)) != 0) delete child;

    for(auto l:synergy_label) delete l;
    for(auto s:synergy_slider) delete s;

    synergy_label.clear();
    synergy_slider.clear();

    for(int i=0;i<number_of_joints;i++)
    {
        const std::string& lab = grasp_msg.grasp_trajectory.joint_names.at(i);
        // // NOTE: the label shows up as in the message, but the published joints are different;
        // //       uncomment the next line to show in the label what gets published instead
        // std::string lab = joint_msg.name.at(i);

	QLabel* label = new QLabel(QString::fromStdString(lab));
	QSlider* slider = new QSlider();
	slider->setOrientation(Qt::Horizontal);
	slider->setRange(0,100);
	slider->setSliderPosition(grasp_msg.grasp_trajectory.points.at(current_wp).positions.at(i)*100);
    
	synergy_label.push_back(label);
	synergy_slider.push_back(slider);

	QHBoxLayout* layout = new QHBoxLayout();
	layout->addWidget(label);
	layout->addWidget(slider);
	layout4.addLayout(layout);

	connect(synergy_slider.at(i), SIGNAL(valueChanged(int)),&slider_mapper, SLOT(map())) ;
	slider_mapper.setMapping(synergy_slider.at(i), i) ;
    }

    if(number_of_joints!=0)
    {
	connect(&slider_mapper, SIGNAL(mapped(int)), this, SLOT(on_slider_moved(int))) ;
	QHBoxLayout* layout = new QHBoxLayout();
	layout->addWidget(&time_from_start_label);
	layout->addWidget(&time_from_start_text);
	time_from_start_text.setText(QString::number(grasp_msg.grasp_trajectory.points.at(current_wp).time_from_start.toSec(), 'f', 2));
	layout4.addLayout(layout);
	on_slider_moved(0);
    }
}

void gmu_gui::toggle_top_layout(bool enable)
{
    object_text.setEnabled(enable);
    grasp_id_text.setEnabled(enable);
    edit.setEnabled(enable);
    copy.setEnabled(enable);
}

void gmu_gui::toggle_middle_layouts(bool enable)
{
    waypoint_selection.setEnabled(enable);
    delete_button.setEnabled(enable);
    add_before_button.setEnabled(enable);
    add_last_button.setEnabled(enable);
    for(auto text:coord_text) text.second->setEnabled(enable);
    for(auto slid:synergy_slider) slid->setEnabled(enable);
    time_from_start_text.setEnabled(enable);
    check_box.setEnabled(enable);
}

void gmu_gui::toggle_bottom_layout(bool enable)
{
    save.setEnabled(enable);
    abort.setEnabled(enable);
}

void gmu_gui::starting_mode(bool starting)
{
    toggle_top_layout(starting);
    toggle_middle_layouts(!starting);
    toggle_bottom_layout(!starting);
}

bool gmu_gui::initialize_gmu()
{
    obj_id = object_text.text().toDouble();
    grasp_id = grasp_id_text.text().toDouble();
//     file_name = "object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id );
    
//     if( deserialize_ik( grasp_msg, file_name ) )
    if(read_grasp_msg(obj_id, grasp_id, grasp_msg))
    {
	ROS_INFO_STREAM("Deserialization object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id ) << " OK! ... You can modify the grasp in Rviz");
	std::cout << "grasp_msg" << grasp_msg << std::endl;
	if( !gmu.db_mapper->Objects.count( obj_id ) )
	{
	    ROS_ERROR_STREAM("Object " << grasp_msg.object_db_id << " is not in the database! . . . Retry!");
	    return false;
	}
    }
    else
    {
	ROS_ERROR_STREAM("Error in deserialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Retry!");
	return false;
    }
    
    // normalize all the poses in the message (to be sure everything will work in KDL)
    normalizePoses(grasp_msg.ee_pose);
    normalizePoses(grasp_msg.attObject.object.mesh_poses);

    // // TEST: see if everything works well if using completely different joint names
    // grasp_msg.grasp_trajectory.joint_names.clear();
    // grasp_msg.grasp_trajectory.joint_names.push_back("hamal_test_hand_joint1");
    // grasp_msg.grasp_trajectory.joint_names.push_back("hamal_test_hand_joint2");
    // grasp_msg.grasp_trajectory.joint_names.push_back("hamal_test_hand_some_other_joint_3");
    // for(auto& pt:grasp_msg.grasp_trajectory.points)
    // {
    //     // add two more joints in each message
    //     pt.positions.push_back(0.1);
    //     pt.positions.push_back(0.9);
    // }
        
    // HACK: change names to publish the right joints
    update_joint_names();
    update_sliders(grasp_msg.grasp_trajectory.joint_names.size());
    
    gmu.set_object(obj_id);
    KDL::Frame obj_hand_postGrasp;
    tf::poseMsgToKDL( grasp_msg.attObject.object.mesh_poses.front(), obj_hand_postGrasp );
    obj_hand_postGrasp = obj_hand_postGrasp.Inverse();
    geometry_msgs::Pose postGrasp_pose;
    tf::poseKDLToMsg( obj_hand_postGrasp, postGrasp_pose );
    gmu.set_hands( grasp_msg.ee_pose, postGrasp_pose );

    waypoint_selection.clear();
    waypoint_selection.addItem(QString::number(0));
    for(int i=1;i<grasp_msg.ee_pose.size();i++) waypoint_selection.addItem(QString::number(i));

    if(waypoint_selection.count()==1) delete_button.setEnabled(false);
    gmu.publish_object();
    gmu.publish_hands();

    return true;
}

void gmu_gui::on_text_changed(const int& id)
{
    geometry_msgs::Pose hand = gmu.get_wp(current_wp);

    if(id==0) hand.position.x = coord_text.at(id)->text().toDouble();
    if(id==1) hand.position.y = coord_text.at(id)->text().toDouble();
    if(id==2) hand.position.z = coord_text.at(id)->text().toDouble();

    if(id>2)
    {
	double ro,pi,ya;
	tf::Quaternion q;
	tf::quaternionMsgToTF(hand.orientation,q);
	tf::Matrix3x3(q).getRPY(ro,pi,ya);

	if(id==3) q.setRPY(coord_text.at(id)->text().toDouble(),pi,ya);
	if(id==4) q.setRPY(ro,coord_text.at(id)->text().toDouble(),ya);
	if(id==5) q.setRPY(ro,pi,coord_text.at(id)->text().toDouble());

	tf::quaternionTFToMsg(q,hand.orientation);
    }

    normalizePose(hand);
    gmu.set_wp(current_wp,hand);
    gmu.publish_hands();
}

void gmu_gui::on_edit_button_clicked()
{
    starting_mode(false);
    editing = true;
    if(!initialize_gmu()) on_abort_button_clicked();
}

void gmu_gui::on_copy_button_clicked()
{
    starting_mode(false);
    editing = false;
    if(!initialize_gmu()) on_abort_button_clicked();
}

void gmu_gui::on_waypoint_selection_changed()
{
    current_wp = waypoint_selection.currentText().toInt();
    gmu.setCurrentWaypoint(current_wp);
    gmu.force_im_update();
    gmu.publish_hands();

    update_coords(gmu.get_wp(current_wp));
    update_joints_info();
}

void gmu_gui::update_joints_info()
{
    for(int i=0;i<grasp_msg.grasp_trajectory.joint_names.size();i++)
    {
	synergy_slider.at(i)->setSliderPosition(grasp_msg.grasp_trajectory.points.at(current_wp).positions.at(i)*100);
    }
    time_from_start_text.setText(QString::number(grasp_msg.grasp_trajectory.points.at(current_wp).time_from_start.toSec(), 'f', 2));
    publish_joint_state();
}

void gmu_gui::update_coords(geometry_msgs::Pose wp)
{
    coord_text.at(0)->setText(QString::number(wp.position.x, 'f', 2));
    coord_text.at(1)->setText(QString::number(wp.position.y, 'f', 2));
    coord_text.at(2)->setText(QString::number(wp.position.z, 'f', 2));
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(wp.orientation,q);
    tf::Matrix3x3 M(q);
    double roll,pitch,yaw;
    M.getRPY(roll,pitch,yaw);

    coord_text.at(3)->setText(QString::number(roll, 'f', 2));
    coord_text.at(4)->setText(QString::number(pitch, 'f', 2));
    coord_text.at(5)->setText(QString::number(yaw, 'f', 2));
}

void gmu_gui::im_callback(const visualization_msgs::InteractiveMarkerFeedback& feedback)
{
    if(feedback.marker_name!="hands") return;

    static tf::TransformListener tf;
    static tf::Transform transform_;

    tf::StampedTransform hand_palm;
    double timeout = 5.0;
    if(!tf.waitForTransform("gmu_hand_palm_link","hand",ros::Time(0), ros::Duration(timeout)))
	hand_palm.setIdentity();
    else
	tf.lookupTransform("gmu_hand_palm_link","hand", ros::Time(0), hand_palm);

    transform_.setOrigin( tf::Vector3(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z) );
    transform_.setRotation( tf::Quaternion( feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w) );
    transform_.mult(transform_,hand_palm);

    update_coords(feedback.pose);
}

void gmu_gui::on_delete_button_clicked()
{
    gmu.delete_wp(current_wp);
    delete_joint_wp();
    int last = waypoint_selection.count()-1;
    waypoint_selection.removeItem(last);
    gmu.publish_hands();
    gmu.force_im_update();

    if(last==1) delete_button.setEnabled(false);
}

void gmu_gui::on_add_before_button_clicked()
{
    gmu.add_extra_wp(current_wp);
    add_joint_wp();
    int last = waypoint_selection.count()-1;
    waypoint_selection.addItem(QString::number(last+1));
    delete_button.setEnabled(true);
    gmu.publish_hands();
    gmu.force_im_update();
}

void gmu_gui::on_add_last_button_clicked()
{
    int last = waypoint_selection.count()-1;
    gmu.add_extra_wp(last);
    add_joint_wp();
    waypoint_selection.addItem(QString::number(last+1));
    waypoint_selection.setCurrentIndex(last+1);
    delete_button.setEnabled(true);
    gmu.publish_hands();
    gmu.force_im_update();
}

void gmu_gui::on_check_box_changed()
{
    if(check_box.checkState() == Qt::Checked)
    {
	gmu.toggle_objects_interaction(true);
    }

    if(check_box.checkState() == Qt::Unchecked)
    {
	gmu.toggle_objects_interaction(false);
    }

    gmu.publish_object();
}

void gmu_gui::delete_joint_wp()
{
    grasp_msg.grasp_trajectory.points.erase(grasp_msg.grasp_trajectory.points.begin()+current_wp);
}

void gmu_gui::add_joint_wp()
{
    trajectory_msgs::JointTrajectory new_traj =  grasp_msg.grasp_trajectory;
    new_traj.points.clear();

    for(int i=0;i<grasp_msg.grasp_trajectory.points.size();i++)
    {
	new_traj.points.push_back(grasp_msg.grasp_trajectory.points.at(i));
	if(i==current_wp) new_traj.points.push_back(grasp_msg.grasp_trajectory.points.at(i));
    }

    grasp_msg.grasp_trajectory=new_traj;
}

void gmu_gui::publish_joint_state()
{
    joint_msg.position.clear();

    for(auto q:grasp_msg.grasp_trajectory.points.at(current_wp).positions) joint_msg.position.push_back(q);

    joint_pub.publish(joint_msg);
}

void gmu_gui::on_slider_moved(const int& id)
{
    grasp_msg.grasp_trajectory.points.at(current_wp).positions.at(id) = (double)(synergy_slider.at(id)->value()/100.0);

    publish_joint_state();
}

void gmu_gui::on_time_from_start_changed()
{
    ros::Duration time(time_from_start_text.text().toDouble());
    grasp_msg.grasp_trajectory.points.at(current_wp).time_from_start = time;
}

void gmu_gui::on_save_button_clicked()
{
    geometry_msgs::Pose object,final_object, final_hand;
    gmu.get_object(object, final_object);
    gmu.get_hands(grasp_msg.ee_pose, final_hand);
    
    // normalize again all poses
    normalizePose(object);
    normalizePose(final_object);
    normalizePose(final_hand);
    normalizePoses(grasp_msg.ee_pose);

    KDL::Frame world_object, world_hand;
    tf::poseMsgToKDL(object, world_object);
    
    for( int j=0; j < grasp_msg.ee_pose.size(); j++ ) //if we have changed the object position we want to update all the hand position w.r.t. the object
    {
	tf::poseMsgToKDL(grasp_msg.ee_pose.at(j),world_hand);
	tf::poseKDLToMsg(world_object.Inverse()*world_hand,grasp_msg.ee_pose.at(j));
    }
    
    // also modify post-grasp pose
    tf::poseMsgToKDL(final_object,world_object);
    tf::poseMsgToKDL(final_hand,world_hand);
    tf::poseKDLToMsg(world_hand.Inverse()*world_object,grasp_msg.attObject.object.mesh_poses.front());

    int actual_grasp_id = compute_grasp_id(obj_id,grasp_id);

    int new_grasp_id = gmu.db_writer->checkGraspId(actual_grasp_id);
    
    if(!editing)
    {
        std::cout << "new grasp_id: " << new_grasp_id << std::endl;
        std::cout << "copying name: " << std::get<2>( gmu.db_mapper->Grasps.at(actual_grasp_id) ) << std::endl;

        std::string new_grasp_name = std::get<2>( gmu.db_mapper->Grasps.at( actual_grasp_id ) ) + " (copy)";
        std::cout << "new name is: " << new_grasp_name << std::endl;

        int ee_id = std::get<1>( gmu.db_mapper->Grasps.at( actual_grasp_id ) );
        std::cout << "copying ee_id: " << ee_id << std::endl;
        std::cout << "writing to database..." << std::endl;

        if(gmu.db_writer->writeNewGrasp( new_grasp_id, obj_id, ee_id, new_grasp_name) <= 0)
        {
            ROS_ERROR_STREAM("Couldn't write the database");
            return;
        }

    }
    
    if(write_grasp_msg(obj_id, new_grasp_id, grasp_msg) < 0)
    {
	ROS_ERROR_STREAM("Error in serialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(new_grasp_id) << "! . . . Aborting!");
        gmu.db_writer->deleteGrasp(new_grasp_id);
        on_abort_button_clicked();
        return;
    }
    else
    {
	ROS_INFO_STREAM("Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!");
    }

    on_abort_button_clicked();
}

void gmu_gui::on_abort_button_clicked() //this is also used to reset everything
{
    ROS_INFO_STREAM("Resetting everything.");
    starting_mode(true);
    gmu.clear();
}

gmu_gui::~gmu_gui()
{

}
