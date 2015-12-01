#include "GMU_gui.h"
#include "normalization_utils.cpp"
#include "dual_manipulation_shared/serialization_utils.h"
#include "tf_conversions/tf_kdl.h"
#include "tf/tf.h"

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

	connect(coord_text.at(i), SIGNAL(textEdited(QString)),&signalMapper, SLOT(map())) ;
	signalMapper.setMapping(coord_text.at(i), i) ;
    }
    connect(&signalMapper, SIGNAL(mapped(int)), this, SLOT(on_text_changed(int))) ; 

    layout3.addWidget(&pos_label,0,0);
    layout3.addWidget(&or_label,0,3);

    synergy_label.setText("synergy joint");
    synergy_slider.setOrientation(Qt::Horizontal);
    layout4.addWidget(&synergy_label);
    layout4.addWidget(&synergy_slider);

    save.setText("SAVE");
    abort.setText("ABORT");
    layout5.addWidget(&save);
    layout5.addWidget(&abort);

    main_layout.addLayout(&layout1);
    insertSeparator(&main_layout);
    main_layout.addLayout(&layout2);
    main_layout.addLayout(&layout3);
    main_layout.addLayout(&layout4);
    insertSeparator(&main_layout);
    main_layout.addLayout(&layout5);

    setLayout(&main_layout);

    connect(&edit,SIGNAL(clicked(bool)),this,SLOT(on_edit_button_clicked()));
    connect(&copy,SIGNAL(clicked(bool)),this,SLOT(on_copy_button_clicked()));

    connect(&waypoint_selection,SIGNAL(currentIndexChanged(int)),this,SLOT(on_waypoint_selection_changed()));
    connect(&delete_button,SIGNAL(clicked(bool)),this,SLOT(on_delete_button_clicked()));
    connect(&add_before_button,SIGNAL(clicked(bool)),this,SLOT(on_add_before_button_clicked()));
    connect(&add_last_button,SIGNAL(clicked(bool)),this,SLOT(on_add_last_button_clicked()));

    connect(&save,SIGNAL(clicked(bool)),this,SLOT(on_save_button_clicked()));
    connect(&abort,SIGNAL(clicked(bool)),this,SLOT(on_abort_button_clicked()));

    starting_mode(true);
    gmu.clear();
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
    synergy_slider.setEnabled(enable);
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
    file_name = "object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id );
    
    if( deserialize_ik( grasp_msg, file_name ) )
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
    gmu.publish_hands();

    update_coords(gmu.get_wp(current_wp));
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
    int last = waypoint_selection.count()-1;
    waypoint_selection.removeItem(last);
    gmu.publish_hands();

    if(last==1) delete_button.setEnabled(false);
}

void gmu_gui::on_add_before_button_clicked()
{
    gmu.add_extra_wp(current_wp);
    int last = waypoint_selection.count()-1;
    waypoint_selection.addItem(QString::number(last+1));
    delete_button.setEnabled(true);
    gmu.publish_hands();
}

void gmu_gui::on_add_last_button_clicked()
{
    int last = waypoint_selection.count()-1;
    gmu.add_extra_wp(last);
    waypoint_selection.addItem(QString::number(last+1));
    delete_button.setEnabled(true);
    gmu.publish_hands();
}

void gmu_gui::on_save_button_clicked()
{
    if(!editing)
    {
	std::cout << "grasp_id: " << grasp_id << std::endl;
	std::cout << "copying name: " << std::get<2>( gmu.db_mapper->Grasps.at(grasp_id) ) << std::endl;

	std::string new_grasp_name = std::get<2>( gmu.db_mapper->Grasps.at( grasp_id ) ) + " (copy)";
	std::cout << "new name is: " << new_grasp_name << std::endl;

	int ee_id = std::get<1>( gmu.db_mapper->Grasps.at( grasp_id ) );
	std::cout << "copying ee_id: " << ee_id << std::endl;
	std::cout << "writing to database..." << std::endl;

	grasp_id = gmu.db_writer->writeNewGrasp( obj_id, ee_id, new_grasp_name);

	if(!(grasp_id > 0) )
	    ROS_ERROR_STREAM("Couldn't write the database");

	file_name = "object" + std::to_string( obj_id ) + "/grasp" + std::to_string( grasp_id );
    }

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

    if( !serialize_ik( grasp_msg, file_name ) )
    {
	ROS_ERROR_STREAM("Error in serialization object" + std::to_string(obj_id) + "/grasp" + std::to_string(grasp_id) << "! . . . Aborting!");
    }
    else
    {
	ROS_INFO_STREAM("Serialization object" + std::to_string(obj_id) << "/grasp" << grasp_id << " OK!");
    }

    on_abort_button_clicked();
}

void gmu_gui::on_abort_button_clicked() //this is also used to reset everything
{
    ROS_INFO_STREAM("Aborting, resetting everything.");
    starting_mode(true);
    gmu.clear();
}

gmu_gui::~gmu_gui()
{

}
