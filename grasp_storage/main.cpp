#include "grasp_storage.h"
#include "ros/ros.h"
#include <fstream>

#define DB_NAME "containerB.db"

bool read_from_file(std::string filename, std::vector<ros::Time>& v)
{
  std::fstream fs;
  // fs.open (path, std::fstream::in | std::fstream::out | std::fstream::app*/);
  fs.open (filename, std::fstream::in);
  if (fs.fail())
      return false;
  
  while(1)
  {
    double tmp;
    fs >> tmp;
    if(fs.eof())
      break;
    v.push_back(ros::Time(tmp));
  }
  
  fs.close();
  return true;
}

int main(int argc, char** argv)
{
  if(argc < 5)
  {
    std::cout<<std::endl;
    std::cout<<" - Usage[1]:   ./grasp_storage    object_id    end_effector_id    grasp_name    snapshot_time0    snapshot_time1 ..."<<std::endl<<std::endl;
    std::cout<<"\t - object_id: identifies the object type"<<std::endl;
    std::cout<<"\t - end_effector_id: identifies the e-e"<<std::endl;
    std::cout<<"\t - grasp_name: custom name for grasp (e.g. \"RH_top\")"<<std::endl;
    std::cout<<"\t - snapshot_times (at least 2): timing vector to use for taking snapshots"<<std::endl<<std::endl;
    std::cout<<" - Usage[2]:   ./grasp_storage    object_id    end_effector_id    grasp_name    filename"<<std::endl<<std::endl;
    std::cout<<"\t - filename: file from which snapshot times will be read"<<std::endl<<std::endl;
  
    return -1;
  }

  int object_id, end_effector_id;
  std::string grasp_name;
  std::vector<ros::Time> snapshots;
  
  object_id = atoi(argv[1]);
  end_effector_id = atoi(argv[2]);
  grasp_name = argv[3];
  
  if(argc == 5)
  {
    // std::string filename = ros::package::getPath("dual_manipulation_ik_control");
    // filename.append(argv[4]);
    std::string filename = argv[4];
    read_from_file(filename,snapshots);
  }
  else
  {
    double snap_time;
    int counter = 4;
    while(counter < argc)
    {
      snap_time = atoi(argv[counter++]);
      snapshots.push_back(ros::Time(snap_time));
    }
  }
  
  std::cout << "Snapshots: ";
  for(auto s:snapshots)
    std::cout << s.toSec() << " ";
  std::cout << std::endl;
  
  std::cout<<std::endl;
  std::cout<<" - Ready to store grasp for:"<<std::endl<<std::endl;
  std::cout<<"\t - object_id = "<<object_id<<std::endl;
  std::cout<<"\t - end_effector_id = "<<end_effector_id<<std::endl;
  std::cout<<"\t - grasp_name = "<<grasp_name<<std::endl;
  std::cout<<"\t - "<<snapshots.size()<<" snapshots in total"<<std::endl<<std::endl;
  std::cout<<"No data will be saved on disk until the \"!! ATTENTION !!\" messages"<<std::endl<<std::endl;

  ros::init(argc, argv, "grasp_storage");
  
  grasp_storage graspStorage(DB_NAME);
  
  if(!graspStorage.set_grasp_info(object_id, end_effector_id, grasp_name,snapshots))
  {
    ROS_ERROR_STREAM("Unable to set required information in graspStorage - returning...");
    return -1;
  }
  
  char a;

  ROS_INFO("Press 'q' to exit, or any other key to START recording TRAJECTORY: ");
  
  std::cin>>a;
  if(a=='q')
    return 0;
  
  if(!graspStorage.record_grasp())
  {
    ROS_ERROR_STREAM("Unable to record required grasp - returning...");
    return -1;
  }
  
  while(!graspStorage.finished())
    usleep(100000);

  ROS_INFO("!! ATTENTION !! Recording ended, press 'q' to exit without saving or any other key to insert row in the database and generate grasp file: ");

  std::cin>>a;
  if(a=='q')
    return 0;

  if(!graspStorage.save_recording())
  {
    ROS_ERROR_STREAM("Unable to saved recorded information in the DB/file - returning...");
    return -1;
  }
  
  ROS_INFO("Recording done!");
  
  return 0;
}