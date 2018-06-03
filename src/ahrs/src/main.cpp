#include <string>
#include <iostream>
#include <fstream>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>

#include "ahrs.h"
#include "imu_filter.h"

using namespace std;

template <typename Out>
void split(const std::string &s, char delim, Out result){
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)){
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim){
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

void func_1_algo(string pathBag){
    //AHRS algo;
    imu_filter algo;
    string imuTopic = "imu0";
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);
    rosbag::View view(bag);
    ros::Rate r(100);
    unsigned int msg_count=0;
    for (const rosbag::MessageInstance& m : view) {
        if(!ros::ok()){
            break;
        }
        sensor_msgs::Imu::ConstPtr s1 = m.instantiate<sensor_msgs::Imu>();
        if (s1 != NULL && m.getTopic() == imuTopic) {
            if(msg_count==0){
                msg_count=s1->header.seq;
            }
            if(msg_count!=s1->header.seq){
                std::cout<<"lose one imu data"<<std::endl;
            }
            
            imu_data item;
            item.id=msg_count;
            item.time=s1->header.stamp.toSec();
            item.ax=s1->linear_acceleration.x;
            item.ay=s1->linear_acceleration.y;
            item.az=s1->linear_acceleration.z;
            item.gx=s1->angular_velocity.x;
            item.gy=s1->angular_velocity.y;
            item.gz=s1->angular_velocity.z;
            //std::cout<<std::setprecision(20)<<item.ax<<std::endl;
            static double last_time=-1;
            static unsigned int last_id=0;
            if(last_time==-1){
                last_time=item.time;
                last_id=s1->header.seq;
            }else{
                if(item.time<last_time){
                    std::cout<<"time order issue!!"<<std::endl;
                    continue;
                }
                if(s1->header.seq<=last_id){
                    //std::cout<<"id order issue!!"<<std::endl;
                    //continue;
                }
                algo.processIMU(item);
                last_time=item.time;
                last_id=s1->header.seq;
            }
            r.sleep();
            msg_count++;
        }
    }
    std::cout<<"get "<<msg_count<<" imu."<<std::endl;
}

void func_2_make_bag(string pathtxt){
    rosbag::Bag bag;
    bag.open("imu_data.bag", rosbag::bagmode::Write);
    std::ifstream re_file(pathtxt);
    while (true){
        std::string str;
        std::getline(re_file, str);
        if (str == ""){
            break;
        }else{
            std::vector<std::string> split_re = split(str, ',');
            if (split_re.size() !=7){
                std::cout << "txt file error!!" << std::endl;
                break;
            }
            double time_stamp=atof(split_re[0].c_str())/1e15;
            double gx = atof(split_re[1].c_str());
            double gy = atof(split_re[2].c_str());
            double gz = atof(split_re[3].c_str());
            double ax = atof(split_re[4].c_str());
            double ay = atof(split_re[5].c_str());
            double az = atof(split_re[6].c_str());
            sensor_msgs::Imu msg;
            msg.angular_velocity.x=gx;
            msg.angular_velocity.y=gy;
            msg.angular_velocity.z=gz;
            msg.linear_acceleration.x=ax;
            msg.linear_acceleration.y=ay;
            msg.linear_acceleration.z=az;
            static int imu_data_seq=0;
            msg.header.seq=imu_data_seq;
            msg.header.stamp= ros::Time(time_stamp);
            imu_data_seq++;
            ros::Time ros_time(ros::Time::now());
            bag.write("imu0", ros_time, msg);
        }
    }
    bag.close();
}

void func_3_savetxt(string pathBag){
    string imuTopic = "imu0";
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);
    rosbag::View view(bag);
    unsigned int msg_count=0;
    FILE *m_fpImu;
    m_fpImu = fopen("imu_data.txt", "w+");
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::Imu::ConstPtr s1 = m.instantiate<sensor_msgs::Imu>();
        if (s1 != NULL && m.getTopic() == imuTopic) {
            if(msg_count==0){
                msg_count=s1->header.seq;
            }
            if(msg_count!=s1->header.seq){
                std::cout<<"lose one imu data"<<std::endl;
            }
            
            imu_data item;
            item.time=s1->header.stamp.toSec();
            item.ax=s1->linear_acceleration.x;
            item.ay=s1->linear_acceleration.y;
            item.az=s1->linear_acceleration.z;
            item.gx=s1->angular_velocity.x;
            item.gy=s1->angular_velocity.y;
            item.gz=s1->angular_velocity.z;
            //std::cout<<std::setprecision(20)<<item.ax<<std::endl;
            fprintf(m_fpImu, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", item.time, item.gx, item.gy, item.gz, item.ax, item.ay, item.az);
            msg_count++;
        }
    }
    std::cout<<"get "<<msg_count<<" imu."<<std::endl;
}

int main(int argc, char **argv) {
    ROS_INFO("Starting up");
    if(argc < 2) {
        ROS_ERROR("Error please specify a rosbag file");
        ROS_ERROR("Command Example: rosrun bagconvert bagconvert <rosbag> <func>");
        return EXIT_FAILURE;
    }
    
    ros::init(argc, argv, "ahrs");
    ros::NodeHandle n;
    std::string func_id=argv[2];
    
    string pathBag = argv[1];
    if(func_id=="1"){
        func_1_algo(argv[1]);
    }else if(func_id=="2"){
        func_2_make_bag(argv[1]);
    }else if(func_id=="3"){
        func_3_savetxt(argv[1]);
    }
    return EXIT_SUCCESS;
}


