#define UNIX_BUILD

#include <ros/ros.h> 
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "mscl/mscl.h" 
#include "mscl/Types.h" 
#include "mscl/Communication/Connection.h" 
#include "mscl/MicroStrain/Inertial/InertialNode.h" 
#include "mscl/Exceptions.h" 
#include "mscl/MicroStrain/MIP/MipTypes.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "hello_world_node");
	
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
	ros::Rate loop_rate(31.5);
	
	sensor_msgs::Imu msg_gps_imu;

	try {
		ROS_INFO_STREAM("making connection...");
		mscl::Connection connection = mscl::Connection::Serial("/dev/ttyACM0");
		mscl::InertialNode node(connection);
		node.enableDataStream(mscl::MipTypes::CLASS_AHRS_IMU);

		while (ros::ok()) {
			mscl::MipDataPackets packets = node.getDataPackets(500);
			for (mscl::MipDataPacket packet : packets) {
				packet.descriptorSet();
//				packet.timestamp();

				mscl::MipDataPoints points = packet.data();
				
				for (mscl::MipDataPoint dataPoint : points) {
					dataPoint.channelName();
					dataPoint.as_float();

	                        	sensor_msgs::Imu imu_msg;
        	                	imu_msg.header.stamp = ros::Time::now();
                	        	imu_msg.header.frame_id ="/base_link";
	
        	                	imu_msg.angular_velocity.x = 3;
                	        	imu_msg.angular_velocity.y = dataPoint.as_float();
                       			chatter_pub.publish(imu_msg);
				}
			}
		}
	}

	catch(mscl::Error& e) {
		ROS_FATAL(e.what());
	}

	return 0;
}
