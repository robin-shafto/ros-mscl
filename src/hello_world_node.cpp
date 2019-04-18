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
#include "tf/tf.h"

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
		float roll = 0;
		float pitch = 0;
		float yaw = 0;

		while (ros::ok()) {
			mscl::MipDataPackets packets = node.getDataPackets(500);
			for (mscl::MipDataPacket packet : packets) {
				packet.descriptorSet();
//				packet.timestamp();

				mscl::MipDataPoints points = packet.data();
				
				for (mscl::MipDataPoint dataPoint : points) {
					dataPoint.as_float();
					dataPoint.channelName();
//					ROS_INFO(dataPoint.channelName().c_str());
					if (strcmp(dataPoint.channelName().c_str(), "roll") == 0) {
						roll = dataPoint.as_float() / 3.1;
					}
					else if (strcmp(dataPoint.channelName().c_str(), "pitch") == 0) {
						pitch = dataPoint.as_float() / 3.1;
					}
					else {
						yaw = dataPoint.as_float() / 3.1;
					}
				}

				sensor_msgs::Imu imu_msg;
                                imu_msg.header.stamp = ros::Time::now();
                                imu_msg.header.frame_id ="/base_link";

//`                                tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
//                                imu_msg.orientation.x = q.x();
//                                imu_msg.orientation.y = q.y();
//                                imu_msg.orientation.z = q.z();
//                                imu_msg.orientation.w = q.w();
                                imu_msg.orientation.x = roll;
				imu_msg.orientation.y = pitch;
				imu_msg.orientation.z = yaw;
				chatter_pub.publish(imu_msg);
			}
		}
	}

	catch(mscl::Error& e) {
		ROS_FATAL(e.what());
	}

	return 0;
}
