#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>

class bebop_gimbal_tf
{
private:
	ros::NodeHandle nh_;
	tf::TransformBroadcaster br_;
	tf::Transform staticTransform_;
	std::string bebopName_;
	std::string bebopImageName_;
	int pan_; // yaw
	int tilt_; // pitch

public:
	bebop_gimbal_tf(tf::Transform staticTransform): pan_(0), tilt_(0), staticTransform_(staticTransform)
	{
		// Get parameters
		nh_.param<std::string>("bebop_mocap_name", bebopName_, "bebop");
		nh_.param<std::string>("bebop_image_name", bebopImageName_, "bebop_image");
		
		// Bebop camera gimbal state subscriber
		nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation",10,&bebop_gimbal_tf::camStateCB,this);
		
		// Mocap subscriber
		nh_.subscribe("pose",10,&bebop_gimbal_tf::poseCB,this);
	}
	
	void poseCB(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		// Transform of mocap rigid body w.r.t. world, i.e., P_world = transform*P_body
		tf::Transform transform(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w),
								tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
		
		// Transform of camera w.r.t. world, i.e., P_world = transform*P_camera
		transform *= staticTransform_;
		
		// Get roll, pitch, yaw of transform
		tfScalar roll, pitch, yaw;
		transform.getBasis().getEulerYPR(yaw, pitch, roll);
		std::cout << "Built in:" << std::endl;
		std::cout << "\troll:\t" << roll << std::endl;
		std::cout << "\tpitch:\t" << pitch << std::endl;
		std::cout << "\tyaw:\t" << yaw << std::endl;
		
		tf::Quaternion quat = transform.getRotation();
		tfScalar q1 = quat.getX();
		tfScalar q2 = quat.getY();
		tfScalar q3 = quat.getZ();
		tfScalar q0 = quat.getW();
		
		roll = atan2(2*(q0*q1+q2*q3),1-2*(pow(q1,2)+pow(q2,2)));
		pitch = asin(2*(q0*q2-q3*q1));
		yaw = atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2)));
		
		std::cout << "Manual:" << std::endl;
		std::cout << "\troll:\t" << roll << std::endl;
		std::cout << "\tpitch:\t" << pitch << std::endl;
		std::cout << "\tyaw:\t" << yaw << std::endl;
		
		// Adjust transform to compensate for gimbal
		tf::Quaternion rollQuat;
		rollQuat.setEuler(0,0,-roll);
		tf::Quaternion pitchQuat;
		pitchQuat.setEuler(0,-pitch,0);
		tf::Quaternion panTiltQuat;
		panTiltQuat.setEuler(pan_,tilt_,0);
		
		transform *= tf::Transform(rollQuat)*tf::Transform(pitchQuat);
		transform *= tf::Transform(panTiltQuat);
		
		// Publish transform
		br_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, bebopName_, bebopImageName_));
	}
	
	void camStateCB(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
	{
		pan_ = msg->pan;
		tilt_ = msg->tilt;
	}
}; // end bebop_gimbal_tf

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_gimbal_tf_node");
    
    tf::Transform staticTransform;
    
    if (argc == 11)
    {
		staticTransform = tf::Transform(tf::Quaternion(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7])),
										tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
	}
	else
	{
		staticTransform = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
	}
    
    bebop_gimbal_tf obj(staticTransform);
    
    ros::spin();
    return 0;
}
