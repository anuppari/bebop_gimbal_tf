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
	ros::Subscriber gimbalSub_;
	ros::Subscriber mocapSub_;
	std::string bebopName_;
	std::string bebopImageName_;
	tfScalar pan_; // yaw, in radians
	tfScalar tilt_; // pitch, in radians

public:
	bebop_gimbal_tf(tf::Transform staticTransform): pan_(0), tilt_(0), staticTransform_(staticTransform)
	{
		// Get parameters
		nh_.param<std::string>("bebop_mocap_name", bebopName_, "bebop");
		nh_.param<std::string>("bebop_image_name", bebopImageName_, "bebop_image");
		
		// Bebop camera gimbal state subscriber
		gimbalSub_ = nh_.subscribe("bebop/states/ARDrone3/CameraState/Orientation",10,&bebop_gimbal_tf::camStateCB,this);
		
		// Mocap subscriber
		mocapSub_ = nh_.subscribe(bebopName_+"/pose",10,&bebop_gimbal_tf::poseCB,this);
	}
	
	void poseCB(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		// Transform of mocap rigid body w.r.t. world, i.e., P_world = transform*P_body
		tf::Transform transform(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w),
								tf::Vector3(0,0,0));
		
		// Transform of camera w.r.t. world, i.e., P_world = transform*P_camera.
		// Additional matrix is to compensate for the fact that neutral camera has z out, x right, while neutral rpy has x out, y left
		tf::Transform camImageTransform = tf::Transform(tf::Matrix3x3(0,-1,0, 0,0,-1, 1,0,0),tf::Vector3(0,0,0));
		transform *= staticTransform_*camImageTransform;
		
		// Get roll, pitch, yaw of transform
		tfScalar roll, pitch, yaw;
		transform.getBasis().getEulerYPR(yaw, pitch, roll);
		
		// Adjust transform to compensate for gimbal, i.e., undoing roll and pitch, add back in pan and tilt
		tf::Matrix3x3 rollMat;
		rollMat.setEulerYPR(0,0,-roll);
		tf::Matrix3x3 pitchMat;
		pitchMat.setEulerYPR(0,-pitch,0);
		tf::Matrix3x3 panTiltMat;
		panTiltMat.setEulerYPR(pan_,tilt_,0);
		tf::Transform panTiltTransform(panTiltMat,tf::Vector3(0,0,0));
		
		tf::Transform newTransform = staticTransform_*camImageTransform; // start with nominal/flat trim transform
		newTransform *= tf::Transform(rollMat,tf::Vector3(0,0,0))*tf::Transform(pitchMat,tf::Vector3(0,0,0)); // undo roll and pitch
		newTransform *= panTiltTransform*camImageTransform.inverse(); // compensate for pan tilt setting and camera neutral
		
		// Publish transform
		br_.sendTransform(tf::StampedTransform(newTransform, msg->header.stamp, bebopName_, bebopImageName_));
		br_.sendTransform(tf::StampedTransform(staticTransform_*panTiltTransform, msg->header.stamp, bebopName_, bebopImageName_+"_static"));
	}
	
	void camStateCB(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
	{
		pan_ = -1*msg->pan*M_PI/180.0;
		tilt_ = -1*msg->tilt*M_PI/180.0;
	}
}; // end bebop_gimbal_tf

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bebop_gimbal_tf_node");
    
    tf::Transform staticTransform;
    
    if (argc == 8)
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
