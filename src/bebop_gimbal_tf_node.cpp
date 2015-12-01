#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>

class bebop_gimbal_tf
{
private:
	ros::NodeHandle nh_;
	tf::TransformBroadcaster br_;
	tf::Transform
	int pan_; // yaw
	int tilt_; // pitch

public:
	bebop_gimbal_tf(): pan(0), tilt(0)
	{
		//Mocap subscriber
		nh.subscribe("pose",10,&bebop_gimbal_tf::poseCB,this);
		
		// Bebop camera gimbal state subscriber
		nh.subscribe("bebop/states/ARDrone3/CameraState/Orientation",10,&bebop_gimbal_tf::camStateCB,this);
	}
	
	void poseCB(const geometry_msgs::PoseStampedConstPtr& pose)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, msg->theta);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
	}
	
	void camStateCB(const bebop_msgs::Ardrone3CameraStateOrientation::ConstPtr& msg)
	{
		pan_ = msg->pan;
		tilt_ = msg->tilt;
	}
} // end bebop_gimbal_tf

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
    
    bebop_gimbal_tf obj;
    
    ros::spin();
    return 0;
}
