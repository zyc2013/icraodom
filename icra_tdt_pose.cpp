#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Imu.h>
#include <math.h>

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double x_base = 0.0;
double y_base = 0.0;
double g_vel_x = 0.0;
double g_vel_y = 0.0;
double linear_velocity_x_last,linear_velocity_y_last,angular_velocity_last;
double angular_scale=1.0, linear_scale=1.0;
double g_vel_dt = 0.0;
double g_imu_dt = 0.0;
double g_imu_z = 0.0;
double first_x=0.0,first_y=0.0,first_ang=0.0;
double linear_velocity_x,linear_velocity_y,angular_velocity;
double delta_theta,delta_x,delta_y;
double uwb_x_pos;
double uwb_y_pos;
double uwb_angular;
int overround=0;
ros::Time g_last_loop_time(0.0);
ros::Time g_last_vel_time(0.0);
ros::Time g_last_imu_time(0.0);
#define DEBUG 1
void velCallback( const geometry_msgs::Twist& vel) {
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();
  //ROS_INFO("%f,%f,%f",vel.linear.x,vel.linear.y,vel.angular.z);//9-1
  g_vel_x = vel.linear.x-first_x;
  g_vel_y = vel.linear.y-first_y;
  g_imu_z = vel.angular.z-6.2832*overround;
 if(g_imu_z > 6.2832)
  {
overround++;
  }
else if(g_imu_z<0)
{
overround--;
  }
  g_vel_dt = (current_time - g_last_vel_time).toSec();
  g_imu_dt = (current_time - g_last_vel_time).toSec();

  if(g_last_vel_time.toSec()==0)
{
g_vel_dt=0;
g_imu_dt=0;
linear_velocity_x_last=0;
linear_velocity_y_last=0;
angular_velocity_last=0;
linear_velocity_x=0;
linear_velocity_y=0;
angular_velocity=0;
first_x=vel.linear.x;
first_y=vel.linear.y;
first_ang=g_imu_z;
}
/*
//linear velocity is the linear velocity published from the Teensy board in x axis
   linear_velocity_x =g_vel_x;

    //linear velocity is the linear velocity published from the Teensy board in y axis
   linear_velocity_y = g_vel_y;

    //angular velocity is the rotation in Z from imu_filter_madgwick's output
    angular_velocity = g_imu_z;
//ROS_INFO("%f,%f,%f",linear_velocity_x,linear_velocity_y,angular_velocity);//9-1
    //calculate angular displacement  θ = ω * t
   delta_theta = 
     angular_velocity * g_imu_dt * angular_scale; //radians
    delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * g_vel_dt * linear_scale; //m
    delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * g_vel_dt * linear_scale; //m
	//ROS_INFO("%f,%f,%f",delta_theta ,delta_x,delta_y);//9-1
    //calculate current position of the robot
*/
else
{
//linear_velocity_x=(vel.linear.x-linear_velocity_x_last)/g_vel_dt;
/*static float x1=0,x2=0;
float dx1,dx2;
float R = 50;
dx1 = x2;
dx2 = -R*R*(x1-vel.linear.x) -2*R*x2;
x1 = x1 + dx1*g_vel_dt;
x2 = x2 + dx2*g_vel_dt;
ROS_INFO("x1:%f,x2:%f,dx1:%f,dx2:%f",x1,x2,dx1,dx2);//9-1
linear_velocity_x = x2;*/
static float last_vx=0,last_vy=0,last_vw=0;
float a = 1.0;
linear_velocity_x=a*(g_vel_x-linear_velocity_x_last)/g_vel_dt + (1-a)*last_vx;
last_vx = linear_velocity_x;
linear_velocity_y=a*(g_vel_y-linear_velocity_y_last)/g_vel_dt+ (1-a)*last_vy;
last_vy=linear_velocity_y;
angular_velocity=(g_imu_z-angular_velocity_last)/g_vel_dt+(1-a)*last_vw;
last_vw=angular_velocity;
}
#ifdef DEBUG
//    ROS_INFO("x_vel:%f,y_vel:%f,theta_vel:%f,dalta_t:%f",linear_velocity_x,linear_velocity_y,angular_velocity,g_vel_dt);//9-1
  //  ROS_INFO("x_pos:%f,y_pos:%f,theta_pos:%f,dalta_t:%f",g_vel_x,g_vel_y,g_imu_z,g_vel_dt);//9-1
#endif
linear_velocity_x_last=g_vel_x;
linear_velocity_y_last=g_vel_y;
angular_velocity_last=g_imu_z;
    x_pos = g_vel_x-0.275;
    y_pos = g_vel_y;
    theta = g_imu_z;
    x_base=x_pos+0.275*cos(theta);
    y_base=y_pos+0.275*sin(theta);
    //ROS_INFO("x_pos:%f,y_pos:%f,theta:%f",x_pos,y_pos,theta);//9-1
g_last_vel_time = current_time;
}
void uwbCallback(const geometry_msgs::Twist& pos){
//tf树
uwb_x_pos=pos.linear.x;
uwb_y_pos=pos.linear.y;
uwb_angular=pos.angular.z;
}
int main(int argc, char** argv){
  geometry_msgs::Twist base_vel;
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_pos", 50, velCallback);
  ros::Publisher pub=n.advertise<geometry_msgs::Twist>("raw_vel", 1000);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 50, IMUCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber uwb_pose_sub_=n.subscribe("uwb_msg",2,uwbCallback);
  nh_private_.getParam("angular_scale", angular_scale);
  nh_private_.getParam("linear_scale", linear_scale);

  double rate = 10.0;
  

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::Time current_time = ros::Time::now();

    
    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::Quaternion base_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::Quaternion uwb_quat = tf::createQuaternionMsgFromYaw(uwb_angular);     
    geometry_msgs::TransformStamped odom_trans,base_center,uwb_pos;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "orthgonality_wheel";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(odom_trans);
	base_center.header.frame_id="odom";
	base_center.child_frame_id = "base_footprint";
	base_center.transform.translation.x =x_base;
	base_center.transform.translation.y =y_base;
	base_center.transform.translation.z =0.0;
	base_center.transform.rotation = base_quat ;
        base_center.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(base_center);
    base_vel.linear.x=linear_velocity_x;
    base_vel.linear.y=linear_velocity_y;
    base_vel.angular.z=angular_velocity;
    pub.publish(base_vel);

 uwb_pos.header.frame_id = "odom";
     uwb_pos.child_frame_id = "uwb";
    uwb_pos.transform.translation.x = uwb_x_pos;
    uwb_pos.transform.translation.y = uwb_y_pos;
    uwb_pos.transform.translation.z = 0.0;
    uwb_pos.transform.rotation = uwb_quat;
    uwb_pos.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(uwb_pos);
/*
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_footprint";
    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x;
    odom.twist.twist.linear.y = linear_velocity_y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from IMU
    odom.twist.twist.angular.z = g_imu_z;

	//TODO: include covariance matrix here
	odom.pose.covariance[0] = 20;
	odom.pose.covariance[7] = 20;
	odom.pose.covariance[14] = FLT_MAX;
	odom.pose.covariance[21] = FLT_MAX;
	odom.pose.covariance[28] =FLT_MAX;
	odom.pose.covariance[35] = 50;

	odom.twist.covariance[0] = .1; 
	odom.twist.covariance[7] = .1; 
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1; 

    odom_pub.publish(odom);
*/
    g_last_loop_time = current_time;
    r.sleep();
  }
}
