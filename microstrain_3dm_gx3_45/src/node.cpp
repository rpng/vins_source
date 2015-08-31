#include "ros/ros.h"
#include "microstrain_3dm_gx3_45/driver.h"
#include "microstrain_3dm_gx3_45/node.h"

using namespace microstrain_3dm_gx3_45;
using namespace std;
using namespace boost;
using namespace ros;

/*
 * TODOs
 * add some services etc.
 */


imuNode::imuNode() : nh_priv_("~") {

	param::param<string>("~port",port_,"/dev/ttyACM0");
	param::param<int>("~baud_rate",baud_rate_,115200);
	param::param<float>("~declination",declination_, 3.8); // http://www.ngdc.noaa.gov/geomag-web/#declination
	param::param<string>("~frame_id",frame_id_,"/imu_link");
	param::param<string>("~child_frame_id",child_frame_id_,"/imu_link");
	param::param<float>("~rate",rate_,10.0);

	param::param<bool>("~publish_pose",publish_pose_,true); // TODO make this in form publish/imu/pose
	param::param<bool>("~publish_imu",publish_imu_,true);
	param::param<bool>("~publish_gps",publish_gps_,true);
	param::param<bool>("~publish_gps_as_odom",publish_gps_as_odom_,true);
	param::param<bool>("~publish_nav_odom",publish_nav_odom_,true);
	//param::param<bool>("~nav_odom_rel",nav_odom_rel_,true);
	param::param<bool>("~publish_nav_pose",publish_nav_pose_,true);
	param::param<bool>("~publish_nav_fix",publish_nav_fix_,true);

	param::param<bool>("~zero_height",zero_height_,true);

	param::param("~linear_acceleration_stdev", linear_acceleration_stdev_, 0.098);
	param::param("~orientation_stdev", orientation_stdev_, 0.035);
	param::param("~angular_velocity_stdev", angular_velocity_stdev_, 0.012);
	
	imu_.reset(new IMU((int)floor(rate_)));

	started_ = false;
	inited_ = false;

	gps_fix_available_ = false;

	if (publish_imu_) imu_data_pub_ = nh_priv_.advertise<sensor_msgs::Imu>("imu/data", 10);
	if (publish_pose_) imu_pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("imu/pose", 10);

	service_reset_ = service_reset_ = nh_priv_.advertiseService("reset_kf", &imuNode::srvResetKF,this);

	if (publish_gps_) gps_pub_ = nh_priv_.advertise<sensor_msgs::NavSatFix>("gps/fix", 100);
	if (publish_gps_as_odom_) gps_odom_pub_ = nh_priv_.advertise<nav_msgs::Odometry>("gps/odom", 10);

	if (publish_nav_odom_) nav_odom_pub_ = nh_priv_.advertise<nav_msgs::Odometry>("nav/odom", 10);
	if (publish_nav_pose_) nav_pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("nav/pose", 10);
	if (publish_nav_fix_) nav_fix_pub_ = nh_priv_.advertise<sensor_msgs::NavSatFix>("nav/fix", 10);

}

bool imuNode::srvResetKF(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {


	ROS_INFO("Resetting KF.");

	if (!imu_->setToIdle()) ROS_ERROR("%s",imu_->getLastError().c_str());
	if (!imu_->initKalmanFilter(declination_)) ROS_ERROR("%s",imu_->getLastError().c_str());
	if (!imu_->resume()) ROS_ERROR("%s",imu_->getLastError().c_str());

	return true;

}

bool imuNode::init() {


	if (!imu_->openPort(port_,(unsigned int)baud_rate_)) {

		ROS_ERROR("Can't open port.");
		return false;

	}

	started_ = false;

	ROS_INFO("Pinging device");
	imu_->setTimeout(posix_time::seconds(0.5));
	if (!imu_->ping()) {

		printErrMsgs("Pinging device");
		return false;

	}

	ROS_INFO("Setting to idle");
	if (!imu_->setToIdle()) {

		printErrMsgs("Setting to idle");
		return false;

	}

	ROS_INFO("Checking status");
	if (!imu_->devStatus()) {

		printErrMsgs("Checking status");
		return false;

	}

	ROS_INFO("Disabling all streams");
	if (!imu_->disAllStreams()) {

		printErrMsgs("Disabling all streams");
		return false;

	}

	ROS_INFO("Device self test");
	if (!imu_->selfTest()) {

		printErrMsgs("Device self test");
		return false;
	}

	ROS_INFO("Setting AHRS msg format");
	if (!imu_->setAHRSMsgFormat()) {

		printErrMsgs("Setting AHRS msg format");
		return false;

	}

	ROS_INFO("Setting GPS msg format");
	if (!imu_->setGPSMsgFormat()) {

			printErrMsgs("Setting GPS msg format");
			return false;

		}

	ROS_INFO("Setting NAV msg format");
	if (!imu_->setNAVMsgFormat()) {

		printErrMsgs("Setting NAV msg format");
		return false;

	}

	start();

	ROS_INFO("KF initialization");
	if (!imu_->initKalmanFilter(declination_)) {

		printErrMsgs("KF initialization");
		return false;

	}

	inited_ = true;
	return true;

}

bool imuNode::start() {

	if (!imu_->resume()) {

			printErrMsgs("Resuming");
			return false;

		}

	started_ = true;
	return true;

}

bool imuNode::stop() {

	if (!imu_->setToIdle()) {

		printErrMsgs("To idle");
		return false;

	}

	started_ = false;
	return true;

}

void imuNode::spin() {

	if (!imu_->isOpen()) {

		ROS_ERROR("Port is not opened. Can't continue.");
		return;

	}

	Rate r(rate_);

	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = frame_id_;
	ps.pose.position.x = ps.pose.position.y = ps.pose.position.z = 0.0;

	geometry_msgs::PoseStamped nav_pose;
	nav_pose.header.frame_id = frame_id_;
	nav_pose.pose.position.x = nav_pose.pose.position.y = nav_pose.pose.position.z = 0.0; // TODO ????

	sensor_msgs::Imu imu;
	imu.header.frame_id = frame_id_;

	double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
	double orientation_covariance = orientation_stdev_ * orientation_stdev_;
	double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;

	imu.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	imu.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	imu.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	imu.angular_velocity_covariance[0] = angular_velocity_covariance;
	imu.angular_velocity_covariance[4] = angular_velocity_covariance;
	imu.angular_velocity_covariance[8] = angular_velocity_covariance;

	imu.orientation_covariance[0] = orientation_covariance;
	imu.orientation_covariance[4] = orientation_covariance;
	imu.orientation_covariance[8] = orientation_covariance;

	sensor_msgs::NavSatFix gps;

	gps.header.frame_id = frame_id_;
	gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

	sensor_msgs::NavSatFix nav_fix;
	nav_fix.header.frame_id = frame_id_;
	nav_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;


	nav_msgs::Odometry gps_odom;

	gps_odom.header.frame_id = "/odom"; // TODO make conf.
	gps_odom.child_frame_id = child_frame_id_;
	gps_odom.pose.pose.orientation.x = 1; // identity quaternion
	gps_odom.pose.pose.orientation.y = 0;
	gps_odom.pose.pose.orientation.z = 0;
	gps_odom.pose.pose.orientation.w = 0;
	gps_odom.pose.covariance[21] = 99999; // rot x
	gps_odom.pose.covariance[28] = 99999; // rot y
	gps_odom.pose.covariance[35] = 99999; // rot z

	nav_msgs::Odometry nav_odom;

	nav_odom.header.frame_id = "/odom";
	nav_odom.child_frame_id = child_frame_id_;
	nav_odom.pose.pose.position.x = nav_odom.pose.pose.position.y = nav_odom.pose.pose.position.z = 0.0;


	ROS_INFO("Start polling device.");

	int gps_msg_cnt = 0;

	while(ok()) {

		if (publish_nav_odom_ || publish_nav_pose_ || publish_nav_fix_) {

			if (!imu_->pollNAV()) {

				printErrMsgs("NAV");

			}

			// TODO check nav filter status


		}

		if (publish_imu_ || publish_pose_) {

			if (!imu_->pollAHRS()) {

				printErrMsgs("AHRS");

			}

		}


		if (publish_nav_pose_ && nav_pose_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing NAV as Pose.");

			tnav n = imu_->getNAV();

			nav_pose.header.stamp.fromNSec(n.time);

			float yaw = n.est_y;

			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-n.est_r, n.est_p, -yaw), nav_pose.pose.orientation);

			nav_pose_pub_.publish(nav_pose);

		}

		if (publish_nav_odom_ && nav_odom_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing NAV as Odometry.");

			tnav n = imu_->getNAV();

			nav_odom.header.stamp.fromNSec(n.time);
			
			if (n.est_latitude != 0 && n.est_longtitude != 0) {

			   geographic_msgs::GeoPoint pt;

			   pt.latitude = n.est_latitude;
			   pt.longitude = n.est_longtitude;
			   if (!zero_height_) pt.altitude = n.est_height;
			   else pt.altitude = 0.0;

			   geodesy::UTMPoint utm;

			   geodesy::fromMsg(pt,utm);

			   ROS_INFO_ONCE("UTM zone: %d.",(int)utm.zone);

			  //gps_common::LLtoUTM(n.est_latitude, n.est_longtitude, northing, easting, zone);

			  nav_odom.pose.pose.position.x = utm.easting;
			  nav_odom.pose.pose.position.y = utm.northing;
			  nav_odom.pose.pose.position.z = utm.altitude;
			
			} /*else {
			
			  nav_odom.pose.pose.position.x = 0;
			  nav_odom.pose.pose.position.y = 0;
			  nav_odom.pose.pose.position.z = 0;
			
			}*/

			float yaw = n.est_y;

			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-n.est_r, n.est_p, -yaw), nav_odom.pose.pose.orientation);

			if (n.est_pos_unc_valid) {

				nav_odom.pose.covariance[0] = pow(n.est_east_pos_unc,2); // TODO check if this is ok
				nav_odom.pose.covariance[7] = pow(n.est_north_pos_unc,2);
				nav_odom.pose.covariance[14] = pow(n.est_down_pos_unc,2);

			} else {

				nav_odom.pose.covariance[0] = 99999;
				nav_odom.pose.covariance[7] = 99999;
				nav_odom.pose.covariance[14] = 99999;

			}


            if (n.est_vel_unc_valid) {

				nav_odom.pose.covariance[21] = pow(n.est_east_vel_unc,2);
				nav_odom.pose.covariance[28] = pow(n.est_north_vel_unc,2);
				nav_odom.pose.covariance[35] = pow(n.est_down_vel_unc,2);


            } else {

            	nav_odom.pose.covariance[21] = 99999;
            	nav_odom.pose.covariance[28] = 99999;
            	nav_odom.pose.covariance[35] = 99999;

            }

            nav_odom.twist.twist.linear.x = -n.est_acc_lin_x;
            nav_odom.twist.twist.linear.y = n.est_acc_lin_y;
            nav_odom.twist.twist.linear.z = -n.est_acc_lin_z;

            nav_odom.twist.twist.angular.x = -n.est_acc_rot_x;
            nav_odom.twist.twist.angular.y = n.est_acc_rot_y;
            nav_odom.twist.twist.angular.z = -n.est_acc_rot_z;

            if (n.est_acc_lin_valid) {

            	nav_odom.twist.covariance[0] = linear_acceleration_covariance;
            	nav_odom.twist.covariance[7] = linear_acceleration_covariance;
            	nav_odom.twist.covariance[14] = linear_acceleration_covariance;
            } else {

            	nav_odom.twist.covariance[0] = 99999;
				nav_odom.twist.covariance[7] = 99999;
				nav_odom.twist.covariance[14] = 99999;

            }

            if (n.est_acc_rot_valid) {

				nav_odom.twist.covariance[21] = angular_velocity_covariance;
				nav_odom.twist.covariance[28] = angular_velocity_covariance;
				nav_odom.twist.covariance[35] = angular_velocity_covariance;

			} else {

				nav_odom.twist.covariance[21] = 99999;
				nav_odom.twist.covariance[28] = 99999;
				nav_odom.twist.covariance[35] = 99999;

			}


            nav_odom_pub_.publish(nav_odom);

		}


		if (publish_imu_ && imu_data_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing IMU data.");

			tahrs q = imu_->getAHRS();

			imu.header.stamp.fromNSec(q.time);

			imu.linear_acceleration.x = -q.ax;
			imu.linear_acceleration.y = q.ay;
			imu.linear_acceleration.z = -q.az;

			imu.angular_velocity.x = -q.gx;
			imu.angular_velocity.y = q.gy;
			imu.angular_velocity.z = -q.gz;

			float yaw = q.y;

            // TODO is this needed?
			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-q.r, q.p, -yaw), imu.orientation);

			imu_data_pub_.publish(imu);

		}

		if (publish_pose_ && imu_pose_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing IMU data as PoseStamped.");

			tahrs q = imu_->getAHRS();

			//ps.header.stamp.fromBoost(q.time);
			ps.header.stamp.fromNSec(q.time);

			float yaw = q.y;
			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-q.r, q.p, -yaw),ps.pose.orientation);

			imu_pose_pub_.publish(ps);

		}

		if (publish_gps_ || publish_gps_as_odom_ || publish_nav_fix_ || publish_nav_odom_) {

			if (!imu_->pollGPS()) {

				printErrMsgs("GPS");

			}

			tgps g;
			g = imu_->getGPS();

			if (!g.lat_lon_valid) ROS_WARN_ONCE("GPS fix not available.");

			if (g.lat_lon_valid && !gps_fix_available_) {

				ROS_INFO("GPS fix available.");
				gps_fix_available_ = true;

			}

			if (!g.lat_lon_valid && gps_fix_available_) {

				ROS_WARN("GPS fix lost.");
				gps_fix_available_ = false;

			}


			if (gps_fix_available_) {

				if (gps_msg_cnt++==6*rate_) {

					gps_msg_cnt = 0;

					if (!g.lat_lon_valid) ROS_WARN("LAT/LON not valid.");
					if (!g.hor_acc_valid) ROS_WARN("Horizontal accuracy not valid.");
					else ROS_INFO("GPS horizontal accuracy: %f",g.horizontal_accuracy);

				}

			}

		}

		if (publish_nav_fix_ && nav_fix_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing NAV as NavSatFix.");

			tnav n = imu_->getNAV();

			nav_fix.header.stamp.fromNSec(n.time);

			nav_fix.latitude = n.est_latitude;
			nav_fix.longitude = n.est_longtitude;
			if (!zero_height_) nav_fix.altitude = n.est_height;
			else nav_fix.altitude = 0.0;

			if (n.est_llh_valid) nav_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
			else nav_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

			if (n.est_pos_unc_valid) {

				nav_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
				nav_fix.position_covariance[0] = pow(n.est_north_pos_unc,2);
				nav_fix.position_covariance[4] = pow(n.est_east_pos_unc,2);
				nav_fix.position_covariance[8] = pow(n.est_down_pos_unc,2);

			} else {

				nav_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

			}

			nav_fix_pub_.publish(nav_fix);

		}

		if (publish_gps_ && gps_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publishing GPS as NavSatFix.");

			tgps g;
			g = imu_->getGPS();

			gps.header.stamp.fromNSec(g.time);

			gps.latitude = g.latitude;
			gps.longitude = g.longtitude;
			gps.altitude = 0.0;

			if (g.lat_lon_valid) gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
			else gps.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

			if (!g.hor_acc_valid) gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
			else {

				//gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
				gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

				gps.position_covariance[0] = g.horizontal_accuracy * g.horizontal_accuracy;
				gps.position_covariance[4] = g.horizontal_accuracy * g.horizontal_accuracy;
				gps.position_covariance[8] = 100.0; // TODO add this to driver

			}

			gps_pub_.publish(gps);

		}

		if (publish_gps_as_odom_ && gps_odom_pub_.getNumSubscribers() > 0) {

			ROS_INFO_ONCE("Publihing GPS as Odometry.");

			tgps g;
			g = imu_->getGPS();

			gps_odom.header.stamp.fromNSec(g.time);

			if (g.lat_lon_valid) {

				gps_odom.pose.covariance[0] = g.horizontal_accuracy * g.horizontal_accuracy;
				gps_odom.pose.covariance[7] = g.horizontal_accuracy * g.horizontal_accuracy;
				gps_odom.pose.covariance[14] = 99999; // TODO check vertical acc.

			} else {

				gps_odom.pose.covariance[0] = 99999;
				gps_odom.pose.covariance[7] = 99999;
				gps_odom.pose.covariance[14] = 99999;

			}

			geographic_msgs::GeoPoint pt;

		    pt.latitude = g.latitude;
		    pt.longitude = g.longtitude;
		    pt.altitude = 0;

		    geodesy::UTMPoint utm;

		    geodesy::fromMsg(pt,utm);

			//gps_common::LLtoUTM(g.latitude, g.longtitude, northing, easting, zone);

			gps_odom.pose.pose.position.x = utm.easting;
			gps_odom.pose.pose.position.y = utm.northing;
			gps_odom.pose.pose.position.z = 0.0; // TODO fill this

			gps_odom_pub_.publish(gps_odom);

		}

		r.sleep();
		spinOnce();

	};

	stop();

}

void imuNode::printErrMsgs(string prefix) {

	string msg = "...";

	while(msg!="") {

	  msg = imu_->getLastError();
	  if (msg!="") ROS_ERROR("%s: %s",prefix.c_str(),msg.c_str());

  }

}

imuNode::~imuNode() {

	imu_->closePort();

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu3dmgx3");

  imuNode node;

  ROS_INFO("Initializing.");
  if (!node.init()) {

	  ROS_ERROR("Initialization failed. Please check logs.");
	  return 0;

  }

  ROS_INFO("Initialization completed.");

  node.spin();


  ROS_INFO("Finished");


  return 0;

}
