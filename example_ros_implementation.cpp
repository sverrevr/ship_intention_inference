/****************************************************************************************
 *
 *  File name : intent_inference_node.cpp
 *
 *  Function  : ROS nodes that interfaces the intention model. This node is responsible for
 *  parsing messages and calling the intention node with the correct variables. It is also
 *  resposible for setting up correct number of ships.
 *	           ---------------------
 *
 *  Version 1.0
 *
 *  Copyright (C) 2021 Sverre Velten Rothmund, NTNU Trondheim.
 *  All rights reserved.
 *
 *  Author    : Sverre Velten Rothmund
 *
 *  Modified  :
 *
 *****************************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ctime>
#include <limits>
#include <iostream>
#include <string>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "custom_msgs/IntentionNodeState.h"
#include "custom_msgs/IntentionNodeStateSingleShip.h"
#include "custom_msgs/IntentionMeasurement.h"
#include "custom_msgs/IntentionMeasurementSingleShip.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros_af_msgs/DynamicObstaclesData.h"
#include "Eigen/Dense"
#include "intention_model.h"
#include "parameters.h"
#include "utils.h"
#include "std_msgs/String.h"

#define OWN_SHIP_ID -1

using namespace INTENTION_INFERENCE;

class Intent_Inference_Node
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle nhp{"~"};
	ros::Subscriber dynamic_obstacle_subscriber_ = nh.subscribe(nhp.param<std::string>("intent_inference/do_data_in", ""), 1, &Intent_Inference_Node::dynamic_obstacle_callback, this);
	ros::Subscriber control_mode_subscriber = nh.subscribe("/milliampere/mode", 1, &Intent_Inference_Node::control_mode_callback, this);

	ros::Publisher scenario_probabilities_publisher_ = nh.advertise<ros_af_msgs::DynamicObstaclesData>(nhp.param<std::string>("intent_inference/do_data_out", ""), 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub{nh, nhp.param<std::string>("ownship_pose_topic_name", ""), 1};
	message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub{nh, nhp.param<std::string>("ownship_velocity_topic_name", ""), 1};
	message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> sync;

	ros::Publisher node_state_publisher_ = nh.advertise<custom_msgs::IntentionNodeState>(nhp.param<std::string>("logging_states", ""), 10);
	ros::Publisher measurement_publisher_ = nh.advertise<custom_msgs::IntentionMeasurement>(nhp.param<std::string>("logging_measurements", ""), 10);
	ros::Publisher saved_measurement_publisher_ = nh.advertise<custom_msgs::IntentionMeasurement>(nhp.param<std::string>("logging_saved_measurements", ""), 10);
	ros::Publisher traj_sim_node_state_publisher_ = nh.advertise<custom_msgs::IntentionNodeState>(nhp.param<std::string>("logging_traj_sim_states", ""), 10);
	ros::Publisher traj_sim_measurement_publisher_ = nh.advertise<custom_msgs::IntentionMeasurement>(nhp.param<std::string>("logging_traj_sim_measurements", ""), 10);
	ros::Publisher crash_measurement_publisher = nh.advertise<custom_msgs::IntentionMeasurement>("/intention_inference/logging/crash_measurement", 10);
	std::string control_mode = "";

	const double starting_distance = nhp.param<double>("starting_distance_m", 125);

	std::map<int, IntentionModel> ship_intention_models_;
	std::map<int, Eigen::Vector4d> ship_states;

	std::map<int, Eigen::Vector4d> previous_ship_states;
	ros::Time previous_ship_state_timestep;

	std::map<int, std::map<int, Eigen::MatrixXd>> trajectory_candidates_; //<ship_id, <trajectory_id, trajectory>>
	double trajectory_dt;
	std::vector<int> currently_tracked_ships;

	std::vector<double> trajectory_priors = nhp.param<std::vector<double>>("trajectory_prior_dist", {});

	ros_af_msgs::DynamicObstacleData obstacle_message;

	custom_msgs::IntentionMeasurement previus_measurement_msg;

	double previous_Pr_CCEM = 1;
	double previous_Pr_WGW = 1;

	const IntentionModelParameters intention_model_parameters = [&]
	{
		IntentionModelParameters param;
		param.number_of_network_evaluation_samples = nhp.param<int>("number_of_network_evaluation_samples", 0);
		param.max_number_of_obstacles = nhp.param<int>("max_number_of_obstacles", -1);
		param.time_into_trajectory = nhp.param<int>("time_into_trajectory_s", -1);
		param.expanding_dbn.min_time_s = nhp.param<double>("expanding_dbn/min_time_s", NAN);
		param.expanding_dbn.max_time_s = nhp.param<double>("expanding_dbn/max_time_s", NAN);
		param.expanding_dbn.min_course_change_rad = DEG2RAD * nhp.param<double>("expanding_dbn/min_course_change_deg", NAN);
		param.expanding_dbn.min_speed_change_m_s = nhp.param<double>("expanding_dbn/min_speed_change_m_s", NAN);
		param.ample_time_s.mu = nhp.param<double>("ample_time_s/mu", NAN);
		param.ample_time_s.sigma = nhp.param<double>("ample_time_s/sigma", NAN);
		param.ample_time_s.max = nhp.param<double>("ample_time_s/max", NAN);
		param.ample_time_s.n_bins = 30; // this value must match the bayesian network
		param.ample_time_s.minimal_accepted_by_ownship = nhp.param<double>("ample_time_s/minimal_accepted_by_ownship", NAN);
		param.safe_distance_m.mu = nhp.param<double>("safe_distance_m/mu", NAN);
		param.safe_distance_m.sigma = nhp.param<double>("safe_distance_m/sigma", NAN);
		param.safe_distance_m.max = nhp.param<double>("safe_distance_m/max", NAN);
		param.safe_distance_m.n_bins = 30; // this value must match the bayesian network
		param.safe_distance_midpoint_m.mu = nhp.param<double>("safe_distance_midpoint_m/mu", NAN);
		param.safe_distance_midpoint_m.sigma = nhp.param<double>("safe_distance_midpoint_m/sigma", NAN);
		param.safe_distance_midpoint_m.max = nhp.param<double>("safe_distance_midpoint_m/max", NAN);
		param.safe_distance_midpoint_m.n_bins = 30; // this value must match the bayesian network
		param.safe_distance_front_m.mu = nhp.param<double>("safe_distance_front_m/mu", NAN);
		param.safe_distance_front_m.sigma = nhp.param<double>("safe_distance_front_m/sigma", NAN);
		param.safe_distance_front_m.max = nhp.param<double>("safe_distance_front_m/max", NAN);
		param.safe_distance_front_m.n_bins = 30; // this value must match the bayesian network
		param.change_in_course_rad.minimal_change = DEG2RAD * nhp.param<double>("change_in_course_deg/minimal_change", NAN);
		param.change_in_speed_m_s.minimal_change = nhp.param<double>("change_in_speed_m_s/minimal_change", NAN);
		param.colregs_situation_borders_rad.HO_uncertainty_start = DEG2RAD * nhp.param("colregs_situation_borders_deg/HO_uncertainty_start", NAN);
		param.colregs_situation_borders_rad.HO_start = DEG2RAD * nhp.param("colregs_situation_borders_deg/HO_start", NAN);
		param.colregs_situation_borders_rad.HO_stop = DEG2RAD * nhp.param("colregs_situation_borders_deg/HO_stop", NAN);
		param.colregs_situation_borders_rad.HO_uncertainty_stop = DEG2RAD * nhp.param("colregs_situation_borders_deg/HO_uncertainty_stop", NAN);
		param.colregs_situation_borders_rad.OT_uncertainty_start = DEG2RAD * nhp.param("colregs_situation_borders_deg/OT_uncertainty_start", NAN);
		param.colregs_situation_borders_rad.OT_start = DEG2RAD * nhp.param("colregs_situation_borders_deg/OT_start", NAN);
		param.colregs_situation_borders_rad.OT_stop = DEG2RAD * nhp.param("colregs_situation_borders_deg/OT_stop", NAN);
		param.colregs_situation_borders_rad.OT_uncertainty_stop = DEG2RAD * nhp.param("colregs_situation_borders_deg/OT_uncertainty_stop", NAN);
		param.ignoring_safety_probability = nhp.param<double>("ignoring_safety_probability", NAN);
		param.colregs_compliance_probability = nhp.param<double>("colregs_compliance_probability", NAN);
		param.good_seamanship_probability = nhp.param<double>("good_seamanship_probability", NAN);
		param.unmodeled_behaviour = nhp.param<double>("unmodelled_behaviour_probability", NAN);
		param.priority_probability["lower"] = nhp.param<double>("priority_probability/lower", NAN);
		param.priority_probability["similar"] = nhp.param<double>("priority_probability/similar", NAN);
		param.priority_probability["higher"] = nhp.param<double>("priority_probability/higher", NAN);

		return param;
	}();

	const std::string network_filename = [&]
	{
		if (intention_model_parameters.max_number_of_obstacles == 1)
		{
			return nhp.param<std::string>("network_filename_one_obstacle", "");
		}
		else if (intention_model_parameters.max_number_of_obstacles == 2)
		{
			return nhp.param<std::string>("network_filename_two_obstacles", "");
		}
		else
		{
			ROS_ERROR("Model not implemented for specified max number of obstacles");
			assert(false);
			return std::string("");
		}
	}();

	void ensure_initialization()
	{
		for (const auto ship_id : currently_tracked_ships)
		{
			// Initiate ships that are not the own-ship, that have not already been initiated, and that are sufficiently close
			if (ship_id != OWN_SHIP_ID && !ship_intention_models_.count(ship_id) && evaluateDistance(better_at(ship_states, ship_id)[PX] - better_at(ship_states, OWN_SHIP_ID)[PX], better_at(ship_states, ship_id)[PY] - better_at(ship_states, OWN_SHIP_ID)[PY]) < starting_distance)
			{
				ship_intention_models_.insert({ship_id, IntentionModel(network_filename, intention_model_parameters, ship_id, ship_states)});
				ROS_INFO("New model for ship %d", ship_id);
			}
		}
	}

public:
	Intent_Inference_Node() : sync(pose_sub, twist_sub, 1)
	{
		sync.registerCallback(boost::bind(&Intent_Inference_Node::ownship_state_CB, this, _1, _2));
	}

	void control_mode_callback(const std_msgs::String::ConstPtr &msg)
	{
		control_mode = msg->data;
	}

	void ownship_state_CB(geometry_msgs::PoseStamped::ConstPtr pose_msg, geometry_msgs::TwistStamped::ConstPtr twist_msg)
	{
		Eigen::Quaternion q(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
		auto yaw = q.toRotationMatrix().eulerAngles(0, 1, 2)(2);

		ship_states[OWN_SHIP_ID](PX) = pose_msg->pose.position.x;
		ship_states[OWN_SHIP_ID](PY) = pose_msg->pose.position.y;
		ship_states[OWN_SHIP_ID](CHI) = surgeSwayToCourse(yaw, twist_msg->twist.linear.x, twist_msg->twist.linear.y);
		ship_states[OWN_SHIP_ID](U) = 1.1; //surgeSwayToSpeed(twist_msg->twist.linear.x, twist_msg->twist.linear.y);
	}

	void dynamic_obstacle_callback(
		ros_af_msgs::DynamicObstaclesData msg // In: ROS message containing obstacle state estimates, covariance estimates and their IDS and size info
	)
	{
		if (ship_states.find(OWN_SHIP_ID) == ship_states.end() || (control_mode != "psbmpc" && control_mode != "sbmpc"))
		{
			// ROS_INFO_THROTTLE(1, "Missing own ship data, not evaluating");
			return;
		}

		currently_tracked_ships.clear();
		currently_tracked_ships.push_back(OWN_SHIP_ID); // ownship is allways tracked
		// Write down information
		for (const auto &ship_msg : msg.do_data)
		{
			currently_tracked_ships.push_back(ship_msg.ID);
			for (const auto &trajectory_msg : ship_msg.predicted_trajectories)
			{
				trajectory_candidates_[ship_msg.ID][trajectory_msg.ID] = Eigen::MatrixXd(4, trajectory_msg.trajectory.size());
				for (size_t i = 0; i < trajectory_msg.trajectory.size(); ++i)
				{
					const auto &state = trajectory_msg.trajectory[i];
					trajectory_candidates_[ship_msg.ID][trajectory_msg.ID](PX, i) = state.pos_est.x;
					trajectory_candidates_[ship_msg.ID][trajectory_msg.ID](PY, i) = state.pos_est.y;
					trajectory_candidates_[ship_msg.ID][trajectory_msg.ID](CHI, i) = std::atan2(state.vel_est.y, state.vel_est.x);
					trajectory_candidates_[ship_msg.ID][trajectory_msg.ID](U, i) = std::sqrt(std::pow(state.vel_est.x, 2) + std::pow(state.vel_est.y, 2));
				}
			}
			ship_states[ship_msg.ID](PX) = trajectory_candidates_[ship_msg.ID][ship_msg.predicted_trajectories.front().ID](PX, 0);
			ship_states[ship_msg.ID](PY) = trajectory_candidates_[ship_msg.ID][ship_msg.predicted_trajectories.front().ID](PY, 0);
			ship_states[ship_msg.ID](CHI) = trajectory_candidates_[ship_msg.ID][ship_msg.predicted_trajectories.front().ID](CHI, 0);
			ship_states[ship_msg.ID](U) = trajectory_candidates_[ship_msg.ID][ship_msg.predicted_trajectories.front().ID](U, 0);
		}
		// ROS_INFO("Message written down, there are now %d ships and ship 0 has %d trajs", trajectory_candidates_.size(),better_at(trajectory_candidates_, msg.do_data[0].ID).size());
		ensure_initialization();

		custom_msgs::IntentionNodeState node_state_msg;
		custom_msgs::IntentionMeasurement measurement_msg;
		custom_msgs::IntentionMeasurement measurement_traj_msg;
		for (auto &ship_msg : msg.do_data)
		{
			try
			{
				node_state_msg.intention_ample_time.clear();
				node_state_msg.intention_safe_distance.clear();
				node_state_msg.intention_safe_distance_front.clear();
				node_state_msg.intention_safe_distance_midpoint.clear();
				node_state_msg.ship_intermediate_nodes.clear();
				node_state_msg.traj_probabilities.clear();
				measurement_msg.ship_measurements.clear();

				bool should_update = true;
				if (should_update)
				{
					try
					{
						bool did_save = better_at(ship_intention_models_, ship_msg.ID).insertObservation(ship_states, currently_tracked_ships, ship_msg.is_currently_changing_course_or_speed, &node_state_msg, &measurement_msg);
						node_state_msg.trajectory_candidate_id = -1;
						measurement_msg.trajectory_candidate_id = -1;
						if (did_save)
						{
							saved_measurement_publisher_.publish(previus_measurement_msg);
						}
						previus_measurement_msg = measurement_msg;

						ship_msg.Pr_CCEM = node_state_msg.intention_colregs_compliant;
						auto element = std::find_if(node_state_msg.ship_intermediate_nodes.begin(), node_state_msg.ship_intermediate_nodes.end(),
													[&](const auto &x)
													{ return x.towards_ship_id == OWN_SHIP_ID; });
						ship_msg.Pr_WGW = (1 - element->priority_intention_higher) * node_state_msg.intention_colregs_compliant * (1 - node_state_msg.intention_unmodeled_behaviour);
						//std::cout << "IN i = " << ship_msg.ID << ": Pr_WGW = " << ship_msg.Pr_WGW << " | Pr_CCEM = " << ship_msg.Pr_CCEM << std::endl;
						previous_Pr_CCEM = ship_msg.Pr_CCEM;
						previous_Pr_WGW = ship_msg.Pr_WGW;
					}
					catch (...)
					{
						crash_measurement_publisher.publish(measurement_msg);
						throw("ERROR");
					}
					//ROS_WARN("UPDATING");
				}
				else
				{
					ship_msg.Pr_CCEM = previous_Pr_CCEM;
					ship_msg.Pr_WGW = previous_Pr_WGW;
					//ROS_WARN("NOT_UPDATING");
				}

				double trajectory_prob_sum = 0;
				int num_trajs = ship_msg.predicted_trajectories.size();
				int num_priors = trajectory_priors.size();
				for (auto &trajectory_msg : ship_msg.predicted_trajectories)
				{
					try
					{
						auto res = better_at(ship_intention_models_, ship_msg.ID).evaluateTrajectory(better_at(better_at(trajectory_candidates_, ship_msg.ID), trajectory_msg.ID), ship_states, currently_tracked_ships, trajectory_msg.dt_p, &measurement_traj_msg);
						measurement_traj_msg.trajectory_candidate_id = trajectory_msg.ID;
						traj_sim_measurement_publisher_.publish(measurement_traj_msg);

						//Multiply trajectory probs with prior
						//Must handle that trajectories has a different size than priors
						//Make sure the middle prior hits the middle trajectory, and then apply outwards
						int trajs_midpoint = (int)(num_trajs / 2);
						int priors_midpoint = (int)(num_priors / 2);
						int prior_index = std::max(std::min(((int)trajectory_msg.ID - trajs_midpoint) + priors_midpoint, num_priors - 1), 0);

						trajectory_msg.probability = res * trajectory_priors[prior_index];
						trajectory_prob_sum += trajectory_msg.probability;
					}
					catch (...)
					{
						ROS_ERROR("CRASHED ON EVALUATING TRAJECTORY");
						crash_measurement_publisher.publish(measurement_traj_msg);
						throw("ERROR");
					}
				}

				if (trajectory_prob_sum > 0.0001)
				{
					for (auto &trajectory_msg : ship_msg.predicted_trajectories)
					{
						trajectory_msg.probability /= trajectory_prob_sum;
						node_state_msg.traj_probabilities.push_back(trajectory_msg.probability);
					}
				}
				else
				{
					for (auto &trajectory_msg : ship_msg.predicted_trajectories)
					{
						trajectory_msg.probability = 1 / num_trajs;
						node_state_msg.traj_probabilities.push_back(trajectory_msg.probability);
					}
				}

				if (should_update)
				{
					node_state_publisher_.publish(node_state_msg);
					measurement_publisher_.publish(measurement_msg);
				}
			}
			catch (...)
			{
				ship_msg.Pr_CCEM = previous_Pr_CCEM;
				ship_msg.Pr_WGW = previous_Pr_WGW;
				//If something fails, send prior distribution
				int num_trajs = ship_msg.predicted_trajectories.size();
				int num_priors = trajectory_priors.size();
				double prior_sum = 0;
				for (auto &element : trajectory_priors)
				{
					prior_sum += element;
				}
				for (auto &trajectory_msg : ship_msg.predicted_trajectories)
				{
					int trajs_midpoint = (int)(num_trajs / 2);
					int priors_midpoint = (int)(num_priors / 2);
					int prior_index = std::max(std::min(((int)trajectory_msg.ID - trajs_midpoint) + priors_midpoint, num_priors - 1), 0);
					trajectory_msg.probability = trajectory_priors[prior_index] / prior_sum;
				}
			}
		}
		scenario_probabilities_publisher_.publish(msg);
		previous_ship_states = ship_states;
		previous_ship_state_timestep = msg.header.stamp;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "intent_inference_node");
	Intent_Inference_Node intent_inference_node;
	ros::spin();

	return 0;
}
