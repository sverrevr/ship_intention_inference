// This file includes the intentio model class. This saves all the information needed and implements the needed functions for intention inference.
#pragma once

#include <map>
#include "Eigen/Dense"
#include <string>
#include <sstream>
//#include <boost/math/special_functions/sign.hpp>
#include <math.h>
#include <optional>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "bayesian_network.h"
#include "identifiers.h"
#include "geometry.h"
#include "parameters.h"
#include "utils.h"

//TODO fiks:
// ros::Time::now() <- tidspunktet for denne målingen
// custom_msgs::... <- er bare brukt for å vise frem resultatet, kan fjernes (erstattes?)
// putt inn tid som input i observation

namespace INTENTION_INFERENCE
{
	class IntentionModel
	{
	private:
		const IntentionModelParameters &parameters;
		BayesianNetwork net;
		const int my_id;
		std::map<int, std::string> ship_name_map;
		std::vector<std::string> ship_names;
		const std::vector<std::string> intention_node_names_ship_specific = {"colav_situation_towards_", "priority_intention_to_", "disable_"};
		const std::vector<std::string> intention_node_names_general = {"intention_colregs_compliant", "intention_good_seamanship", "intention_safe_distance", "intention_safe_distance_front", "intention_safe_distance_midpoint", "intention_ample_time", "intention_ignoring_safety", "unmodelled_behaviour"};
		const std::vector<std::string> intermediate_node_names_ship_specific = {"is_pre_ample_time_to_", "safe_distance_at_CPA_towards_", "safe_crossing_front_towards_", "safe_distance_to_", "safe_distance_to_midpoint_", "gives_way_correct_towards_", "Observation_applicable_towards_", "role_towards_", "good_seamanship_to_"};
		std::vector<std::string> intermediate_node_names_general = {"has_turned_portwards", "has_turned_starboardwards", "observation_applicable", "stands_on_correct"};
		std::vector<std::string> all_node_names;
		const std::string output_name = "observation_applicable";

		const std::map<int, Eigen::Vector4d> initial_ship_states;
		std::map<int, Eigen::Vector4d> previously_saved_ship_states;
		std::map<int, double> time_last_saved_shipstate;

		bool doSave(const std::map<int, Eigen::Vector4d> &ship_states, double time)
		{
			//const auto min_time_between_saved_states = ros::Duration{parameters.expanding_dbn.min_time_s};
			//const auto max_time_between_saved_states = ros::Duration{parameters.expanding_dbn.max_time_s};
			const auto min_time_between_saved_states = 20;
			const auto max_time_between_saved_states = 1200;
			const auto heading_change_to_save = parameters.expanding_dbn.min_course_change_rad;
			const auto speed_change_to_save = parameters.expanding_dbn.min_speed_change_m_s;
			if (!previously_saved_ship_states.size())
			{
				previously_saved_ship_states = ship_states;
				for (const auto &[ship_id, state] : previously_saved_ship_states)
				{
					(void)state; // discard state to avoid compiler warning for unused variable
					time_last_saved_shipstate[ship_id] = time;
				}
				return false;
			}
			else
			{
				for (auto const &[ship_id, current_ship_state] : ship_states)
				{
					const auto &past_ship_state = better_at(previously_saved_ship_states, ship_id);
					const auto time_passed = time - better_at(time_last_saved_shipstate, ship_id);
					if (time_passed > min_time_between_saved_states && (time_passed > max_time_between_saved_states || std::abs(current_ship_state[CHI] - past_ship_state[CHI]) > heading_change_to_save || std::abs(current_ship_state[U] - past_ship_state[U]) > speed_change_to_save))
					{
						previously_saved_ship_states = ship_states;
						time_last_saved_shipstate[ship_id] = time;
						return true;
					}
				}
			}
			return false;
		}

		void evaluate_nodes(std::ofstream &intentionFile){
			int reference_ship_id = my_id;

			auto result = net.evaluateStates(all_node_names);

			auto intention_colregs_compliant = better_at(better_at(result, "intention_colregs_compliant"), "true");
			intentionFile << intention_colregs_compliant << ",";
			std::cout << "intention_colregs_compliant: " << intention_colregs_compliant << std::endl;
			auto intention_ignoring_safety = better_at(better_at(result, "intention_ignoring_safety"), "true");
			auto intention_good_seamanship = better_at(better_at(result, "intention_good_seamanship"), "true");
			intentionFile << intention_good_seamanship << ",";
			std::cout << "intention_good_seamanship: " << intention_good_seamanship << std::endl;
			auto intention_unmodeled_behaviour = better_at(better_at(result, "unmodelled_behaviour"), "true");
			intentionFile << intention_unmodeled_behaviour << ",";
			std::cout << "intention_unmodeled_behaviour: " << intention_unmodeled_behaviour << std::endl;
			auto has_turned_portwards = better_at(better_at(result, "has_turned_portwards"), "true");
			auto has_turned_starboardwards = better_at(better_at(result, "has_turned_starboardwards"), "true");
			std::cout << "has_turned_starboardwards: " << has_turned_starboardwards << std::endl;
			//node_state_msg->stands_on_correct = better_at(better_at(result, "stands_on_correct"), "true");
			auto stands_on_correct = better_at(better_at(result, "stands_on_correct"), "true");
			auto observation_applicable = better_at(better_at(result, "observation_applicable"), "true");
			
			for (auto const &[ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					auto intention_colav_situation_HO = better_at(better_at(result, "colav_situation_towards_" + ship_name), "HO");
					intentionFile << intention_colav_situation_HO << ",";
					std::cout << "intention_colav_situation_HO: " << intention_colav_situation_HO << std::endl;
					auto intention_colav_situation_CR_SS = better_at(better_at(result, "colav_situation_towards_" + ship_name), "CR_SS");
					intentionFile << intention_colav_situation_CR_SS << ",";
					std::cout << "intention_colav_situation_CR_SS: " << intention_colav_situation_CR_SS << std::endl;
					auto intention_colav_situation_CR_PS = better_at(better_at(result, "colav_situation_towards_" + ship_name), "CR_PS");
					intentionFile << intention_colav_situation_CR_PS << ",";
					std::cout << "intention_colav_situation_CR_PS: " << intention_colav_situation_CR_PS << std::endl;
					auto intention_colav_situation_OT_ing = better_at(better_at(result, "colav_situation_towards_" + ship_name), "OT_ing");
					intentionFile << intention_colav_situation_OT_ing << ",";
					std::cout << "intention_colav_situation_OT_ing: " << intention_colav_situation_OT_ing << std::endl;
					auto intention_colav_situation_OT_en = better_at(better_at(result, "colav_situation_towards_" + ship_name), "OT_en");
					intentionFile << intention_colav_situation_OT_en << ",";
					std::cout << "intention_colav_situation_OT_en: " << intention_colav_situation_OT_en << std::endl;

					auto priority_intention_lower = better_at(better_at(result, "priority_intention_to_" + ship_name), "lower");
					intentionFile << priority_intention_lower << ",";
					auto priority_intention_similar = better_at(better_at(result, "priority_intention_to_" + ship_name), "similar");
					intentionFile << priority_intention_similar << ",";
					std::cout << "priority_intention_similar: " << priority_intention_similar << std::endl;
					auto autopriority_intention_higher = better_at(better_at(result, "priority_intention_to_" + ship_name), "higher");
					intentionFile << autopriority_intention_higher;
				}
			}
			intentionFile << "\n";

		}


	public:
		IntentionModel(std::string network_file_name, const IntentionModelParameters &parameters, int my_id, const std::map<int, Eigen::Vector4d> &ship_states) : IntentionModel(network_file_name, parameters, my_id, ship_states, std::map<std::string, std::string>{}) {}

		IntentionModel(std::string network_file_name, const IntentionModelParameters &parameters, int my_id, const std::map<int, Eigen::Vector4d> &ship_states, const std::map<std::string, std::string> &priors) : parameters(parameters),
																																																					net(network_file_name, parameters.number_of_network_evaluation_samples),
																																																					my_id(my_id),
																																																					initial_ship_states(ship_states)
		{
			ship_names.clear();
			for (unsigned i = 0; i < parameters.max_number_of_obstacles; ++i)
			{
				std::string name = "ship" + std::to_string(i);
				ship_names.push_back(name);
			}

			// Convert ship-ids to the names used in the network
			size_t i = 0;
			for (auto const &[ship_id, ship_state] : ship_states)
			{
				(void)ship_state; // discard unused variable
				if (ship_id != my_id)
				{
					ship_name_map[ship_id] = ship_names[i];
					printf("Added ship name \"%s\" for ship id %d", ship_names[i].c_str(), ship_id);
					++i;
				}
			}

			// Generate node names
			all_node_names.insert(all_node_names.end(), intention_node_names_general.begin(), intention_node_names_general.end());
			all_node_names.insert(all_node_names.end(), intermediate_node_names_general.begin(), intermediate_node_names_general.end());
			for (auto [ship_id, ship_name] : ship_name_map)
			{
				(void)ship_id;
				for (auto node : intention_node_names_ship_specific)
				{
					all_node_names.push_back(node + ship_name);
				}
				for (auto node : intermediate_node_names_ship_specific)
				{
					all_node_names.push_back(node + ship_name);
				}
			}

	
			net.setEvidence(priors);

			// Initiate colregs situation
			for (auto const &[ship_id, ship_state] : ship_states)
			{
				if (ship_id != my_id)
				{
					std::string ship_name = better_at(ship_name_map, ship_id);
					const auto situation = evaluateSitution(parameters, better_at(ship_states, my_id), ship_state);
					net.setPriors("colav_situation_towards_" + ship_name, situation);

					std::stringstream situation_ss;
					situation_ss.precision(2);
					situation_ss << "Ship " << my_id << " init colav situation towards ship " << ship_id << " as: ";
					for (const auto &[name, value] : situation)
					{
						situation_ss << name << "=" << value << ", ";
					}
					auto s = situation_ss.str();
					printf("%s", s.c_str());
				}
			}

			net.setBinaryPriors("intention_ignoring_safety", parameters.ignoring_safety_probability);
			net.setBinaryPriors("intention_colregs_compliant", parameters.colregs_compliance_probability);
			net.setBinaryPriors("intention_good_seamanship", parameters.good_seamanship_probability);
			net.setBinaryPriors("unmodelled_behaviour", parameters.unmodeled_behaviour);

			for (auto [ship_id, ship_name] : ship_name_map)
			{
				if (ship_id != my_id)
				{
					net.setPriors("priority_intention_to_" + ship_name, parameters.priority_probability);
				}
			}

			net.setPriorNormalDistribution("intention_ample_time", parameters.ample_time_s.mu, parameters.ample_time_s.sigma, parameters.ample_time_s.max / parameters.ample_time_s.n_bins);
			net.setPriorNormalDistribution("intention_safe_distance", parameters.safe_distance_m.mu, parameters.safe_distance_m.sigma, parameters.safe_distance_m.max / parameters.safe_distance_m.n_bins);
			net.setPriorNormalDistribution("intention_safe_distance_midpoint", parameters.safe_distance_midpoint_m.mu, parameters.safe_distance_midpoint_m.sigma, parameters.safe_distance_midpoint_m.max / parameters.safe_distance_midpoint_m.n_bins);
			net.setPriorNormalDistribution("intention_safe_distance_front", parameters.safe_distance_front_m.mu, parameters.safe_distance_front_m.sigma, parameters.safe_distance_front_m.max / parameters.safe_distance_front_m.n_bins);

			//net.save_network("network_with_config_probabilities");
		}

		bool insertObservation(const std::map<int, Eigen::Vector4d> &ship_states, std::vector<int> currently_tracked_ships, bool is_changing_course, double time, std::ofstream &myfile)
		{
			//node_state_msg->header.stamp = ros::Time::now();
			//measurement_msgs->header.stamp = ros::Time::now();

			//node_state_msg->reference_ship_id = my_id;
			//measurement_msgs->reference_ship_id = my_id;

			bool did_save = false;

			if (doSave(ship_states, time))
			{
				net.incrementTime();
				did_save = true;
			}
			
			//node_state_msg->number_of_timesteps = net.getNumberOfTimeSteps();

			//measurement_msgs->change_in_course_deg = RAD2DEG * (better_at(ship_states, my_id)[CHI] - better_at(initial_ship_states, my_id)[CHI]);
			//measurement_msgs->change_in_course_state_name = changeInCourseIdentifier(parameters, better_at(ship_states, my_id)[CHI], better_at(initial_ship_states, my_id)[CHI]);
			net.setEvidence("change_in_course", changeInCourseIdentifier(parameters, better_at(ship_states, my_id)[CHI], better_at(initial_ship_states, my_id)[CHI]));
			//std::cout << "change course: " << changeInCourseIdentifier(parameters, better_at(ship_states, my_id)[CHI], better_at(initial_ship_states, my_id)[CHI]) << std::endl;
			//measurement_msgs->change_in_speed_m_s = better_at(ship_states, my_id)[U] - better_at(initial_ship_states, my_id)[U];
			//measurement_msgs->change_in_speed_state_name = changeInSpeedIdentifier(parameters, better_at(ship_states, my_id)[U], better_at(initial_ship_states, my_id)[U]);
			net.setEvidence("change_in_speed", changeInSpeedIdentifier(parameters, better_at(ship_states, my_id)[U], better_at(initial_ship_states, my_id)[U]));

			//measurement_msgs->is_changing_course = is_changing_course;

			net.setEvidence("is_changing_course", is_changing_course);  
			
			//measurement_msgs->ship_measurements.clear();

			std::vector<std::string> handled_ship_names;
			for (auto const &ship_id : currently_tracked_ships)
			{
				if (ship_id != my_id)
				{
					const std::string ship_name = better_at(ship_name_map, ship_id);
					const auto ship_state = better_at(ship_states, ship_id);
					handled_ship_names.push_back(ship_name);
					//std::cout << "ship_id " << ship_id << std::endl;
					//custom_msgs::IntentionMeasurementSingleShip single_ship_meas_msg;
					//single_ship_meas_msg.measured_ship_id = ship_id;

					CPA cpa = evaluateCPA(better_at(ship_states, my_id), ship_state);

					net.setEvidence("disable_" + ship_name, "enabled");

					std::cout <<cpa.time_untill_CPA << std::endl;
					//single_ship_meas_msg.time_untill_CPA_state_id = timeIdentifier(parameters, cpa.time_untill_CPA);
					//std::cout << "timeidentifyer: " << timeIdentifier(parameters, cpa.time_untill_CPA) << std::endl;
					net.setEvidence("time_untill_closest_point_of_approach_towards_" + ship_name, timeIdentifier(parameters, cpa.time_untill_CPA));

					///single_ship_meas_msg.distance_at_CPA_m = cpa.distance_at_CPA;
					//single_ship_meas_msg.distance_at_CPA_state_id = highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA);
					net.setEvidence("distance_at_cpa_towards_" + ship_name, highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));

					double crossing_in_front_distance = crossingInFrontDistance(better_at(ship_states, my_id), ship_state);

					//single_ship_meas_msg.crossing_distance_front_m = crossing_in_front_distance;
					//single_ship_meas_msg.crossing_distance_front_state_id = crossInFrontHighresIdentifier(parameters, crossing_in_front_distance);
					net.setEvidence("crossing_distance_front_towards_" + ship_name, crossInFrontHighresIdentifier(parameters, crossing_in_front_distance));

					auto distanceToMidpointResult = distanceToMidpointCourse(better_at(ship_states, my_id), ship_state);
					//single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_m = distanceToMidpointResult.distance_to_midpoint;
					//single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_state_id = twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint);
					net.setEvidence("two_times_distance_to_midpoint_at_cpa_to_" + ship_name, twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint));

					//single_ship_meas_msg.crossing_with_midpoint_on_side = crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side);
					net.setEvidence("crossing_with_midpoint_on_side_"+ship_name, crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side));
					std::cout << "midpoint side: " << distanceToMidpointResult.crossing_with_midpoint_on_port_side << std::endl;

					//single_ship_meas_msg.aft_front_crossing_side = frontAftIdentifier(cpa.passing_in_front);
					net.setEvidence("aft_front_crossing_side_to_" + ship_name, frontAftIdentifier(cpa.passing_in_front));
					//std::cout << "in front: " << cpa.passing_in_front << std::endl;
					std::cout << "aft side: " << frontAftIdentifier(cpa.passing_in_front) << std::endl;

					//single_ship_meas_msg.passed = hasPassedIdentifier(cpa.time_untill_CPA);
					net.setEvidence("passed_" + ship_name, hasPassedIdentifier(cpa.time_untill_CPA));
					std::cout << "passed: " << hasPassedIdentifier(cpa.time_untill_CPA) << std::endl;

					//single_ship_meas_msg.port_starboard_crossing_side = crossing_port_starboard_identifier(cpa.bearing_relative_to_heading);
					net.setEvidence("crossing_wiht_other_on_port_side_to_" + ship_name, crossing_port_starboard_identifier(cpa.bearing_relative_to_heading));
					//std::cout << "crossing other on port: " << crossing_port_starboard_identifier(cpa.bearing_relative_to_heading) << std::endl;

					//measurement_msgs->ship_measurements.push_back(single_ship_meas_msg);
				}
			}

	
			for (const auto ship_name1 : ship_names)
			{
				if (!std::count(handled_ship_names.begin(), handled_ship_names.end(), ship_name1))
				{
					net.setEvidence("disable_" + ship_name1, "disabled");
				}
			}

			net.setEvidence(output_name, "true");
			evaluate_nodes(myfile);

			return did_save;
			
		}

		// double evaluateTrajectory(const Eigen::MatrixXd &trajectory, const std::map<int, Eigen::Vector4d> &ship_states, std::vector<int> currently_tracked_ships, double dt, custom_msgs::IntentionMeasurement *measurement_msgs)
		// {
		// 	measurement_msgs->header.stamp = ros::Time::now();

		// 	measurement_msgs->reference_ship_id = my_id;

		// 	net.incrementTime();

		// 	unsigned steps_into_trajectory = parameters.time_into_trajectory;

		// 	measurement_msgs->change_in_course_deg = RAD2DEG * (trajectory(CHI, steps_into_trajectory) - better_at(initial_ship_states, my_id)[CHI]);
		// 	measurement_msgs->change_in_course_state_name = changeInCourseIdentifier(parameters, trajectory(CHI, steps_into_trajectory), better_at(initial_ship_states, my_id)[CHI]);
		// 	net.setEvidence("change_in_course", changeInCourseIdentifier(parameters, trajectory(CHI, steps_into_trajectory), better_at(initial_ship_states, my_id)[CHI]));

		// 	measurement_msgs->change_in_speed_m_s = trajectory(U, steps_into_trajectory) - better_at(initial_ship_states, my_id)[U];
		// 	measurement_msgs->change_in_speed_state_name = changeInSpeedIdentifier(parameters, trajectory(U, steps_into_trajectory), better_at(initial_ship_states, my_id)[U]);
		// 	net.setEvidence("change_in_speed", changeInSpeedIdentifier(parameters, trajectory(U, steps_into_trajectory), better_at(initial_ship_states, my_id)[U]));

		// 	measurement_msgs->is_changing_course = false;
		// 	net.setEvidence("is_changing_course", false);

		// 	measurement_msgs->ship_measurements.clear();
		// 	std::vector<std::string> handled_ship_names;
		// 	for (auto const ship_id : currently_tracked_ships)
		// 	{
		// 		if (ship_id != my_id)
		// 		{
		// 			std::string ship_name = better_at(ship_name_map, ship_id);
		// 			handled_ship_names.push_back(ship_name);
		// 			const auto ship_state = better_at(ship_states, ship_id);
		// 			custom_msgs::IntentionMeasurementSingleShip single_ship_meas_msg;
		// 			single_ship_meas_msg.measured_ship_id = ship_id;

		// 			net.setEvidence("disable_" + ship_name, "enabled");

		// 			CPA cpa = evaluateCPA(trajectory, ship_state, dt);
		// 			auto minimum_acceptable_ample_time = parameters.ample_time_s.minimal_accepted_by_ownship;

		// 			single_ship_meas_msg.time_untill_CPA_sec = minimum_acceptable_ample_time;
		// 			single_ship_meas_msg.time_untill_CPA_state_id = timeIdentifier(parameters, minimum_acceptable_ample_time);
		// 			net.setEvidence("time_untill_closest_point_of_approach_towards_" + ship_name, timeIdentifier(parameters, minimum_acceptable_ample_time));

		// 			single_ship_meas_msg.distance_at_CPA_m = cpa.distance_at_CPA;
		// 			single_ship_meas_msg.distance_at_CPA_state_id = highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA);
		// 			net.setEvidence("distance_at_cpa_towards_" + ship_name, highresCPADistanceIdentifier(parameters, cpa.distance_at_CPA));

		// 			double crossing_in_front_distance = crossingInFrontDistanceTrajectory(trajectory, ship_state, dt);

		// 			single_ship_meas_msg.crossing_distance_front_m = crossing_in_front_distance;
		// 			single_ship_meas_msg.crossing_distance_front_state_id = crossInFrontHighresIdentifier(parameters, crossing_in_front_distance);
		// 			net.setEvidence("crossing_distance_front_towards_" + ship_name, crossInFrontHighresIdentifier(parameters, crossing_in_front_distance));

		// 			auto distanceToMidpointResult = distanceToMidpointTrajectory(trajectory, ship_state, dt);
		// 			single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_m = distanceToMidpointResult.distance_to_midpoint;
		// 			single_ship_meas_msg.two_times_distance_to_midpoint_at_cpa_to_state_id = twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint);
		// 			net.setEvidence("two_times_distance_to_midpoint_at_cpa_to_" + ship_name, twotimesDistanceToMidpointIdentifier(parameters, distanceToMidpointResult.distance_to_midpoint));

		// 			single_ship_meas_msg.crossing_with_midpoint_on_side = crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side);
		// 			net.setEvidence("crossing_with_midpoint_on_side_"+ship_name, crossingWithMidpointOnSideIdentifier(distanceToMidpointResult.crossing_with_midpoint_on_port_side));

		// 			single_ship_meas_msg.aft_front_crossing_side = frontAftIdentifier(cpa.passing_in_front);
		// 			net.setEvidence("aft_front_crossing_side_to_" + ship_name, frontAftIdentifier(cpa.passing_in_front));

		// 			single_ship_meas_msg.passed = hasPassedIdentifier(cpa.time_untill_CPA);
		// 			net.setEvidence("passed_" + ship_name, hasPassedIdentifier(cpa.time_untill_CPA));

		// 			single_ship_meas_msg.port_starboard_crossing_side = crossing_port_starboard_identifier(cpa.bearing_relative_to_heading);
		// 			net.setEvidence("crossing_wiht_other_on_port_side_to_" + ship_name, crossing_port_starboard_identifier(cpa.bearing_relative_to_heading));

		// 			measurement_msgs->ship_measurements.push_back(single_ship_meas_msg);
		// 		}
		// 	}

		// 	for (const auto ship_name : ship_names)
		// 	{
		// 		if (!std::count(handled_ship_names.begin(), handled_ship_names.end(), ship_name))
		// 		{
		// 			net.setEvidence("disable_" + ship_name, "disabled");
		// 		}
		// 	}

		// 	// evaluate_nodes(node_state_msg);
		// 	auto res = net.evaluateStates({output_name});
		// 	measurement_msgs->probability_of_observation = better_at(better_at(res, output_name), "true");

		// 	net.decrementTime();
		// 	return better_at(better_at(res, output_name), "true");
		// }

		/*auto run_inference(const std::map<int, Eigen::Vector4d> &ship_states, std::map<int, Eigen::MatrixXd> trajectory_candidates, double dt)
	{
		struct
		{
			custom_msgs::IntentionNodeState node_state_msg;
			custom_msgs::IntentionMeasurement measurement_msgs;

			std::map<int, custom_msgs::IntentionNodeState> trajectory_node_state_msg;
			std::map<int, custom_msgs::IntentionMeasurement> trajectory_measurement_msgs;

			std::map<int, double> traj_probability;
		} res;


		auto observe_res = insertObservation(ship_states);
		res.node_state_msg = observe_res.node_state_msg;
		res.measurement_msgs = observe_res.measurement_msgs;
		std::map<int, double> res;
		for (auto const &[traj_id, trajecotry] : trajectory_candidates)
		{
			//TOdo implement for trajectories!

			//res.emplace(traj_id, evaluateTrajectory(trajecotry, ship_states, dt));
		}
		return res;
	}*/
	};
}
