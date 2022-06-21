#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <math.h>
#include "intention_model.h"
#include "Eigen/Dense"
#include <map>




int main(){
    //using namespace INTENTION_INFERENCE;

    INTENTION_INFERENCE::IntentionModelParameters param;
		param.number_of_network_evaluation_samples = 100000;
		param.max_number_of_obstacles = 1;
		param.time_into_trajectory = 10;
		param.expanding_dbn.min_time_s = 10;
		param.expanding_dbn.max_time_s = 1200;
		param.expanding_dbn.min_course_change_rad = 0.13;
		param.expanding_dbn.min_speed_change_m_s = 0.5;
		param.ample_time_s.mu = 60;
		param.ample_time_s.sigma = 7;
		param.ample_time_s.max = 100;
		param.ample_time_s.n_bins = 30; // this value must match the bayesian network
		param.ample_time_s.minimal_accepted_by_ownship = 20;
		param.safe_distance_m.mu = 15;
		param.safe_distance_m.sigma = 2.5;
		param.safe_distance_m.max = 30;
		param.safe_distance_m.n_bins = 30; // this value must match the bayesian network
		param.safe_distance_midpoint_m.mu = 15;
		param.safe_distance_midpoint_m.sigma = 2.5;
		param.safe_distance_midpoint_m.max = 30;
		param.safe_distance_midpoint_m.n_bins = 30; // this value must match the bayesian network
		param.safe_distance_front_m.mu = 20;
		param.safe_distance_front_m.sigma = 4;
		param.safe_distance_front_m.max = 50;
		param.safe_distance_front_m.n_bins = 30; // this value must match the bayesian network
		param.change_in_course_rad.minimal_change = 7.5;
		param.change_in_speed_m_s.minimal_change = 1;
		param.colregs_situation_borders_rad.HO_uncertainty_start = 160;
		param.colregs_situation_borders_rad.HO_start = 170;
		param.colregs_situation_borders_rad.HO_stop = -170;
		param.colregs_situation_borders_rad.HO_uncertainty_stop = -160;
		param.colregs_situation_borders_rad.OT_uncertainty_start = 100;
		param.colregs_situation_borders_rad.OT_start = 125;
		param.colregs_situation_borders_rad.OT_stop = -125;
		param.colregs_situation_borders_rad.OT_uncertainty_stop = -100;
		param.ignoring_safety_probability = 0;
		param.colregs_compliance_probability = 0.98;
		param.good_seamanship_probability = 0.99;
		param.unmodeled_behaviour = 0.00001;
		param.priority_probability["lower"] = 0.05;
		param.priority_probability["similar"] = 0.9;
		param.priority_probability["higher"] = 0.05;

    std::vector<std::map<int, Eigen::Vector4d > > ship_state;

	int num_ships = 2;
    

    std::ifstream ifile("new_case_LQLVS-60-sec.csv");
    std::vector<int> mmsi_vec;
    std::vector<double> time_vec;
	std::vector<double> new_time_vec;
    std::vector<double> x_vec, y_vec, sog_vec, cog_vec;
    int mmsi;
    time_t time;
    double time_1;
    double x, y, sog, cog;
    std::string str;
    if(ifile.is_open()){
        getline(ifile,str);
        while(getline(ifile,str)){
            std::istringstream iss(str);
            std::string token;
            getline(iss, token, ',');
            mmsi = std::stoi(token);
            
            getline(iss, token, ',');
            struct std::tm td;
            std::istringstream ss(token);
            ss >> std::get_time(&td, "%Y-%m-%d %H:%M:%S"); // or just %T in this case
            std::time_t time = mktime(&td);
            time_1 = time;
            //std::tm local = *std::localtime(&time);
            //std::cout << "local: " << std::put_time(&local, "%c %Z") << '\n';
            //std::cout << token << std::endl;
            //std::cout << time << std::endl;
            
            getline(iss, token, ',');
            x = stod(token);
            int new_x = round(x);
            getline(iss, token, ',');
            y = stod(token);
            getline(iss, token, ',');
            sog = stod(token);
            getline(iss, token, ',');
            cog = stod(token);

            mmsi_vec.push_back(mmsi);
            x_vec.push_back(x);
            y_vec.push_back(y);
            sog_vec.push_back(sog);
            cog_vec.push_back(cog);
            time_vec.push_back(time_1);
            

        }
    
    }

	

	for (int i = 0; i < time_vec.size()/num_ships; i++ ) {
        std::map<int, Eigen::Vector4d> current_ship_states;
        for (int c = 0; c < num_ships; c++){
            int index = c*time_vec.size()/num_ships + i;
            Eigen::Vector4d states(x_vec[index],y_vec[index],cog_vec[index],sog_vec[index]);
            std::map<int,Eigen::Vector4d>::iterator it = current_ship_states.end();
            current_ship_states.insert(it, std::pair<int, Eigen::Vector4d>(mmsi_vec[index],states));
        }
        ship_state.push_back(current_ship_states);
				new_time_vec.push_back(time_vec[i]);

    }
     
    
    /*for (int i= 0; i <2; i++){
        std::cout << "mmsi: " << mmsi_vec[i] << std::endl;
        std::cout << "time: " << time_vec[i] << std::endl;
        std::cout << "x: " << x_vec[i] << std::endl;
        std::cout << "y: " << y_vec[i] << std::endl;
        std::cout << "sog: " << sog_vec[i] << std::endl;
        std::cout << "cog: " << cog_vec[i] << std::endl;
    }  */

    for(int i = 1; i < 5; i++){
            for(auto it = ship_state[i].cbegin(); it != ship_state[i].cend(); ++it){
            std::cout << it->first << " -> " << it->second << std::endl;
            std::cout << " time: " << new_time_vec[i] << std::endl;
        }
    }


    

    std::set<int> s( mmsi_vec.begin(), mmsi_vec.end() );
    std::vector<int> ship_list;
    ship_list.assign( s.begin(), s.end() );

    //for (int ship = 0; ship < ship_list.size(); ++ship){
        //std::cout << ship_list[ship] << std::endl;
    //}

    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;
    ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[0], INTENTION_INFERENCE::IntentionModel("intention_model_two_ships.xdsl",param,ship_list[0],ship_state[1])));
    ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[1], INTENTION_INFERENCE::IntentionModel("intention_model_two_ships.xdsl",param,ship_list[1],ship_state[1])));

    
    
   for(int i = 10; i < 35; i++){
        std::cout << "timestep: " << i << std::endl;
       for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
        std::cout << "ship id:" << ship_id << std::endl;
        current_ship_intention_model.insertObservation(ship_state[i],ship_list,false,new_time_vec[i]);
    }
   }

    
    
    
}