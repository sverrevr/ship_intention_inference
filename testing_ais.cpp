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
#include <cmath>
#include "intention_model.h"
#include "parameters.h"
#include "Eigen/Dense"
#include <map>

void readFileToVecs (std::string filename, std::vector<int> &mmsi_vec, std::vector<double> &time_vec, std::vector<double> &x_vec, std::vector<double> &y_vec, std::vector<double> &sog_vec, std::vector<double> &cog_vec){
    std::string filename_open = "files/"+filename;
    std::ifstream ifile(filename_open);
    std::vector<int> mmsi_vec;
    std::vector<time_t> time_vec;
	std::vector<time_t> new_time_vec;
    std::vector<double> x_vec, y_vec, sog_vec, cog_vec;

    int mmsi;
    time_t time;
    double time_1;
    double x, y, sog, cog;
    std::string str;
    double time_null = 0;
    double time_from_null;

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

            double cog_n = cog;
            double cog_rad = cog_n*M_PI/180;
            cog_vec.push_back(cog_rad);
            if (time_vec.empty()){
                //std::cout << "0: " << time_1 << std::endl;
                time_vec.push_back(time_1);
            }
            else {
                //std::cout << "for: " << time_1 << std::endl;
                double time_from_null = time_1-time_vec[0];
                time_vec.push_back(time_from_null);
            }
        }
        time_vec[0]=0;
    }
}


void vecsToShipStateVectorMap(std::vector<std::map<int, Eigen::Vector4d > > &ship_state, std::vector<double> &unique_time_vec, int num_ships, std::vector<int> mmsi_vec, std::vector<double> time_vec, std::vector<double> x_vec, std::vector<double> y_vec, std::vector<double> sog_vec, std::vector<double> cog_vec){
    int mmsi;
    time_t time;
    double time_1;
    double x, y, sog, cog;
    std::string str;

	for (int i = 0; i < time_vec.size()/num_ships; i++ ) {
        std::map<int, Eigen::Vector4d> current_ship_states;
        for (int c = 0; c < num_ships; c++){
            int index = c*time_vec.size()/num_ships + i;
            Eigen::Vector4d states(x_vec[index],y_vec[index],cog_vec[index],sog_vec[index]);
            std::map<int,Eigen::Vector4d>::iterator it = current_ship_states.end();
            current_ship_states.insert(it, std::pair<int, Eigen::Vector4d>(mmsi_vec[index],states));
        }
        ship_state.push_back(current_ship_states);
				unique_time_vec.push_back(time_vec[i]);
    }
}


std::vector<int> getShipList(std::vector<int> mmsi_vec){
    std::set<int> s( mmsi_vec.begin(), mmsi_vec.end() );
    std::vector<int> ship_list;
    ship_list.assign( s.begin(), s.end() );
    return ship_list;
}


void writeIntentionToFile(INTENTION_INFERENCE::IntentionModelParameters parameters, std::string filename, std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions, std::vector<std::map<int, Eigen::Vector4d > > ship_state, std::vector<int> ship_list, std::vector<double> unique_time_vec, std::vector<double> x_vec, std::vector<double> y_vec){
    std::ofstream intentionFile;
    std::string filename_intention = "intention_"+filename;
    intentionFile.open (filename_intention);
    intentionFile << "mmsi,x,y,time,CR_PS,CR_SS,HO,OT_en,OT_ing,colreg_compliant,good_seamanship,unmodeled_behaviour,priority_lower,priority_similar,priority_higher\n"; //,CR_SS2,CR_PS2,OT_ing2,OT_en2,priority_lower2,priority_similar2,priority_higher2\n";
    
    for(int i = 1; i < unique_time_vec.size() ; i++){ //from 1 because first state might be NaN
        std::cout << "timestep: " << i << std::endl;
        int j= 0;
        for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
            std::cout << "ship_id" << ship_id << std::endl;
            intentionFile << ship_id << ",";
            intentionFile << x_vec[unique_time_vec.size()*j+i] << ",";
            intentionFile << y_vec[unique_time_vec.size()*j+i] << ","; 
            intentionFile << unique_time_vec[i] << ",";
            current_ship_intention_model.insertObservation(parameters, ship_state[i], ship_list, false, unique_time_vec[i], intentionFile);
            j++;
    }
   }
    intentionFile.close(); 
    printf("Finished writing intentions to file \n");
}


INTENTION_INFERENCE::IntentionModelParameters setModelParameters(){
    INTENTION_INFERENCE::IntentionModelParameters param;
    param.number_of_network_evaluation_samples = 100000;
	param.max_number_of_obstacles = 1; //must be set to num_ships-1 or else segmantation fault
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
	param.change_in_course_rad.minimal_change = 0.13;
	param.change_in_speed_m_s.minimal_change = 1;
	param.colregs_situation_borders_rad.HO_uncertainty_start = 2.79;
	param.colregs_situation_borders_rad.HO_start = 2.96;
	param.colregs_situation_borders_rad.HO_stop = -2.96;
	param.colregs_situation_borders_rad.HO_uncertainty_stop = -2.79;
	param.colregs_situation_borders_rad.OT_uncertainty_start = 1.74;
	param.colregs_situation_borders_rad.OT_start = 2.18;
	param.colregs_situation_borders_rad.OT_stop = -2.18;
	param.colregs_situation_borders_rad.OT_uncertainty_stop = -1.74;
	param.ignoring_safety_probability = 0;
	param.colregs_compliance_probability = 0.98;
    param.good_seamanship_probability = 0.99;
	param.unmodeled_behaviour = 0.00001;
	param.priority_probability["lower"] = 0.05;
	param.priority_probability["similar"] = 0.9;
	param.priority_probability["higher"] = 0.05;
    return param;
}


int main(){
    
	int num_ships = 2;
    //std::string filename = "new_Case_LQLVS-60-sec.csv"; //crossing
    //std::string filename = "new_Case - 04-12-2019, 20-10-56 - DOTVP-two-ships-60-sec-kopi.csv";
    //std::string filename = "new_case_2ZC9Z-60-sec-two-ships.csv"; //head on
    //std::string filename = "new_Case - 01-08-2021, 08-21-29 - AQ5VM-60-sec-two-ships.csv"; //overtaking
    std::string filename = "new_Case - 01-15-2020, 09-05-49 - VATEN-60-sec-two-ships.csv"; //overtaking
    std::string intentionModelFilename = "intention_model_two_ships.xdsl";

    std::vector<std::map<int, Eigen::Vector4d> > ship_state;
    std::vector<int> mmsi_vec;
    std::vector<double> time_vec, x_vec, y_vec, sog_vec, cog_vec, unique_time_vec;;

    readFileToVecs(filename, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);


    vecsToShipStateVectorMap(ship_state, unique_time_vec, num_ships, mmsi_vec, time_vec, x_vec, y_vec, sog_vec, cog_vec);

    std::vector<int> ship_list = getShipList(mmsi_vec);



    INTENTION_INFERENCE::IntentionModelParameters parameters = setModelParameters();


    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;

    for (int i = 0; i < num_ships; i++){
        ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[i], INTENTION_INFERENCE::IntentionModel(intentionModelFilename,parameters,ship_list[i],ship_state[1]))); //ship_state[1] as initial as first state might be NaN
    }

    
    writeIntentionToFile(parameters,filename, ship_intentions, ship_state, ship_list, unique_time_vec, x_vec,y_vec); //intentionfile is called: intention_<filename>  NB: not all intentions!
    

    /* OLD PRINTS

    for (int i= 0; i <2; i++){
        std::cout << "mmsi: " << mmsi_vec[i] << std::endl;
        std::cout << "time: " << time_vec[i] << std::endl;
        std::cout << "x: " << x_vec[i] << std::endl;
        std::cout << "y: " << y_vec[i] << std::endl;
        std::cout << "sog: " << sog_vec[i] << std::endl;
        std::cout << "cog: " << cog_vec[i] << std::endl;
    } 


    /*for(int i = 0; i < ship_state.size(); i++){

            for(auto it = ship_state[i].cbegin(); it != ship_state[i].cend(); ++it){
            std::cout << it->first << " -> " << it->second << std::endl;
            std::cout << " time: " << unique_time_vec[i] << std::endl;
        }

    } */

    

    std::set<int> s( mmsi_vec.begin(), mmsi_vec.end() );
    std::vector<int> ship_list;
    ship_list.assign( s.begin(), s.end() );

    //for (int ship = 0; ship < ship_list.size(); ++ship){
        //std::cout << ship_list[ship] << std::endl;
    //}

    std::map<int, INTENTION_INFERENCE::IntentionModel> ship_intentions;
    ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[0], INTENTION_INFERENCE::IntentionModel("intention_model_two_ships.xdsl",param,ship_list[0],ship_state[1])));
	ship_intentions.insert(std::pair<int, INTENTION_INFERENCE::IntentionModel>(ship_list[1], INTENTION_INFERENCE::IntentionModel("intention_model_two_ships.xdsl",param,ship_list[1],ship_state[1])));

	/*for(int i = 0; i < ship_state.size(); i++){
    	for(auto& [ship_name, current_ship_intention_model] : ship_intentions){
        	current_ship_intention_model.insertObservation(ship_state[i],ship_list,false,new_time_vec[i]);

        }
    }
	*/
	/*
	for(auto ship: ship_intentions){
            for(auto& [ship_id, current_ship_intention_model] : ship_intentions){
            //std::cout << ship_id << std::endl;
			//std::cout << current_ship_intention_model << std::endl;
        }
    }
	*/

}