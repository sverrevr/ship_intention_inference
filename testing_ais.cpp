#include "Eigen/Dense"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>

void read_ais_to_ship_states(){
    using namespace std;
    vector<Eigen::Vector4d> current_ship_states;
    ifstream ifile("files/new_case_LQLVS-60-sec.csv");
    int ship_mmsi = 248375000; //wanted ship for current states
    vector<int> mmsi_vec;
    vector<time_t> time_vec;
    vector<double> x_vec, y_vec, sog_vec, cog_vec;
    int mmsi;
    time_t time;
    double x, y, sog, cog;
    string str;
    if(ifile.is_open()){
        //while ( ifile.good()) {
        //getline(ifile, myline);
        //cout << myline << ": " << ifile.tellg() << '\n';
        getline(ifile,str);
        while(getline(ifile,str)){
            istringstream iss(str);
            string token;
            getline(iss, token, ',');
            mmsi = stoi(token);
            
            getline(iss, token, ',');
            struct std::tm td;
            istringstream ss(token);
            ss >> std::get_time(&td, "%Y-%m-%d %H:%M:%S"); // or just %T in this case
            std::time_t time = mktime(&td);
            std::tm local = *std::localtime(&time);
            //std::cout << "local: " << std::put_time(&local, "%c %Z") << '\n';
            //cout << token << std::endl;
            //cout << time << std::endl;
            
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
            
            if(mmsi==ship_mmsi){
                time_vec.push_back(time);
                vector<Eigen::VectorXd> ship_states;
                ship_states.push_back(Eigen::VectorXd(4));
                ship_states[0] << x,y,sog,cog; 
                current_ship_states.push_back(ship_states[0]);
            }

        }
    
    }
     
     
    for (int i= 0; i <2; i++){
        cout << "mmsi: " << mmsi_vec[i] << endl;
        cout << "time: " << time_vec[i] << endl;
        cout << "current states: " << current_ship_states[i] << endl;
        cout << "x: " << x_vec[i] << endl;
        cout << "y: " << y_vec[i] << endl;
        cout << "sog: " << sog_vec[i] << endl;
        cout << "cog: " << cog_vec[i] << endl;
    }  

}

int main(){
    read_ais_to_ship_states();

}