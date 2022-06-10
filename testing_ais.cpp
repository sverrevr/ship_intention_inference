#include "Eigen/Dense"
#include "math.h"
#include <map>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

void read_ais_to_ship_states(){
    using namespace std;
    vector<Eigen::Vector4d> current_ship_states;
    ifstream ifile("new_case_LQLVS-60-sec.csv");
    int ship_mmsi = 248375000; //wanted ship for current states
    vector<int> mmsi_vec;
    vector<string> time_vec;
    vector<double> x_vec, y_vec, sog_vec, cog_vec;
    int mmsi;
    string time;
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
            getline(iss, time, ',');
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
            time_vec.push_back(time);
            x_vec.push_back(x);
            y_vec.push_back(y);
            sog_vec.push_back(sog);
            cog_vec.push_back(cog);
            
            if(mmsi==ship_mmsi){
                vector<Eigen::VectorXd> ship_states;
                ship_states.push_back(Eigen::VectorXd(4));
                ship_states[0] << x,y,sog,cog; 
                current_ship_states.push_back(ship_states[0]);
            }

        }
    
    }
     
     
    for (int i= 0; i <=3; i++){
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