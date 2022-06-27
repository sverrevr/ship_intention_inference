#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <numeric> // std::inner_product
#include <cmath>
#include <algorithm>
#include <map>

double find_mean(std::vector<double> vect){
    double sum = accumulate(vect.begin(), vect.end(), 0.0);
    double mean = sum/vect.size();
    return mean;
}

double find_median(std::vector<double> vec) 
{
    sort(vec.begin(), vec.end());     // sort temperatures
            
    double tmedian;
    if (vec.size() % 2 == 0)           // even
        tmedian = (vec[vec.size() / 2 - 1] + vec[vec.size() / 2]) / 2;
    else                                // odd
        tmedian = vec[vec.size() / 2];
    
    return tmedian;
}

double find_sigma(std::vector<double> vect, double mean){
    double sq_sum = inner_product(vect.begin(), vect.end(), vect.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / vect.size() - mean * mean);
    return stdev;
}

void print_info(std::vector<double> v){
    double median  = find_median(v);
    double mean = find_mean(v);
    double sigma = find_sigma(v, mean);
    int n_bins_cpa_OTGW = v.size();

    std::cout << "mean: "<< mean <<" "<<"\n";
    std::cout << "sigma: "<< sigma  <<" "<<"\n";
    std::cout << "median: "<< median<<" "<<"\n";
}

void print_cpa_and_r(std::vector<double> cpa, std::vector<double> r_man){
    std::cout << "cpa: "<<"\n";
    print_info(cpa);
    std::cout <<"\n"<< "r maneuver: "<<"\n";
    print_info(r_man);
}

std::vector<std::vector<std::string> > read_file(std::string filename){
    std::vector<std::vector<std::string> > content;
    std::vector<std::string> row;
    std::string line, num;
    std::fstream file (filename, std::ios::in);
    std::cout << "Reading file" << filename;
    if(file.is_open()){
            while(getline(file, line)){
                row.clear();

                std::stringstream str(line);

                while(std::getline(str, num, ';'))

                row.push_back(num);
                content.push_back(row);
            }
        }
        else
            std::cout << "Could not open file";
    return content;
}

double find_min(std::vector<double> v){
    double min = v[0];
    for(int i=1; i < v.size(); i++){
        if(v[i] < min){
            min = v[i];
        }
    }
    return min;
}

double find_max(std::vector<double> v){
    double max = v[0];
    for(int i=1; i < v.size(); i++){
        if(v[i] > max){
            max = v[i];
        }
    }
    return max;
}

// return map with vectors 
    // [-2] [<234, 453, 567, ..., ...>]
    // [-1] [<345, ..., ...,         >]
    // [3]  [<231, ..., ..., ..., ...>]
std::map<int, std::vector<double> > aisMap(std::vector<std::vector<std::string> > content, int colreg_idx, int cpa_idx, int timestep){
    std::map<int, std::vector<double> > ais_cases;
   
    for(int i=1;i<content.size();i++){   //start at 1 to not include name 
        std::string colreg_situation = content[i][colreg_idx];
        int col = stoi(colreg_situation);

        if(col == -2){
            std::string cpa_val_OTGW = content[i][cpa_idx];
            double cpa_OTGW = stod(cpa_val_OTGW)*timestep;
            ais_cases[-2].push_back(cpa_OTGW);
        }
        else if(col == -1){
            std::string cpa_val_CRGW = content[i][cpa_idx];
            double cpa_CRGW = stod(cpa_val_CRGW)*timestep;
            ais_cases[-1].push_back(cpa_CRGW);
        }
        else{
            std::string cpa_val_HO = content[i][cpa_idx];
            double cpa_HO = stod(cpa_val_HO)*timestep;
            ais_cases[3].push_back(cpa_HO);
        }
    }
    return ais_cases;
} 

std::vector<double> find_distribution(std::vector<double> v, int n_bins){
    double min = find_min(v);
    double max = find_max(v);
    int num_intervals = n_bins; //want 30 intervals
    double size_of_interval = (max - min) / num_intervals;  
    double start_interval = min;
    int sum = 0;
    std::vector<int> instance_count_vec(num_intervals,0);
    
    for(int j=0; j < num_intervals ; j++){
        for(int i=0; i < v.size(); i++){
            double end_interval = start_interval+size_of_interval;
            if ((v[i] > start_interval) && (v[i] < end_interval)){
                instance_count_vec[j] += 1;
                }
            }
        start_interval += size_of_interval;
    }

    double tot_sum = 0;
    tot_sum = std::accumulate(instance_count_vec.begin(), instance_count_vec.end(),0);
    
    std::vector<double> dist(num_intervals, 0);
    for(int i=0; i<num_intervals;i++){
        dist[i] = (instance_count_vec[i]/tot_sum)*100;
    }
    
    for(int i=0; i < num_intervals; i++){
        std::cout << dist[i]<<" ";
    }
    return dist;
}

std::map<int, std::vector<double> > distributionMap(std::map<int, std::vector<double> > ais_map, int n_bins){
    std::map<int, std::vector<double> > distributionMap;
    for(std::map<int, std::vector<double> >::iterator it=ais_map.begin(); it != ais_map.end(); ++it){
            int col = (*it).first;
            std::vector<double> inVect = (*it).second;
            std::vector<double> dist = find_distribution(inVect, n_bins);
            distributionMap[col] = dist;
        }
    return distributionMap;
}

void printDist(std::map<int, std::vector<double> > my_map){
    std::cout << "Distribution data: \n";
    for(std::map<int, std::vector<double> >::iterator it=my_map.begin(); it != my_map.end(); ++it){
        std::cout << (*it).first << " : ";
        std::vector<double> inVect = (*it).second;
        for(unsigned j=0; j<inVect.size();j++){
            std::cout << inVect[j] << " ";
        }
        std::cout << std::endl;
    }
}

int main(){
    
    const std::string filename = "classified_west.csv";
    const std::string filename2 = "data_west.csv";

    std::vector<std::vector<std::string> > content = read_file(filename);

    int colreg_idx = 7;
    int cpa_ts_idx = 4;  // per nå lik r_maneuver_own (skal byttes til cpa_ts_idx)
    int cpa_dist_idx = 6;
    
    int timestep = 60;
    int n_bins = 30;

    // CPA distance, time step lik 1 
    std::map<int, std::vector<double> > ais_cpa_map = aisMap(content, colreg_idx, cpa_dist_idx, 1);
    std::map<int, std::vector<double> > distr_cpa_map = distributionMap(ais_cpa_map, n_bins);

    printDist(distr_cpa_map);

    // CPA time, time step lik 60
    // Fiks verdier    
    
    return 0;
}

