#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <numeric> // std::inner_product
#include <cmath>
#include <algorithm>

//using namespace std;

//find mu/mean, sigma/standard deviation, max, n_bins 

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

int main(){
    
    //std::vector<std::vector<std::string> > cont;
    //std::vector<std::string> row;
    std::vector<double> cpa_vector_OTGW;
    std::vector<double> cpa_vector_CRGW;
    std::vector<double> cpa_vector_HO;
    std::vector<double> r_man_vector_OTGW; 
    std::vector<double> r_man_vector_CRGW;
    std::vector<double> r_man_vector_HO;
        
    //std::string line, num;
    const std::string filename = "classified_west.csv";
    std::vector<std::vector<std::string> > content = read_file(filename);

    int colreg_idx = 7;
    int r_man_idx = 4;
    int cpa_idx = 6;
    // Find relevant column and extract info, make function
    for(int i=1;i<content.size();i++){   //start at 1 to not include name 
    //for(int i=1;i<100;i++){  //keep for testing, switch to upper one with time
        //cout<<" own_mmsi: "<<content[i][0]<<" "<< "obst_mmsi: "<< content[i][1]<<" "<< "r_cpa: "<< content[i][7]<<" "<< "r_manouvre: "<< content[i][6]<<" "<< "colreg_type: "<< content[i][8];
        //std::cout<<"\n";
        std::string colreg_situation = content[i][colreg_idx];
        double col = stod(colreg_situation);

        if(col == -2){
            std::string cpa_val_OTGW = content[i][cpa_idx];
            double d_cpa_OTGW = stod(cpa_val_OTGW);
            cpa_vector_OTGW.push_back(d_cpa_OTGW);

            std::string r_maneuver_own_OTGW = content[i][r_man_idx];
            double r_man_OTGW = stod(r_maneuver_own_OTGW);
            r_man_vector_OTGW.push_back(r_man_OTGW);
        }
        else if(col == -1){
            std::string cpa_val_CRGW = content[i][cpa_idx];
            double d_cpa_CRGW = stod(cpa_val_CRGW);
            cpa_vector_CRGW.push_back(d_cpa_CRGW);

            std::string r_maneuver_own_CRGW = content[i][r_man_idx];
            double r_man_CRGW = stod(r_maneuver_own_CRGW);
            r_man_vector_CRGW.push_back(r_man_CRGW);
        }
        else{
            std::string cpa_val_HO = content[i][6];
            double d_cpa_HO = stod(cpa_val_HO);
            cpa_vector_HO.push_back(d_cpa_HO);

            std::string r_maneuver_own_HO = content[i][4];
            double r_man_HO = stod(r_maneuver_own_HO);
            r_man_vector_HO.push_back(r_man_HO);
        }
    }

    double min = find_min(cpa_vector_OTGW);
    double max = find_max(cpa_vector_OTGW);
    int num_intervals = 30; //want 30 intervals
    double size_of_interval = (max - min) / num_intervals;  
    //std::cout << size_of_interval <<" ";
    double start_interval = min;
    //std::cout <<start_interval;
    double curr_start;
    //map<int, double> Distribution;   // <interval, num_elements> 
    std::vector<std::vector<double> > intervals;
    std::vector<double> one_interval;
    int sum = 0;
    int array[30] = {0};
    
    for(int j=0; j < 30 ; j++){
        for(int i=0; i < cpa_vector_OTGW.size(); i++){
        //std::cout << cpa_vector_OTGW[i] << " ";
        double end_interval = start_interval+size_of_interval;
        if ((cpa_vector_OTGW[i] > start_interval) && (cpa_vector_OTGW[i] < end_interval)){
            //std::cout << "found: " << cpa_vector_OTGW[i];
            array[j] += 1;
            }
        }
        start_interval += size_of_interval;
    }
    
    int size = sizeof(array)/sizeof(array[0]);
    for(int i=0; i < size; i++){
        //std::cout << sizeof(array);
        std::cout << array[i]<< " ";
    }
    /*
    std::cout << "\n" << "OTGW, Overtake, Give Away: "<<"\n\n";
    print_cpa_and_r(cpa_vector_OTGW,r_man_vector_OTGW);

    std::cout << "\n"<< "CRGW / Crossing, Give Away: "<<"\n\n";
    print_cpa_and_r(cpa_vector_CRGW, r_man_vector_CRGW);

    std::cout << "\n" << "HO / Head On: "<<"\n\n";
    print_cpa_and_r(cpa_vector_HO, r_man_vector_HO);
    */
    return 0;
}

