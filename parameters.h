#pragma once
#include <map>
#include <string>
#include <vector>

namespace INTENTION_INFERENCE
{
struct IntentionModelParameters{
    unsigned number_of_network_evaluation_samples;
    unsigned max_number_of_obstacles;
    unsigned time_into_trajectory;
    struct{
        double min_time_s;
        double max_time_s;
        double min_course_change_rad;
        double min_speed_change_m_s;
    } expanding_dbn;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
        double minimal_accepted_by_ownship;
    }ample_time_s;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_midpoint_m;
    struct{
        double mu;
        double sigma;
        double max;
        unsigned n_bins;
    }safe_distance_front_m;
    struct{
        double minimal_change;
    }change_in_course_rad;
    struct{
        double minimal_change;
    }change_in_speed_m_s;
    struct{
        double HO_uncertainty_start;
        double HO_start;
        double HO_stop;
        double HO_uncertainty_stop;
        double OT_uncertainty_start;
        double OT_start;
        double OT_stop;
        double OT_uncertainty_stop;
    }colregs_situation_borders_rad;
    double ignoring_safety_probability;
    double colregs_compliance_probability;
    double good_seamanship_probability;
    double unmodeled_behaviour;
    std::map<std::string, double> priority_probability;
};
}
