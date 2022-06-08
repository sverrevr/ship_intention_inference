/*
This file contains functions that translates continious values into discrete states used by the bayesian network.
*/

#pragma once
#include <math.h>
#include <map>
#include <algorithm>
#include <limits.h>
#include <string>
#include "geometry.h"
#include "parameters.h"

namespace INTENTION_INFERENCE
{
    unsigned discretizer(double input, int max, int n_bins)
    {
        if (!std::isfinite(input) || std::floor(input * n_bins / max) > INT_MAX)
            return n_bins - 1;
        else
            return std::clamp(int(std::floor(input * n_bins / max)), 0, n_bins - 1);
    }

    unsigned timeIdentifier(const IntentionModelParameters &parameters, double time_s)
    {
        return discretizer(time_s, parameters.ample_time_s.max, parameters.ample_time_s.n_bins);
    }
    unsigned highresCPADistanceIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(distance_m, parameters.safe_distance_m.max, parameters.safe_distance_m.n_bins);
    }
    unsigned twotimesDistanceToMidpointIdentifier(const IntentionModelParameters &parameters, double distance_to_midpoint_m)
    {
        return discretizer(2 * distance_to_midpoint_m, parameters.safe_distance_midpoint_m.max, parameters.safe_distance_midpoint_m.n_bins);
    }
    unsigned crossInFrontHighresIdentifier(const IntentionModelParameters &parameters, double distance_m)
    {
        return discretizer(2 * distance_m, parameters.safe_distance_front_m.max, parameters.safe_distance_front_m.n_bins);
    }

    //Side of other ship at CPA
    std::string sideIdentifier(double angle_diff)
    {
        return angle_diff >= 0 ? "port" : "starboard";
    }

    std::string frontAftIdentifier(bool passing_in_front)
    {
        return passing_in_front ? "front" : "aft";
    }

    std::string crossingWithMidpointOnSideIdentifier(bool port_side)
    {
        return port_side ? "port" : "starboard";
    }

    std::string hasPassedIdentifier(double time)
    {
        if (time <= 0)
            return "true";
        else
            return "false";
    }

    std::string crossing_port_starboard_identifier(double relative_bearing)
    {
        if (relative_bearing < 0)
            return "port";
        else
            return "starboard";
    }

    std::string changeInCourseIdentifier(const IntentionModelParameters &parameters, double current_course, double initial_course)
    {
        auto minimal_change = parameters.change_in_course_rad.minimal_change;
        if (current_course - initial_course > minimal_change)
        {
            return "starboardwards";
        }
        else if (initial_course - current_course > minimal_change)
        {
            return "portwards";
        }
        return "none";
    }

    std::string changeInSpeedIdentifier(const IntentionModelParameters &parameters, double current_speed, double initial_speed)
    {
        auto minimal_change = parameters.change_in_speed_m_s.minimal_change;
        if (current_speed - initial_speed > minimal_change)
        {
            return "higher";
        }
        else if (initial_speed - current_speed > minimal_change)
        {
            return "lower";
        }
        return "similar";
    }

    std::map<std::string, double> evaluateSitution(const IntentionModelParameters &parameters, const Eigen::Vector4d &ownship_state, const Eigen::Vector4d &obstacle_state)
    {
        const auto &p = parameters.colregs_situation_borders_rad;

        double relative_heading = obstacle_state(CHI) - ownship_state(CHI);
        wrapPI(&relative_heading);

        //Check bearing relative to ownship heading to see if we are being overtaken
        const double angle_from_own_to_obstacle_ship = std::atan2(obstacle_state(PY) - ownship_state(PY), obstacle_state(PX) - ownship_state(PX));
        double bearing_relative_to_ownship_heading = angle_from_own_to_obstacle_ship - ownship_state(CHI);
        wrapPI(&bearing_relative_to_ownship_heading);

        //Check bearing realtive to obstacleship heading to see if we are overtaking
        const double angle_from_obstacle_to_own = std::atan2(ownship_state(PY) - obstacle_state(PY), ownship_state(PX) - obstacle_state(PX));
        double bearing_relative_to_obstacle_heading = angle_from_obstacle_to_own - obstacle_state(CHI);
        wrapPI(&bearing_relative_to_obstacle_heading);

        std::map<std::string, double> result;
        result["HO"] = 0;
        result["CR_PS"] = 0;
        result["CR_SS"] = 0;
        result["OT_en"] = 0;
        result["OT_ing"] = 0;

        double CR = 0; //crossing is added to the results later di be ble to distinguish between CR_PS adn CR_SS
        if (relative_heading > p.HO_start || relative_heading < p.HO_stop)
        {
            result.at("HO") = 1;
        }
        else if (relative_heading > p.HO_uncertainty_start && relative_heading < p.HO_start)
        {
            result.at("HO") = (relative_heading - p.HO_uncertainty_start) / (p.HO_start - p.HO_uncertainty_start);
            CR = (p.HO_start - relative_heading) / (p.HO_start - p.HO_uncertainty_start);
        }
        else if (relative_heading < p.HO_uncertainty_stop && relative_heading > p.HO_stop)
        {
            result.at("HO") = (relative_heading - p.HO_uncertainty_stop) / (p.HO_stop - p.HO_uncertainty_stop);
            CR = (p.HO_stop - relative_heading) / (p.HO_stop - p.HO_uncertainty_stop);
        }
        else if (bearing_relative_to_ownship_heading > p.OT_start || bearing_relative_to_ownship_heading < p.OT_stop)
        {
            result.at("OT_en") = 1;
        }
        else if (bearing_relative_to_ownship_heading > p.OT_uncertainty_start && bearing_relative_to_ownship_heading < p.OT_start)
        {
            result.at("OT_en") = (bearing_relative_to_ownship_heading - p.OT_uncertainty_start) / (p.OT_start - p.OT_uncertainty_start);
            CR = (p.OT_start - bearing_relative_to_ownship_heading) / (p.OT_start - p.OT_uncertainty_start);
        }
        else if (bearing_relative_to_ownship_heading < p.OT_uncertainty_stop && bearing_relative_to_ownship_heading > p.OT_stop)
        {
            result.at("OT_en") = (bearing_relative_to_ownship_heading - p.OT_uncertainty_stop) / (p.OT_stop - p.OT_uncertainty_stop);
            CR = (p.OT_stop - bearing_relative_to_ownship_heading) / (p.OT_stop - p.OT_uncertainty_stop);
        }
        else if (bearing_relative_to_obstacle_heading > p.OT_start || bearing_relative_to_obstacle_heading < p.OT_stop)
        {
            result.at("OT_ing") = 1;
        }
        else if (bearing_relative_to_obstacle_heading > p.OT_uncertainty_start && bearing_relative_to_obstacle_heading < p.OT_start)
        {
            result.at("OT_ing") = (bearing_relative_to_obstacle_heading - p.OT_uncertainty_start) / (p.OT_start - p.OT_uncertainty_start);
            CR = (p.OT_start - bearing_relative_to_obstacle_heading) / (p.OT_start - p.OT_uncertainty_start);
        }
        else if (bearing_relative_to_obstacle_heading < p.OT_uncertainty_stop && bearing_relative_to_obstacle_heading > p.OT_stop)
        {
            result.at("OT_ing") = (bearing_relative_to_obstacle_heading - p.OT_uncertainty_stop) / (p.OT_stop - p.OT_uncertainty_stop);
            CR = (p.OT_stop - bearing_relative_to_obstacle_heading) / (p.OT_stop - p.OT_uncertainty_stop);
        }
        else
        {
            CR = 1;
        }

        if (bearing_relative_to_ownship_heading < 0)
        {
            result.at("CR_PS") = CR;
        }
        else
        {
            result.at("CR_SS") = CR;
        }

        return result;
    }

    std::string compareSpeed(double ownship_speed, double obstacle_speed)
    {
        if (ownship_speed > obstacle_speed)
        {
            return "higher";
        }
        else
        {
            return "lower";
        }
    }

    /*
DENNE er feil, port/starboard gir ikke mening her
std::string compareCourse(double ownship_course, double obstacle_course)
{
    double velocity_angle_diff = ownship_course - obstacle_course;
    wrapPI(&velocity_angle_diff);

    if (velocity_angle_diff >= 0)
        return "port";
    else
        return "starboard";
}*/
}
