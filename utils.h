#pragma once

#include <map>
#include <sstream>
#include <string>
#include <ros/ros.h>

namespace INTENTION_INFERENCE
{
    template <typename map_t>
    auto &better_at(map_t &map, std::string key)
    {
        try
        {
            return map.at(key);
        }
        catch (const std::out_of_range &error)
        {
            std::stringstream ss;
            ss << "Trying to get nonexisting key: " << key << "\n";
            ss << "The following keys exist: ";
            for (const auto &[key, value] : map)
            {
                (void)value;
                ss << key << ", ";
            }
            ROS_ERROR("%s", ss.str().c_str());
            assert(false);
        }
        return map.begin()->second;
    }

    template <typename map_t>
    auto &better_at(map_t &map, int key)
    {
        try
        {
            return map.at(key);
        }
        catch (const std::out_of_range &error)
        {
            std::cout << "HERE";
            std::stringstream ss;
            ss << "Trying to get nonexisting key: " << key << "\n";
            ss << "The following keys exist: ";
            for (const auto &[key, value] : map)
            {
                (void)value;
                ss << key << ", ";
            }
            ROS_ERROR("%s", ss.str().c_str());
            assert(false);
        }
        return map.begin()->second;
    }
}