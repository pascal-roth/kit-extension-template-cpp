// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#pragma once

#include <carb/IObject.h>

// Include ros headers
#include <ros/ros.h>
// Include ros twist message
#include <geometry_msgs/Twist.h>

namespace omni
{
namespace example
{
namespace cpp
{
namespace pybind
{

/**
 * Pure virtual bound object interface.
 */
class ANYmalPublisher : public carb::IObject
{
public:
    /**
     * Get the id of this object.
     *
     * @return Id of this object.
     */
    virtual const char* getId() const = 0;

    ANYmalPublisher(const std::string& topic_name);
    void publishTwist(double linear_x, double angular_z);

private:
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
};

}
}
}
}
