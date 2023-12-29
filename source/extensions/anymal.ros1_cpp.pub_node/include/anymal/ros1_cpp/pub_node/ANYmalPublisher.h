// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#pragma once

#include <anymal/ros1_cpp/pub_node/ANYmalBoundObject.h>

#include <carb/ObjectUtils.h>

#include <omni/String.h>

// Include ros headers
#include <ros/ros.h>
// Include ros twist message
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
// ANYmal message
#include <anymal_msgs/AnymalState.h>
#include <anymal_msgs/Contacts.h>
#include <anymal_msgs/Contact.h>

namespace anymal
{
namespace ros1_cpp
{
namespace pub_node
{

/**
 * Pure virtual bound object interface.
 */
class ANYmalPublisher : public ANYmalBoundObject
{
    CARB_IOBJECT_IMPL

public:
    /**
     * Constructor.
     *
     * @param id Id of the bound object.
     */
    ANYmalPublisher(const char* id)
        : m_id(id ? id : "")
    {
    }

    /**
     * @ref ANYmalBoundObject::getId
     */
    const char* getId() const override
    {
        return m_id.c_str();
    }

    void publishTwist(float stamp, const char* frame_id, float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z);
    void publishANYmalMsg(
        double time,
        const std::vector<std::string>& jointNames,
        const std::vector<double>& jointPositions,
        const std::vector<double>& jointVelocities,
        const std::vector<double>& jointAccelerations,
        const std::vector<double>& jointEfforts,
        const std::vector<std::string>& contactBodyNames,
        const std::vector<double>& contactForces,
        const std::vector<double>& contactPositions,
        const std::vector<double>& contactNormals,
        const std::vector<double>& bodyPositions,
        const std::vector<double>& quaternion,  // [x, y, z, w]
        const std::vector<double>& linearVelocities,
        const std::vector<double>& angularVelocities,
        const std::vector<std::string>& frameTransformChildFrames
    );
    static void initializeROS(int argc, char** argv);

protected:
    int argc_;
    char** argv_;

    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Publisher anymal_msg_pub;
    ros::Publisher anymal_msg_throttlePub;

    // Init ANYmal State message
    anymal_msgs::AnymalState anymal_msg_;
    int seqCounterANYmalMsg = 0;
    int throttleCounterANYmalMsg= 0;
    int throttleDecimationANYmalMsg = 1;

    const omni::string m_id; //!< Id of the bound object.
};

}
}
}
