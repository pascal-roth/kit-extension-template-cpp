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

#include <carb/Interface.h>

namespace anymal
{
namespace ros1_cpp
{
namespace pub_node
{

/**
 * An example interface to demonstrate reflection using pybind.
 */
class ANYmalBoundInterface
{
public:
    /// @private
    CARB_PLUGIN_INTERFACE("anymal::ros1_cpp::pub_node::ANYmalBoundInterface", 1, 0);

    /**
     * Register a bound object.
     *
     * @param object The bound object to register.
     */
    virtual void registerBoundObject(carb::ObjectPtr<ANYmalBoundObject>& object) = 0;

    /**
     * Deregister a bound object.
     *
     * @param object The bound object to deregister.
     */
    virtual void deregisterBoundObject(carb::ObjectPtr<ANYmalBoundObject>& object) = 0;

    /**
     * Find a bound object.
     *
     * @param id Id of the bound object.
     *
     * @return The bound object if it exists, an empty ObjectPtr otherwise.
     */
    virtual carb::ObjectPtr<ANYmalBoundObject> findBoundObject(const char* id) const = 0;
};

}
}
}