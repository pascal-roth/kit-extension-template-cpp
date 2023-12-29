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

namespace anymal
{
namespace ros1_cpp
{
namespace pub_node
{


/**
 * Pure virtual bound object interface.
 */
class ANYmalBoundObject : public carb::IObject
{
public:
    /**
     * Get the id of this object.
     *
     * @return Id of this object.
     */
    virtual const char* getId() const = 0;
};

/**
 * Implement the equality operator so these can be used in std containers.
 */
inline bool operator==(const carb::ObjectPtr<ANYmalBoundObject>& left,
                       const carb::ObjectPtr<ANYmalBoundObject>& right) noexcept
{
    return (left.get() == right.get());
}

}
}
}
