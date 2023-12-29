// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>

#include <anymal/ros1_cpp/pub_node/ANYmalBoundInterface.h>

#include <unordered_map>

const struct carb::PluginImplDesc pluginImplDesc = { "anymal.ros1_cpp.pub_node.plugin",
                                                     "C++ extension to publish ros topic for ANYmal.", "RSL",
                                                     carb::PluginHotReload::eEnabled, "dev" };

namespace anymal
{
namespace ros1_cpp
{
namespace pub_node
{

class ANYmalBoundImplementation : public ANYmalBoundInterface
{
public:
    void registerBoundObject(carb::ObjectPtr<ANYmalBoundObject>& object) override
    {
        if (object)
        {
            m_registeredObjectsById[object->getId()] = object;
        }
    }

    void deregisterBoundObject(carb::ObjectPtr<ANYmalBoundObject>& object) override
    {
        if (object)
        {
            const auto& it = m_registeredObjectsById.find(object->getId());
            if (it != m_registeredObjectsById.end())
            {
                m_registeredObjectsById.erase(it);
            }
        }
    }

    carb::ObjectPtr<ANYmalBoundObject> findBoundObject(const char* id) const override
    {
        const auto& it = m_registeredObjectsById.find(id);
        if (it != m_registeredObjectsById.end())
        {
            return it->second;
        }

        return carb::ObjectPtr<ANYmalBoundObject>();
    }

private:
    std::unordered_map<std::string, carb::ObjectPtr<ANYmalBoundObject>> m_registeredObjectsById;
};

}
}
}

CARB_PLUGIN_IMPL(pluginImplDesc, anymal::ros1_cpp::pub_node::ANYmalBoundImplementation)

void fillInterface(anymal::ros1_cpp::pub_node::ANYmalBoundImplementation& iface)
{
}
