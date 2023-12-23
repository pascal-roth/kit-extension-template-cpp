// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include <carb/BindingsPythonUtils.h>

#include <omni/example/cpp/pybind/ANYmalPublisher.h>

#include <string>

CARB_BINDINGS("omni.example.cpp.pybind.python")

// DISABLE_PYBIND11_DYNAMIC_CAST(omni::example::cpp::pybind::TwistPub)

namespace
{

/**
 * Concrete bound object class that will be reflected to Python.
 */
class PythonBoundObject : public omni::example::cpp::pybind::ANYmalPublisher
{
public:
    /**
     * Factory.
     *
     * @param id Id of the bound action.
     *
     * @return The bound object that was created.
     */
    static carb::ObjectPtr<PythonBoundObject> create(const char* id)
    {
        // Note: It is important to construct the handler using ObjectPtr<T>::InitPolicy::eSteal,
        // otherwise we end up incresing the reference count by one too many during construction,
        // resulting in carb::ObjectPtr<T> instance whose wrapped object will never be destroyed.
        return carb::stealObject<PythonBoundObject>(new PythonBoundObject(id));
    }

    /**
     * Constructor.
     *
     * @param id Id of the bound object.
     */
    PythonBoundObject(const char* id)
        : ExampleBoundObject(id)
        , m_memberInt(0)
    {
    }


    // anymal msg publisher
    // actuator readings publisher
    // readings extended publisher
    void publishReadings(float value)
    {
        m_memberInt *= value;
    }
    // poses publisher
    void publishPoses(float value)
    {
        m_memberInt *= value;
    }
    // twist publisher
    void publishTwist(float stamp, const char* frame_id, float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z)
    {
        geometry_msgs::Vector3 angular_velocity;
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = 1.0;

        geometry_msgs::Vector3 linear_velocity;
        linear_velocity.x = 1.0;
        linear_velocity.y = 0.0;
        linear_velocity.z = 0.0;

        geometry_msgs::Twist twist;
        twist.angular = angular_velocity;
        twist.linear = linear_velocity;

        twist_pub_.publish(twist);

        ros::spinOnce();
    }

    int m_memberInt;

};

// Define the pybind11 module using the same name specified in premake5.lua
PYBIND11_MODULE(_example_pybind_bindings, m)
{
    using namespace omni::example::cpp::pybind;

    m.doc() = "pybind11 omni.example.cpp.pybind bindings";

    py::class_<PythonBoundObject, carb::ObjectPtr<PythonBoundObject>>(m, "ANYmalPublisher")
        .def(py::init([](const char* id) { return PythonBoundObject::create(id); }),
             R"(
             Create a bound object.

             Args:
                 id: Id of the bound object.

             Return:
                 The bound object that was created.
             )",
             py::arg("id"))
        .def_readwrite("property_int", &PythonBoundObject::m_memberInt,
             R"(
             Int property bound directly.
             )")
        .def("publish_twist", &PythonBoundObject::publishTwist,
             R"(
             Bound fuction that returns a value.

             Return:
                 The toggled bool value.
             )")
        /**/;
}
}
