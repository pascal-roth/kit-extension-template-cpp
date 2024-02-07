// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include <carb/BindingsPythonUtils.h>

#include <anymal/ros1_cpp/pub_node/ANYmalPublisher.h>
#include <anymal/ros1_cpp/pub_node/ANYmalBoundInterface.h>

#include <string>

CARB_BINDINGS("anymal.ros1_cpp.pub_node.python")

DISABLE_PYBIND11_DYNAMIC_CAST(anymal::ros1_cpp::pub_node::ANYmalBoundInterface)
DISABLE_PYBIND11_DYNAMIC_CAST(anymal::ros1_cpp::pub_node::ANYmalBoundObject)

namespace
{

/**
 * Concrete bound object class that will be reflected to Python.
 */
class PythonANYmalBound : public anymal::ros1_cpp::pub_node::ANYmalPublisher
{
public:
    /**
     * Factory.
     *
     * @param id Id of the bound action.
     *
     * @return The bound object that was created.
     */
    static carb::ObjectPtr<PythonANYmalBound> create(const char* id)
    {
        // Note: It is important to construct the handler using ObjectPtr<T>::InitPolicy::eSteal,
        // otherwise we end up incresing the reference count by one too many during construction,
        // resulting in carb::ObjectPtr<T> instance whose wrapped object will never be destroyed.
        return carb::stealObject<PythonANYmalBound>(new PythonANYmalBound(id));
    }

    // Initialize ROS node
    static void initializeROS(int argc, char** argv) {
        // Initialize ROS node
        ros::init(argc, argv, "anymal_publisher");
        ROS_INFO("ROS initialized");
    }

    /**
     * Constructor.
     *
     * @param id Id of the bound object.
     */
    PythonANYmalBound(const char* id)
        : ANYmalPublisher(id)
        , m_memberInt(0)
    {
        // Initialize ROS node
        ros::NodeHandle nh("~");

        // Advertise the Twist publisher
        twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // You can add other initializations here

        // Print a message to confirm initialization
        ROS_INFO("ANYmalPublisher initialized");

        // Spin in the background to handle ROS callbacks
        ros::AsyncSpinner spinner(1);
        spinner.start();
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
    }
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
        const std::vector<std::string>& frameTransformChildFrames)
    {
        // Get current time
        ros::Time currTimeRos = ros::Time::now().fromSec(time);

        // Add header
        std_msgs::Header header;
        header.stamp = currTimeRos;
        header.frame_id = "odom";
        header.seq = seqCounterANYmalMsg;
        anymal_msg_.header = header;
        // Define state
        anymal_msg_.state = 0;  // Assuming STATE_OK = 0

        // Add base pose
        anymal_msg_.pose.header = header;
        anymal_msg_.pose.pose.position.x = bodyPositions[0];  // Replace with actual values
        anymal_msg_.pose.pose.position.y = bodyPositions[1];
        anymal_msg_.pose.pose.position.z = bodyPositions[2];
        anymal_msg_.pose.pose.orientation.x = quaternion[0];
        anymal_msg_.pose.pose.orientation.y = quaternion[1];
        anymal_msg_.pose.pose.orientation.z = quaternion[2];
        anymal_msg_.pose.pose.orientation.w = quaternion[3];

        // Add joint states
        anymal_msg_.joints.header = header;
        anymal_msg_.joints.name = jointNames;
        anymal_msg_.joints.position = jointPositions;
        anymal_msg_.joints.velocity = jointVelocities;
        anymal_msg_.joints.acceleration = jointAccelerations;
        anymal_msg_.joints.effort = jointEfforts;

        // Add twist
        anymal_msg_.twist.header = header;
        anymal_msg_.twist.twist.linear.x = linearVelocities[0];
        anymal_msg_.twist.twist.linear.y = linearVelocities[1];
        anymal_msg_.twist.twist.linear.z = linearVelocities[2];
        anymal_msg_.twist.twist.angular.x = angularVelocities[0];
        anymal_msg_.twist.twist.angular.y = angularVelocities[1];
        anymal_msg_.twist.twist.angular.z = angularVelocities[2];

        // Add contacts
        anymal_msgs::Contacts contacts_msg;
        for (size_t i = 0; i < contactBodyNames.size(); ++i) {
            anymal_msgs::Contact contact;
            contact.header = header;
            contact.wrench.force.x = contactForces[i];
            contact.wrench.force.y = contactForces[i + 1];
            contact.wrench.force.z = contactForces[i + 2];
            contact.wrench.torque.x = 0.0;
            contact.wrench.torque.y = 0.0;
            contact.wrench.torque.z = 0.0;
            contact.position.x = contactPositions[i];
            contact.position.y = contactPositions[i + 1];
            contact.position.z = contactPositions[i + 2];
            contact.normal.x = contactNormals[i];
            contact.normal.y = contactNormals[i + 1];
            contact.normal.z = contactNormals[i + 2];
            contact.frictionCoefficient = 0.8;
            contact.restitutionCoefficient = 0.0;
            contacts_msg.contacts.push_back(contact);
        }
        // Assign the vector to the contacts field
        anymal_msg_.contacts = contacts_msg.contacts;

        // Add frame transforms
        std::vector<geometry_msgs::TransformStamped> frameTransforms;
        for (const std::string& childFrame : frameTransformChildFrames) {
            geometry_msgs::TransformStamped transform;
            transform.header = header;
            transform.child_frame_id = childFrame;
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 0.0;
            transform.transform.rotation.w = 1.0;
            frameTransforms.push_back(transform);
        }
        anymal_msg_.frame_transforms = frameTransforms;

        // Publish the messages
        anymal_msg_pub.publish(anymal_msg_);

        if (anymal_msg_throttlePub.getNumSubscribers() > 0 && throttleCounterANYmalMsg % throttleDecimationANYmalMsg == 0) {
            anymal_msg_throttlePub.publish(anymal_msg_);
            throttleCounterANYmalMsg = 0;
        } else {
            throttleCounterANYmalMsg++;
        }

        seqCounterANYmalMsg++;
    }

    int m_memberInt;
};

// Define the pybind11 module using the same name specified in premake5.lua
PYBIND11_MODULE(_anymal_pybind_bindings, m)
{
    using namespace anymal::ros1_cpp::pub_node;

    m.doc() = "pybind11 anymal.ros1_cpp.pub_node bindings";

    py::class_<PythonANYmalBound, carb::ObjectPtr<PythonANYmalBound>>(m, "ANYmalPublisher")
        .def(py::init([](const char* id) { return PythonANYmalBound::create(id); }),
             R"(
             Create a bound object.

             Args:
                 id: Id of the bound object.

             Return:
                 The bound object that was created.
             )",
             py::arg("id"))
        .def_readwrite("property_int", &PythonANYmalBound::m_memberInt,
             R"(
             Int property bound directly.
             )")
        .def("publish_twist", &PythonANYmalBound::publishTwist,
             R"(
             Bound fuction that returns a value.

             Return:
                 The toggled bool value.
             )")
        .def_static("initialize_ros", [](py::args args) {
            // Unpack Python arguments to obtain argc and argv
            int argc = args.size();
            char** argv = new char*[argc];
            for (int i = 0; i < argc; ++i) {
                argv[i] = strdup(args[i].cast<std::string>().c_str());
            }

            // Call the actual initializeROS function
            PythonANYmalBound::initializeROS(argc, argv);

            // Clean up allocated memory
            for (int i = 0; i < argc; ++i) {
                free(argv[i]);
            }
            delete[] argv;
            },
            R"(
            Bound function to initialize ROS.
            )")
        .def("publish_anymal_msg", &PythonANYmalBound::publishANYmalMsg, "Publish the message",
            py::arg("time"),
            py::arg("jointNames"),
            py::arg("jointPositions"),
            py::arg("jointVelocities"),
            py::arg("jointAccelerations"),
            py::arg("jointEfforts"),
            py::arg("contactBodyNames"),
            py::arg("contactForces"),
            py::arg("contactPositions"),
            py::arg("contactNormals"),
            py::arg("bodyPositions"),
            py::arg("quaternion"),
            py::arg("linearVelocities"),
            py::arg("angularVelocities"),
            py::arg("frameTransformChildFrames"),
            R"(
            Bound function to initialize ROS.
            )");
    /**/;

    // Explicitly link against ROS libraries
    m.attr("__ros_libs__") = py::str("-lroscpp -lrosconsole -lrosconsole_bridge");

}
}
