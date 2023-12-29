-- Setup the basic extension information.
local ext = get_current_extension_info()
project_ext(ext)

-- --------------------------------------------------------------------------------------------------------------
-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
}


-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext)
    -- It is important that you add all subdirectories containing C++ code to this project
    add_files("source", "plugins/anymal.ros1_cpp.pub_node")

    includedirs {
        -- System level ROS includes
        "%{root}/_build/target-deps/system_ros/include",

        "%{root}/_build/target-deps/system_ros/include/roscpp",

        "%{root}/_build/target-deps/system_ros/include/rosconsole",

        "%{root}/_build/target-deps/system_ros/include/rosconsole_bridge",

        "%{root}/_build/target-deps/system_ros/include/std_msgs",

        "%{root}/_build/target-deps/system_ros/include/geometry_msgs",

        "%{root}/_build/target-deps/system_ros/include/anymal_msgs",

        -- System level boost
        "%{root}/_build/target-deps/system_boost",

        -- Include the extension's include directory
        "include",
        "plugins/anymal.ros1_cpp.pub_node"

    }

    libdirs {
        -- System level ROS libraries
        "%{root}/_build/target-deps/system_ros/lib",
    }

    links{
        "roscpp", "rosconsole", "rosconsole_bridge", "std_msgs", "geometry_msgs", "anymal_msgs"
    }

    filter { "system:linux" }
        linkoptions { "-Wl,--export-dynamic" }


-- Build Python bindings that will be loaded by the extension.
project_ext_bindings {
    ext = ext,
    project_name = "anymal.ros1_cpp.pub_node",
    module = "_anymal_pybind_bindings",
    src = "bindings/python/anymal.ros1_cpp.pub_node",
    target_subdir = "anymal/ros1_cpp/pub_node"
}
    repo_build.prebuild_link {
        { "python/impl", ext.target_dir.."/anymal/ros1_cpp/pub_node/impl" },
    }
    includedirs {
        -- System level ROS includes
        "%{root}/_build/target-deps/system_ros/include",

        "%{root}/_build/target-deps/system_ros/include/roscpp",

        "%{root}/_build/target-deps/system_ros/include/rosconsole",

        "%{root}/_build/target-deps/system_ros/include/rosconsole_bridge",

        "%{root}/_build/target-deps/system_ros/include/std_msgs",

        "%{root}/_build/target-deps/system_ros/include/geometry_msgs",

        "%{root}/_build/target-deps/system_ros/include/anymal_msgs",

        -- System level boost
        "%{root}/_build/target-deps/system_boost",

        -- Include the extension's include directory
        "include",
    }

    libdirs {
        -- System level ROS libraries
        "%{root}/_build/target-deps/system_ros/lib",
    }

    links{
        "roscpp", "rosconsole", "rosconsole_bridge", "std_msgs", "geometry_msgs", "anymal_msgs"
    }

    filter { "system:linux" }
        linkoptions { "-Wl,--export-dynamic" }
