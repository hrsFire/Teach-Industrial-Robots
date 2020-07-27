#include "interbotix_helper.hpp"

using namespace interbotix;

bool InterbotixHelper::GetRobotInfoFromConfigFiles(std::string robot_name, interbotix_sdk::RobotInfo::Response& res) {
    std::vector<double> home_positions;
    std::vector<double> sleep_positions;
    std::vector<Motor> all_joints;
    size_t joint_num_write;
    GetMotorConfigs(robot_name, home_positions, sleep_positions, all_joints, joint_num_write);

    // Parse the urdf model to get joint position and velocity limits
    urdf::Model model;
    model.initParam(robot_name + "/robot_description");
    for (auto const& joint:all_joints)
    {
        urdf::JointConstSharedPtr ptr;
        if (joint.name == "gripper")
        {
            ptr = model.getJoint("left_finger");
            res.lower_gripper_limit = ptr->limits->lower;
            res.upper_gripper_limit = ptr->limits->upper;
            res.use_gripper = true;
        }
        ptr = model.getJoint(joint.name);
        res.lower_joint_limits.push_back(ptr->limits->lower);
        res.upper_joint_limits.push_back(ptr->limits->upper);
        res.velocity_limits.push_back(ptr->limits->velocity);
        res.joint_names.push_back(joint.name);
        res.joint_ids.push_back(joint.motor_id);
    }

    res.home_pos = home_positions;
    res.sleep_pos = sleep_positions;
    res.num_joints = joint_num_write;
    res.num_single_joints = all_joints.size();
    return true;
}

void InterbotixHelper::GetMotorConfigs(std::string robot_name, std::vector<double>& home_positions, std::vector<double>& sleep_positions,
        std::vector<Motor>& all_joints, size_t& joint_num_write) {
    std::string interbotixSDKPath = ros::package::getPath("interbotix_sdk");
    std::string yaml_file = interbotixSDKPath + "/config/" + robot_name + ".yaml";
    YAML::Node dxl_config = YAML::LoadFile(yaml_file.c_str());
    if (dxl_config.IsNull())
    {
        ROS_ERROR("Config file not found.");
        return;
    }

    // Define the home and sleep positions for the arm joints
    YAML::Node sleep_node = dxl_config["sleep"];
    for (auto const& value: sleep_node)
    {
        sleep_positions.push_back(value.as<double>());
        home_positions.push_back(0);
    }

    // Get the actual motor configs
    YAML::Node order_node = dxl_config["order"];
    YAML::Node singles_node = dxl_config["singles"];
    std::vector <YAML::Node> nodes {order_node, singles_node};
    joint_num_write = 0;

    // Read in each node in the yaml file according to the 'order' sequence. This will make it easier
    // to keep track of where each motor is located in an array. This first loop is mainly used to get
    // a vector of all the motors and their corresponding Dynamixel IDs, as well as determine the number
    // of joints that need to be commanded.
    for (auto const& node: nodes)
    {
        for(size_t i{0}; i < node.size(); i++)
        {
            std::string name = node[i].as<std::string>();
            YAML::Node item = dxl_config[name];
            int32_t id = item["ID"].as<int32_t>();
            int32_t secondary_id = item["Secondary_ID"].as<int32_t>();
            Motor motor = {name, (uint8_t) id};
            std::vector<uint8_t> shadow_list {(uint8_t) id};
            // Determine if a motor is a shadow of another by checking its secondary 'shadow' ID.
            // Only increment 'joint_num_write' if the motor's secondary_id register is disabled (set to 255)
            if (node == dxl_config["order"] && name != "gripper" && secondary_id == 255)
                joint_num_write++;
        }
    }

    // Now that the number of joints that need to be commanded has been figured out, create an array of
    // that size to hold the motor ids. Unfortunately, vectors cannot be used since the Robotis provided function
    // that actually commands the motors ('syncWrite') does not take them as input.
    size_t cntr = 0;

    // Check to see if the Drive_Mode register should be set to 'Time Based Profile' instead of 'Velocity Based Profile'
    bool use_time_based_profile;
    ros::param::get("~use_time_based_profile", use_time_based_profile);

    // For each node in the yaml file, read in the register names and the values that should be written to them.
    for (auto const& node: nodes)
    {
        for (size_t i{0}; i < node.size(); i++)
        {
            std::string name = node[i].as<std::string>();
            YAML::Node item = dxl_config[name];
            int32_t id = item["ID"].as<int32_t>();
            for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
            {
                std::string item_name = it_item->first.as<std::string>();
                int32_t value = it_item->second.as<int32_t>();

                if (item_name == "ID")
                    continue;
                else if (item_name == "Secondary_ID" && value == 255)
                {
                    Motor joint = {name, (uint8_t) id};
                    JointMode jm = {(uint8_t) id, State::NONE};
                    all_joints.push_back(joint);

                    if (node == dxl_config["order"])
                    {
                        if (name != "gripper")
                        {
                            cntr++;
                        }
                    }
                }
                else if (item_name == "Secondary_ID" && value != 255)
                    ;
                else if (item_name == "Drive_Mode" && use_time_based_profile == true)
                    value += 4;
                Info info = {(uint8_t) id, item_name, value};
            }
        }
    }
}

#ifdef COMMUNICATION_MEASUREMENT
std::ofstream* InterbotixHelper::InitializeMeasurementFile(const std::string& filePath) {
    std::ofstream* measurementFile = new std::ofstream(filePath, std::ios::out | std::ios::trunc);
    *measurementFile << "Time [TT.MM.JJJJ HH:MM:SS.zzz]" << std::endl;
    return measurementFile;
}

void InterbotixHelper::SaveCommunicationMeasurement(const std::chrono::system_clock::time_point& timePoint, std::ofstream& measurementFile) {
    auto tt = std::chrono::system_clock::to_time_t(timePoint);
    std::tm* ttm = localtime(&tt);
    char buffer[26];
    strftime(buffer, 26, "%d.%m.%Y %H:%M:%S", ttm);
    std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count() % 1000;
    char fractionBuffer[4];
    sprintf(fractionBuffer, "%03d", milliseconds);
    std::string timeString = std::string(buffer) + "." + std::string(fractionBuffer);

    measurementFile << timeString << std::endl;
    measurementFile.flush();
}
#endif //COMMUNICATION_MEASUREMENT
