#include "system_plug/RobotSystem.hpp"
#include <random>

namespace TODO_SYSTEM_PLUG
{
    /*
     * =================
     * Lifecycle methods
     * =================
     */

    CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Failed to initialize hardware interface");
            flag_ = FLAG::ERROR;
            return CallbackReturn::ERROR;
        }

        /*
         * ________________________________________________________________________________________________
         * TODO: Initialize hardware commmunication
         *       - e.g. Initialize ports, communication protocols, etc.
         *       - e.g. Initialize drivers, internal variables(state interfaces, command interfaces), etc.
         * ________________________________________________________________________________________________
         *
         */

        // Initialize joints states and commands
        for (const auto &joint : info_.joints)
        {
            joint_state_positions_[joint.name] = 0.0;
            joint_state_velocities_[joint.name] = 0.0;
            joint_state_efforts_[joint.name] = 0.0;

            joint_commands_positions_[joint.name] = 0.0;
            joint_commands_velocities_[joint.name] = 0.0;
            joint_commands_efforts_[joint.name] = 0.0;
        }

        // Initialize sensor states (IMUs)
        for (const auto &sensor : info_.sensors)
            if (sensor.name.find("IMU") != std::string::npos)
            {
                sensor_state_imus_[sensor.name] = IMUData();
                RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initialized IMU sensor: %s", sensor.name.c_str());
            }

        flag_ = FLAG::INITIALIZED;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Initialized hardware interface successfully");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    {
        if (flag_ != FLAG::INITIALIZED)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Failed to configure hardware interface");
            flag_ = FLAG::ERROR;
            return CallbackReturn::ERROR;
        }

        /*
         * _________________________________________________________________________________
         * TODO: Configure hardware
         *       - e.g. Configure ports, communication protocols, etc. (if not already done)
         *       - e.g. Configure drivers, limits, etc.
         * _________________________________________________________________________________
         *
         */

        flag_ = FLAG::CONFIGURED;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Configured hardware interface successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activating hardware interface");
        if (flag_ != FLAG::CONFIGURED)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Failed to activate hardware interface");
            flag_ = FLAG::ERROR;
            return CallbackReturn::ERROR;
        }

        /*
         * __________________________________________________________________
         * TODO: Activate hardware
         *       - e.g. Start communication, start operation of drivers, etc.
         *       - e.g. Enable motors, sensors, etc.
         * __________________________________________________________________
         *
         */

        flag_ = FLAG::ACTIVATED;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activated hardware interface successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Deactivating hardware interface");
        if (flag_ != FLAG::ACTIVATED)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Failed to deactivate hardware interface");
            flag_ = FLAG::ERROR;
            return CallbackReturn::ERROR;
        }

        /*
         * ________________________________________________________________
         * TODO: Deactivate hardware
         *       - e.g. Stop communication, stop operation of drivers, etc.
         *       - e.g. Disable motors, sensors, etc.
         * ________________________________________________________________
         *
         */

        flag_ = FLAG::DEACTIVATED;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Deactivated hardware interface successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Cleaning up hardware interface");
        if (flag_ != FLAG::DEACTIVATED)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Failed to cleanup hardware interface");
            flag_ = FLAG::ERROR;
            return CallbackReturn::ERROR;
        }

        /*
         * _______________________________________________________
         * TODO: Cleanup hardware
         *       - e.g. Close ports, communication protocols, etc.
         *       - e.g. Cleanup drivers, internal variables, etc.
         *       - e.g. Delete objects, free memory, etc.
         * _______________________________________________________
         *
         */

        flag_ = FLAG::CLEANED;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Cleaned up hardware interface successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Shutting down hardware interface");

        /*
         * _______________________________________________________
         * TODO: Shutdown hardware
         *       - e.g. Perform any final cleanup or safety checks
         *       - e.g. Perform any necessary shutdown operations
         *       - e.g. Ensure all hardware is in a safe state
         * _______________________________________________________
         *
         */

        flag_ = FLAG::NONE;
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Shutdown hardware interface successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RobotSystem::on_error(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Error occurred in hardware interface");

        /*
         * ___________________________________________________
         * TODO: Handle error state
         *       - e.g. Perform emergency stop, shutdown, etc.
         *       - e.g. Put hardware in a safe state
         *       - e.g. Log detailed error information
         * ___________________________________________________
         *
         */

        flag_ = FLAG::ERROR;
        RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), "Handled error in hardware interface");
        return CallbackReturn::SUCCESS;
    }

    /*
     * =======================
     * Switching command modes
     * =======================
     */

    hardware_interface::return_type RobotSystem::prepare_command_mode_switch(const std::vector<std::string> & /* start_interfaces */, const std::vector<std::string> & /* stop_interfaces */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Preparing command mode switch");

        /*
         * _________________________________________________________
         * TODO: Prepare command mode switch
         *       - e.g. Check if requested interfaces are supported
         *       - e.g  Prepare hardware for mode switch
         * _________________________________________________________
         *
         */

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotSystem::perform_command_mode_switch(const std::vector<std::string> & /* start_interfaces */, const std::vector<std::string> & /* stop_interfaces */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Performing command mode switch");

        /*
         * _______________________________________________________________
         * TODO: Perform command mode switch
         *       - e.g. Change control mode in hardware
         *       - e.g. Update internal state to reflect new command mode
         * _______________________________________________________________
         *
         */

        return hardware_interface::return_type::OK;
    }

    /*
     * ==================
     * Read/Write methods
     * ==================
     */

    hardware_interface::return_type RobotSystem::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Reading ...");

        /*
         * ______________________________________________________________
         * TODO: Read joint states, sensor states, etc. physical hardware
         *       - e.g. Read joint positions, velocities, efforts, etc.
         *       - e.g. Read sensor data (IMU, tcp force/torque, etc.)
         * ______________________________________________________________
         *
         */

        for (const auto &joint : info_.joints)
        {
            /*
             * _________________________________________________________________________________
             * TODO: Read joint states from physical hardware
             *       - e.g. Read joint positions, velocities, efforts, etc.
             *       - e.g. Update internal joint state variables
             * _________________________________________________________________________________
             *
             * Example:
             *         ...
             *         joint_state_positions_[joint.name] = motor_driver->read_position(motor_?)
             *         joint_state_velocities_[joint.name] = motor_driver->read_velocity(motor_?)
             *         joint_state_efforts_[joint.name] = motor_driver->read_effort(motor_?)
             *         ...
             * _________________________________________________________________________________
             *
             */

            joint_state_positions_[joint.name] = joint_commands_positions_[joint.name];
            joint_state_velocities_[joint.name] = joint_commands_velocities_[joint.name];
            joint_state_efforts_[joint.name] = joint_commands_efforts_[joint.name];

            RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                         "Read joint state: %s, position: %.2f, velocity: %.2f, effort: %.2f",
                         joint.name.c_str(),
                         joint_state_positions_[joint.name],
                         joint_state_velocities_[joint.name],
                         joint_state_efforts_[joint.name]);
        }

        // # Mock data
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> orientation_dist(-M_PI, M_PI);
        static std::uniform_real_distribution<> angular_velocity_dist(-10.0, 10.0);
        static std::uniform_real_distribution<> linear_acceleration_dist(-9.81, 9.81);
        // #

        for (const auto &sensor : info_.sensors)
            if (sensor.name.find("IMU") != std::string::npos)
            {
                /*
                 * _________________________________________________________________________________
                 * TODO: Read sensor states from physical hardware
                 *       - e.g. Read sensor data (IMU, tcp force/torque, etc.)
                 *       - e.g. Update internal sensor state variables
                 * _________________________________________________________________________________
                 *
                 * Example:
                 *         ...
                 *         sensor_state_imus_[sensor.name] = imu_driver->read(sensor_?)
                 *         ...
                 * _________________________________________________________________________________
                 *
                 */

                // # Mock data
                double qx = orientation_dist(gen);
                double qy = orientation_dist(gen);
                double qz = orientation_dist(gen);
                double qw = orientation_dist(gen);

                double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
                sensor_state_imus_[sensor.name].orientation_x = qx / norm;
                sensor_state_imus_[sensor.name].orientation_y = qy / norm;
                sensor_state_imus_[sensor.name].orientation_z = qz / norm;
                sensor_state_imus_[sensor.name].orientation_w = qw / norm;

                sensor_state_imus_[sensor.name].angular_velocity_x = angular_velocity_dist(gen);
                sensor_state_imus_[sensor.name].angular_velocity_y = angular_velocity_dist(gen);
                sensor_state_imus_[sensor.name].angular_velocity_z = angular_velocity_dist(gen);

                sensor_state_imus_[sensor.name].linear_acceleration_x = linear_acceleration_dist(gen);
                sensor_state_imus_[sensor.name].linear_acceleration_y = linear_acceleration_dist(gen);
                sensor_state_imus_[sensor.name].linear_acceleration_z = linear_acceleration_dist(gen);
                // #

                RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                             "Read sensor state: %s, orientation: %.2f, %.2f, %.2f, angular_velocity: %.2f, %.2f, %.2f, linear_acceleration: %.2f, %.2f, %.2f",
                             sensor.name.c_str(),
                             sensor_state_imus_[sensor.name].orientation_x,
                             sensor_state_imus_[sensor.name].orientation_y,
                             sensor_state_imus_[sensor.name].orientation_z,
                             sensor_state_imus_[sensor.name].angular_velocity_x,
                             sensor_state_imus_[sensor.name].angular_velocity_y,
                             sensor_state_imus_[sensor.name].angular_velocity_z,
                             sensor_state_imus_[sensor.name].linear_acceleration_x,
                             sensor_state_imus_[sensor.name].linear_acceleration_y,
                             sensor_state_imus_[sensor.name].linear_acceleration_z);
            }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Writing ...");

        /*
         * ___________________________________________________________________________________________
         * TODO: Write joint commands to physical hardware
         *       - e.g. Write joint positions, velocities, efforts, etc.
         * ___________________________________________________________________________________________
         *
         * Example:
         *         ...
         *         motor_driver->write_position(joint_commands_positions_[joint.name])
         *         motor_driver->write_velocity(joint_commands_velocities_[joint.name])
         *         motor_driver->write_effort(joint_commands_efforts_[joint.name])
         *         ...
         * ___________________________________________________________________________________________
         *
         */

        for (const auto &joint : info_.joints)
        {
            joint_commands_positions_[joint.name] = joint_state_positions_[joint.name];
            joint_commands_velocities_[joint.name] = joint_state_velocities_[joint.name];
            joint_commands_efforts_[joint.name] = joint_state_efforts_[joint.name];

            RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                         "Write joint command: %s, position: %.2f, velocity: %.2f, effort: %.2f",
                         joint.name.c_str(),
                         joint_commands_positions_[joint.name],
                         joint_commands_velocities_[joint.name],
                         joint_commands_efforts_[joint.name]);
        }

        return hardware_interface::return_type::OK;
    }

    /*
     * ========================================
     * Exported state/command interface methods
     * ========================================
     */

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> RobotSystem::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

        /*
         * _______________________________________________________________________________
         * TODO: Export state interfaces
         *       - e.g. Export joint states, sensor states and other states interfaces
         *       - e.g. Export mock state interfaces for testing (if no physical hardware)
         * _______________________________________________________________________________
         *
         */

        for (const auto &joint : info_.joints)
        {
            state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                joint.name, hardware_interface::HW_IF_POSITION, &joint_state_positions_[joint.name]));
            state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                joint.name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocities_[joint.name]));
            state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                joint.name, hardware_interface::HW_IF_EFFORT, &joint_state_efforts_[joint.name]));

            RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                         "Exported joint state interface: %s, position, velocity, effort",
                         joint.name.c_str());
        }

        for (const auto &sensor : info_.sensors)
            if (sensor.name.find("IMU") != std::string::npos)
            {
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "orientation_x", &sensor_state_imus_[sensor.name].orientation_x));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "orientation_y", &sensor_state_imus_[sensor.name].orientation_y));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "orientation_z", &sensor_state_imus_[sensor.name].orientation_z));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "orientation_w", &sensor_state_imus_[sensor.name].orientation_w));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "angular_velocity_x", &sensor_state_imus_[sensor.name].angular_velocity_x));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "angular_velocity_y", &sensor_state_imus_[sensor.name].angular_velocity_y));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "angular_velocity_z", &sensor_state_imus_[sensor.name].angular_velocity_z));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "linear_acceleration_x", &sensor_state_imus_[sensor.name].linear_acceleration_x));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "linear_acceleration_y", &sensor_state_imus_[sensor.name].linear_acceleration_y));
                state_interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
                    sensor.name, "linear_acceleration_z", &sensor_state_imus_[sensor.name].linear_acceleration_z));

                RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                             "Exported IMU sensor state interface: %s, orientation, angular_velocity, linear_acceleration",
                             sensor.name.c_str());
            }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> RobotSystem::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

        /*
         * _________________________________________________________________________________
         * TODO: Export command interfaces
         *       - e.g. Export joint commands and other command interfaces
         *       - e.g. Export mock command interfaces for testing (if no physical hardware)
         * _________________________________________________________________________________
         *
         */
        for (const auto &joint : info_.joints)
        {
            command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
                joint.name, hardware_interface::HW_IF_POSITION, &joint_commands_positions_[joint.name]));
            command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
                joint.name, hardware_interface::HW_IF_VELOCITY, &joint_commands_velocities_[joint.name]));
            command_interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
                joint.name, hardware_interface::HW_IF_EFFORT, &joint_commands_efforts_[joint.name]));

            RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"),
                         "Exported joint command interface: %s, position, velocity, effort",
                         joint.name.c_str());
        }

        return command_interfaces;
    }

    /*
     * ==========================================
     * Exported unlisted state/command interfaces
     * ==========================================
     */

    std::vector<hardware_interface::InterfaceDescription> RobotSystem::export_unlisted_state_interface_descriptions()
    {
        std::vector<hardware_interface::InterfaceDescription> state_interface_descriptions;

        /*
         * ______________________________________________________
         * TODO: Export unlisted state interface descriptions
         *       - e.g. Export unlisted states interfaces in URDF
         * ______________________________________________________
         *
         */
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Exporting unlisted state interface descriptions");
        return state_interface_descriptions;
    }

    std::vector<hardware_interface::InterfaceDescription> RobotSystem::export_unlisted_command_interface_descriptions()
    {
        std::vector<hardware_interface::InterfaceDescription> command_interface_descriptions;

        /*
         * _______________________________________________________
         * TODO: Export unlisted command interface descriptions
         *       - e.g. Export unlisted command interfaces in URDF
         * _______________________________________________________
         *
         */
        RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Exporting unlisted command interface descriptions");
        return command_interface_descriptions;
    }

} // namespace TODO_SYSTEM_PLUG

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(TODO_SYSTEM_PLUG::RobotSystem, hardware_interface::SystemInterface)