#ifndef __ROBOT_SYSTEM_HPP__
#define __ROBOT_SYSTEM_HPP__

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace TODO_SYSTEM_PLUG
{
    struct IMUData
    {
        double orientation_x;
        double orientation_y;
        double orientation_z;
        double orientation_w;

        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;

        double linear_acceleration_x;
        double linear_acceleration_y;
        double linear_acceleration_z;
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotSystem : public hardware_interface::SystemInterface
    {
    public:
        /*
         * =================
         * Lifecycle methods
         * =================
         */
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        /*
         * =======================
         * Switching command modes
         * =======================
         */
        hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;
        hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

        /*
         * ==================
         * Read/Write methods
         * ==================
         */
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /*
         * ========================================
         * Exported state/command interface methods
         * ========================================
         */
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        /*
         * ==========================================
         * Exported unlisted state/command interfaces
         * ==========================================
         */
        std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override;
        std::vector<hardware_interface::InterfaceDescription> export_unlisted_command_interface_descriptions() override;

        /*
         * ===========
         * Enum: FLAGS
         * ===========
         */
        enum class FLAG : uint8_t
        {
            // System flags (0x0?)
            NONE = 0x00,
            NOT_INITIALIZED = 0x01,
            INITIALIZED = 0x02,
            CONFIGURED = 0x03,
            ACTIVATED = 0x04,
            DEACTIVATED = 0x05,
            CLEANED = 0x06,

            // Error flags (0x1?)
            ERROR = 0x10,
            FAULT = 0x11,
            EMERGENCY = 0x12,

            // Calibration flags (0x2?)
            UNCALIBRATED = 0x20,
            CALIBRATED = 0x21
        };

        inline FLAG get_flag() const { return flag_; }
        inline void clear_flag() { flag_ = FLAG::NONE; }

    private:
        /*
         * ======================
         * TODO: State interfaces
         * ======================
         */

        // Joint states
        std::unordered_map<std::string, double> joint_state_positions_;
        std::unordered_map<std::string, double> joint_state_velocities_;
        std::unordered_map<std::string, double> joint_state_efforts_;

        // Sensor states
        std::unordered_map<std::string, IMUData> sensor_state_imus_;

        /*
         * ========================
         * TODO: Command interfaces
         * ========================
         */

        // Joint commands
        std::unordered_map<std::string, double> joint_commands_positions_;
        std::unordered_map<std::string, double> joint_commands_velocities_;
        std::unordered_map<std::string, double> joint_commands_efforts_;

        /*
         * ===========
         * TODO: Flags
         * ===========
         */
        FLAG flag_;
    };

} // namespace TODO_SYSTEM_PLUG

#endif // __ROBOT_SYSTEM_HPP__