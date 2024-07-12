#ifndef NURI_HUMBLE_HARDWARE_INTERFACE_HPP
#define NURI_HUMBLE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/visibility_control.h"
#include "nurirobot_msgs/msg/nurirobot_speed.hpp"
#include "nurirobot_msgs/msg/nurirobot_pos.hpp"
#include "nurirobot_msgs/msg/hc_control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include <functional>
#include <memory>

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <unistd.h>
#include <stdexcept>

#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <thread>


#include "protocol.hpp"

namespace nuri_humble_hwif
{
class NuriSystemHardwareInterface: public hardware_interface::SystemInterface
{
    public:
        // NuriSystemHardwareInterface()
        // : node_(std::make_shared<rclcpp::Node>("my_hardware_interface"))
        // {
        //     RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "Initialize ======== node");
        //     // Initialize subscription and service
        //     // subscription_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
        //     //   "byte_array_topic", 10, std::bind(&NuriSystemHardwareInterface::byteArrayCallback, this, std::placeholders::_1));

        //     // Start a thread to spin the node

        // }

        // ~NuriSystemHardwareInterface()
        // {
        //     rclcpp::shutdown();
        //     if (spin_thread_.joinable()) {
        //     spin_thread_.join();
        //     }
        // }

        RCLCPP_SHARED_PTR_DEFINITIONS(NuriSystemHardwareInterface)

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        //
        void commandRemoteStop();
        void commandRemoteStart();
        void feedbackHCCall();
        void protocol_recv(uint8_t byte);
        void readHW();

        void feedbackCall(uint8_t id);
        void setRemote_callback(std_msgs::msg::Bool::UniquePtr msg);
        // void byteMultiArrayCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

    private:
        // LibSerial::SerialPort ser_;

        // int32_t l_last_enc_;
        // int32_t r_last_enc_;
        uint32_t l_last_enc_;
        uint32_t r_last_enc_;

        //GPIO Interfaces
        double enable_motor_cmd_;
        double enable_motor_state_;
        double estop_button_state_;
        double system_voltage_;
        double charging_voltage_;
        double user_power_current1_;
        double user_power_current2_;
        double current_temperature_;
        double fault_flags_;

        double is_estop_processed_;
        double is_enable_motor_processed_;

        // std::vector<int32_t> last_encoder_value_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;


        std::vector<uint16_t> last_pos_value_;
        std::vector<uint16_t> pos_value_;
        std::vector<uint16_t> current_value_;


        unsigned int msg_len = 0;
        uint8_t prev_byte = 0; // uint8_t is nice to store bytes
        uint16_t start_frame = 0;
        uint8_t* p;
        FeedbackResponse msg;
        uint8_t u8CallbackCount = 0;
        uint8_t u8SpeedStep = 1;
        uint8_t u8RecvCount[3];

        uint8_t u8ArrivalSec = 1;
        bool remote = true;
        bool firstrun = true;

        bool recvFeedback[3] = {false, false, false};
        bool recvLastPos[2] = {false, false};

        rclcpp::Clock clock_;
        int port_fd;

        std::string toHexString(const void *data, size_t size)
        {
            const uint8_t *byteData = static_cast<const uint8_t *>(data);
            std::stringstream ss;
            ss << std::hex << std::setfill('0');
            for (size_t i = 0; i < size; ++i)
            {
                ss << std::setw(2) << static_cast<int>(byteData[i]) << " ";
            }
            return ss.str();
        }

        float convertRange(float input)
        {
            return 2.0 * (input - 512.0) / 1023.0;
        }

        int calculate_angle_difference(uint16_t prev_angle, uint16_t current_angle)
        {
            int diff = current_angle - prev_angle;

            if (diff > 32766)
            { // 반대 방향으로 큰 변화가 감지된 경우 (0에서 65533로의 점프)
                diff -= 65533;
            }
            else if (diff < -32766)
            { // 반대 방향으로 큰 변화가 감지된 경우 (65533에서 0으로의 점프)
                diff += 65533;
            }

            return diff;
        }

        rclcpp::Node::SharedPtr node_;
        // rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
        // rclcpp::Service<your_package::srv::ByteArrayService>::SharedPtr service_;
        rclcpp::Publisher<nurirobot_msgs::msg::HCControl>::SharedPtr hc_ctrl_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr hc_joy_pub_;

        // rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr rawdata_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remote_sub_;

        std::thread spin_thread_;
};
} // namespace nuri_humble_hwif

#endif //NURI_HUMBLE_HARDWARE_INTERFACE_HPP

