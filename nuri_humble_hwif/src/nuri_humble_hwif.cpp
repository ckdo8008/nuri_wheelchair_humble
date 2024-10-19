#include "nuri_humble_hwif/nuri_humble_hwif.hpp"

#include <math.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nuri_humble_hwif
{
hardware_interface::CallbackReturn NuriSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    // Get info parameters from URDF
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Name: %s", info_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Number of Joints %zu", info_.joints.size());

    // Initialize hardware_interface
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // last_pos_value_.resize(2, std::numeric_limits<uint16_t>::quiet_NaN());
    // pos_value_.resize(2, std::numeric_limits<uint16_t>::quiet_NaN());
    // current_value_.resize(2, std::numeric_limits<uint16_t>::quiet_NaN());
    last_pos_value_.resize(2, 0);
    pos_value_.resize(2, 0);
    current_value_.resize(2, 0);

    // Check the URDF Data
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),  joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("NuriSystemHardwareInterface"),
                "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    l_last_enc_ = 0;
    r_last_enc_ = 0;

    enable_motor_cmd_ = 1.0;
    enable_motor_state_= 0.0;
    estop_button_state_= 0.0;
    system_voltage_= 0.0;
    charging_voltage_= 0.0;
    user_power_current1_= 0.0;
    user_power_current2_= 0.0;
    current_temperature_= 0.0;
    fault_flags_= 0.0;

    is_estop_processed_ = estop_button_state_;

    auto port_name = info_.hardware_parameters["port_name"];
    auto baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Get port_name [%s] and baudrate [%d]", port_name.c_str(), baudrate);

    if ((port_fd = open(port_name.c_str(), O_RDWR | O_NDELAY)) < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Cannot open serial port to Nurirobot");
        exit(-1); // TODO : put this again
    }

    struct termios2 tty;
    if (ioctl(port_fd, TCGETS2, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error from tcgetattr");
        exit(-1);
    }

    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~PARENB;
    tty.c_cflag		&=	~PARODD;
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CRTSCTS;
    tty.c_cflag     |=  CREAD | CLOCAL;

    tty.c_cflag     &=  ~CBAUD;
    tty.c_cflag     |=  CBAUDEX;
    tty.c_ispeed    =   38400;
    tty.c_ospeed    =   38400;
    tty.c_oflag     =   0;
    tty.c_oflag     &=  ~OPOST;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;    
    tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);
    tty.c_iflag 	&=  ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_lflag		&=  ~ICANON;
    tty.c_lflag     &=  ~(ECHO);
    tty.c_lflag     &=  ~ECHOE;     // Turn off echo erase (echo erase only relevant if canonical input is active)
    tty.c_lflag     &=  ~ECHONL;    //
    tty.c_lflag     &=  ~ISIG;      // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

    if (ioctl(port_fd, TCSETS2, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error from tcsetattr");
        exit(-1);
    }

    node_ = std::make_shared<rclcpp::Node>("my_hardware_interface");

    hc_ctrl_pub_ = node_->create_publisher<nurirobot_msgs::msg::HCControl>("hc/control", 10);
    hc_joy_pub_ = node_->create_publisher<sensor_msgs::msg::Joy>("hc/joy", 10);
    spin_thread_ = std::thread([this]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node_);
        executor.spin();    
    });

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Opened serial port to Nurirobot");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    commandRemoteStop();
    commandRemoteStop();
    commandRemoteStop();
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Wait connection to Motor Driver");
    int cnt = 0;
    while(rclcpp::ok())
    {
        feedbackHCCall();
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        readHW();
        if (!firstrun) break;
        if (cnt++ > 5) break;
    }

    u8ArrivalSec = std::stof(info_.hardware_parameters["robot_acceleration"]);
    // auto robot_dec = std::stof(info_.hardware_parameters["robot_deceleration"]);
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Get robot_acceleration [%6.1f] sec", u8ArrivalSec / 10.0);
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Get robot_deceleration [%6.1f] RPM/s", robot_dec / 10.0);

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Successfully initialized!");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NuriSystemHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "motor_enabled", &enable_motor_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "estop_button_state", &estop_button_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "system_voltage", &system_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "charging_voltage", &charging_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "user_power_current1", &user_power_current1_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "user_power_current2", &user_power_current2_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "curent_temperature", &current_temperature_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "fault_flags", &fault_flags_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NuriSystemHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        if(info_.joints[i].command_interfaces[0].name == "velocity")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]
                )
            );
        }
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "set_enable_motor", &enable_motor_cmd_));

    return command_interfaces;
}

hardware_interface::CallbackReturn NuriSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // reset motor driver and initialize. And then, start motor driver
    commandRemoteStop();
    commandRemoteStop();
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
        if (std::isnan(hw_positions_[i]))
        {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_efforts_[i] = 0;
            hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "activated...");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NuriSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "on_deactivate");
    // stop motor driver
    commandRemoteStart();
    commandRemoteStart();
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type NuriSystemHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "read");
    std::lock_guard<std::mutex> lock(serial_mutex_);
    int cnt = 0;
    while(rclcpp::ok())
    {
        recvFeedback[2] = false;
        feedbackHCCall();
        rclcpp::sleep_for(std::chrono::milliseconds(20));

        readHW();
        if (recvFeedback[2]) break;
        if (cnt++ > 3) break;
    }

    cnt = 0;
    while(rclcpp::ok())
    {
        recvFeedback[0] = false;
        feedbackCall(0);
        rclcpp::sleep_for(std::chrono::milliseconds(20));

        readHW();
        if (recvFeedback[0]) break;
        if (cnt++ > 3) break;
    }

    cnt = 0;
    while(rclcpp::ok())
    {
        recvFeedback[1] = false;
        feedbackCall(1);
        rclcpp::sleep_for(std::chrono::milliseconds(20));

        readHW();
        if (recvFeedback[1]) break;
        if (cnt++ > 3) break;
    }    

    hw_positions_[0] += calculate_angle_difference(last_pos_value_[0], pos_value_[0]) / 800.0;
    hw_positions_[1] += calculate_angle_difference(last_pos_value_[1], pos_value_[1]) / 800.0;

    hw_velocities_[0] = 0.0;
    hw_velocities_[1] = 0.0;
    hw_efforts_[0] = current_value_[0];
    hw_efforts_[1] = current_value_[1];

    last_pos_value_[0] = pos_value_[0];
    last_pos_value_[1] = pos_value_[1];

    RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "position : [ %d, %d] pos :[ %f, %f], current :[ %f, %f]", pos_value_[0], pos_value_[1], hw_positions_[0], hw_positions_[1], hw_efforts_[0], hw_efforts_[1]);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type NuriSystemHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration & /*period*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "write");
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // int16_t l_rpm = hw_commands_[0] / (2.0 * M_PI) * 420;
    // int16_t r_rpm = hw_commands_[1] / (2.0 * M_PI) * 420;
    int16_t l_rpm = hw_commands_[0] / 90.0 * 420;
    int16_t r_rpm = hw_commands_[1] / 90.0 * 420;

    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "rpm =========  %f %f %d %d", hw_commands_[0], hw_commands_[1], l_rpm, r_rpm);


    if (l_rpm == 0.0 && r_rpm == 0.0) {
        std::vector<uint8_t> mutable_stoparray_left = {0xff, 0xfe, 0x00, 0x06, 0xf6, 0x02, 0x00, 0x00, 0x00, 0x01};
        std::vector<uint8_t> mutable_stoparray_right = {0xff, 0xfe, 0x01, 0x06, 0xf5, 0x02, 0x00, 0x00, 0x00, 0x01};
        int rc = ::write(port_fd, mutable_stoparray_left.data(), mutable_stoparray_left.size());
        if (rc < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NuriSystemHardwareInterface"), 
                "Error writing to Nurirobot serial port");
            return hardware_interface::return_type::ERROR;
        }

        rc = ::write(port_fd, mutable_stoparray_right.data(), mutable_stoparray_right.size());
        if (rc < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NuriSystemHardwareInterface"), 
                "Error writing to Nurirobot serial port");
            return hardware_interface::return_type::ERROR;
        }
    }
    else {
        std::vector<uint8_t> mutable_bytearray_left = {0xff, 0xfe, 0x00, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01};
        std::vector<uint8_t> mutable_bytearray_right = {0xff, 0xfe, 0x01, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01};

        uint8_t leftdir = (l_rpm > 0) ? 0x00 : 0x01;
        uint8_t rightdir = (r_rpm > 0) ? 0x00 : 0x01;
        // int absleftrpm = std::floor(std::abs(l_rpm) * 10);
        // int absrightrpm = std::floor(std::abs(r_rpm) * 10);

        int absleftrpm = 4800 + std::abs(l_rpm) * 10;
        int absrightrpm = 4800 + std::abs(r_rpm) * 10;

        mutable_bytearray_left[6] = leftdir;
        mutable_bytearray_right[6] = rightdir;

        mutable_bytearray_left[7] = (absleftrpm >> 8) & 0xFF;
        mutable_bytearray_left[8] = absleftrpm & 0xFF;
        mutable_bytearray_right[7] = (absrightrpm >> 8) & 0xFF;
        mutable_bytearray_right[8] = absrightrpm & 0xFF;

        int sum_val_l = 0;
        for (size_t i = 2; i < mutable_bytearray_left.size(); ++i)
        {
            sum_val_l += mutable_bytearray_left[i];
        }
        sum_val_l -= mutable_bytearray_left[4];

        int sum_val_r = 0;
        for (size_t i = 2; i < mutable_bytearray_right.size(); ++i)
        {
            sum_val_r += mutable_bytearray_right[i];
        }
        sum_val_r -= mutable_bytearray_right[4];

        uint8_t leftchecksum = ~sum_val_l & 0xFF;
        uint8_t rightchecksum = ~sum_val_r & 0xFF;

        mutable_bytearray_left[4] = leftchecksum;
        mutable_bytearray_right[4] = rightchecksum;

        int rc = ::write(port_fd, mutable_bytearray_left.data(), mutable_bytearray_left.size());
        if (rc < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NuriSystemHardwareInterface"), 
                "Error writing to Nurirobot serial port");
            return hardware_interface::return_type::ERROR;
        }

        rc = ::write(port_fd, mutable_bytearray_right.data(), mutable_bytearray_right.size());
        if (rc < 0)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("NuriSystemHardwareInterface"), 
                "Error writing to Nurirobot serial port");
            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}
}

void nuri_humble_hwif::NuriSystemHardwareInterface::commandRemoteStop()
{
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "commandRemoteStop");
    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb3;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error commandRemoteStop writing to Nurirobot serial port");
    }  
}

void nuri_humble_hwif::NuriSystemHardwareInterface::commandRemoteStart()
{
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "commandRemoteStart");
    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb2;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error commandRemoteStart writing to Nurirobot serial port");
    }  

}

void nuri_humble_hwif::NuriSystemHardwareInterface::feedbackHCCall()
{
    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = (uint8_t)0xc0;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xb4;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error writing to Nurirobot serial port");
    }    
}

void nuri_humble_hwif::NuriSystemHardwareInterface::protocol_recv(uint8_t byte)
{
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
    // Read the start frame
    if (start_frame == START_FRAME)
    {
        p = (uint8_t *)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    }
    else if (msg_len >= 2 && msg_len < sizeof(FeedbackResponse))
    {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len > 4)
    {
        if ((unsigned int)(msg.datasize + 4) == msg_len)
        {
            p = (uint8_t *)&msg;
            uint8_t checksum = 0;
            size_t msg_size = (size_t)msg_len;

            u8RecvCount[0] = u8RecvCount[0] + (p[2] == 0 ? 1: 0);            
            u8RecvCount[1] = u8RecvCount[1] + (p[2] == 1 ? 1: 0);
            u8RecvCount[2] = u8RecvCount[2] + (p[2] == 192 ? 1: 0);

            for (size_t i = 2; i < msg_size; i++)
            {
                checksum += p[i];
            }
            checksum -= p[4];
            checksum = ~checksum;

            // 체크섬 확인
            if (checksum == p[4])
            {
                switch (msg.mode)
                {
                case 0x02:
                {
                    // 속도 제어 신호로 처리하면 피드백 성능이 떨어짐
                    // 제어 상태에 따라서 모드 변경이 필요
                    if (remote) {
                        commandRemoteStart();
                    }
                    break;
                }
                case 0x03:
                {
                    // 속도 제어 신호로 처리하면 피드백 성능이 떨어짐
                    // 제어 상태에 따라서 모드 변경이 필요
                    if (remote) {
                        commandRemoteStart();
                    }
                    break;
                }
                case 0xb1:
                {
                    // 휠체어 상태 피드백
                    uint8_t *src = (uint8_t *)&msg;
                    WheelchairResponse tmp;
                    uint8_t *dest = (uint8_t *)&tmp;
                    std::memcpy(dest, src, msg_len);

                    auto msg = std::make_unique<nurirobot_msgs::msg::HCControl>();
                    msg->xaxis = tmp.x;
                    msg->yaxis = tmp.y;
                    msg->adc = tmp.volt;
                    msg->clickbutton = tmp.btn == 4 ? false : true;
                    msg->speed = tmp.speed;
                    u8SpeedStep = tmp.speed;
                    hc_ctrl_pub_->publish(*msg);              

                    auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
                    float x = convertRange(tmp.x);
                    float y = convertRange(tmp.y);                

                    joy_msg->axes.push_back(x);
                    joy_msg->axes.push_back(y);
                    joy_msg->buttons.push_back(tmp.btn == 4 ? false : true);

                    joy_msg->header.stamp = clock_.now();

                    hc_joy_pub_->publish(*joy_msg);

                    RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "y : %d, x:%d, adc: %d, btn: %d", tmp.y, tmp.x, tmp.volt, tmp.btn );
                    if (firstrun) firstrun = false;
                    recvFeedback[2] = true;
                    break;
                }
                case 0xd2:
                {
                    // 속도 피드백
                    // 현재 바퀴의 위치를 발행
                    uint8_t *src1 = (uint8_t *)&msg;
                    SpeedFeedbackResponse tmp1;
                    uint8_t *dest1 = (uint8_t *)&tmp1;
                    std::memcpy(dest1, src1, msg_len);

                    uint16_t tpos = tmp1.getValuePos();

                    RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "msgpos->id : %d msgpos->pos : %d", msg.id, tpos);
                    recvFeedback[msg.id == 0? 0 : 1] = true;
                    current_value_[msg.id == 0? 0 : 1] = tmp1.current / 10.0f;

                    pos_value_[msg.id == 0? 0: 1] = tpos;
                    if (!recvLastPos[msg.id == 0? 0: 1]) {
                        last_pos_value_[msg.id == 0? 0: 1] = tpos;
                        recvLastPos[msg.id == 0? 0: 1] = true;
                        RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "=========================== init pos ==============");
                    }
                    break;
                }
                default:
                    RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "msgpos->id : %d msg.mode : %d", msg.id, msg.mode);
                    break;
                }
            }
            else 
            {
                RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "protocol_recv 1 : %s %d %d",  toHexString(&msg, msg_len).c_str(), checksum, p[4]);
            }
            msg_len = 0;
        }
    }

    if (msg_len == sizeof(FeedbackResponse))
    {
        RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "recv feed: %s", toHexString(&msg, sizeof(msg)).c_str());
        msg_len = 0;
    }

    prev_byte = byte;

}


void nuri_humble_hwif::NuriSystemHardwareInterface::readHW()
{
    if (port_fd != -1)
    {
        uint8_t c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
        {
            protocol_recv(c);
        }

        // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "readHW");

        if (r < 0 && errno != EAGAIN)
            RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Reading failed: %d", r);
    }    
}

void nuri_humble_hwif::NuriSystemHardwareInterface::feedbackCall(uint8_t id)
{
    FeedbackCallCommand command;
    command.header = (uint16_t)START_FRAME;
    command.id = id;
    command.datasize = (uint8_t)0x02;
    command.mode = (uint8_t)0xa2;
    command.checksum = ~(uint8_t)(command.id + command.datasize + command.mode);

    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error writing to Nurirobot serial port");
    }
}


void nuri_humble_hwif::NuriSystemHardwareInterface::setRemote_callback(std_msgs::msg::Bool::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    remote = msg->data;

    if (remote)
    {
        commandRemoteStart();
    }
    else
    {
        commandRemoteStop();
    }
}

// void nuri_humble_hwif::NuriSystemHardwareInterface::byteMultiArrayCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
// {
//     if (port_fd == -1)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Attempt to write on closed serial");
//         return;
//     }

//     int rc = ::write(port_fd, msg->data.data(), msg->data.size());
//     if (rc < 0)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error writing to Nurirobot serial port");
//         return;
//     }

//     // std::ostringstream hex_stream;

//     // for (auto byte : msg->data) {
//     //     hex_stream << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
//     // }

//     // std::string hex_string = hex_stream.str();
//     // RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "size: %d, mc_rawdata : %s", msg->data.size(), hex_string);
// }

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nuri_humble_hwif::NuriSystemHardwareInterface,
    hardware_interface::SystemInterface)