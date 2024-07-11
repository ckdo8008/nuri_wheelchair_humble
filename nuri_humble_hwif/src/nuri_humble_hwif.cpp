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

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Opened serial port to Nurirobot");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    commandRemoteStop();
    commandRemoteStop();
    commandRemoteStop();
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Wait connection to Motor Driver");
    while(rclcpp::ok())
    {
        feedbackHCCall();
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        readHW();
        // try
        // {
        //     // std::string recv_version;
        //     // ser_.ReadLine(recv_version, '\r', 500);

        //     // if(recv_version[0] == 'F' && recv_version[1] == 'I' && recv_version[2] == 'D')
        //     // {
        //     //     RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Connected %s", recv_version.c_str());
        //     //     break;
        //     // }
        //     // char buff;
        //     // try {
        //     //     while(true) {
        //     //         // ser_.ReadByte(&buff, 100);
        //     //         ser_.ReadByte( buff, 100 );
        //     //         protocol_recv(buff);
        //     //     }
        //     // } catch (const ReadTimeout&) {
        //     //     RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Read timeout occurred.");
        //     // } catch (const std::exception& ex) {
        //     //     RCLCPP_ERROR(rclcpp::get_logger("NuriSystemHardwareInterface"), "Error reading from Nurirobot serial port: %s", ex.what());
        //     // }
            
        // }
        // catch(LibSerial::ReadTimeout &e)
        // {
        //     rclcpp::sleep_for(std::chrono::milliseconds(2000));
        //     assert(false);
        // }
        // rclcpp::sleep_for(std::chrono::milliseconds(300));
        if (!firstrun) break;
    }

    u8ArrivalSec = std::stof(info_.hardware_parameters["robot_acceleration"]);
    // auto robot_dec = std::stof(info_.hardware_parameters["robot_deceleration"]);
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Get robot_acceleration [%6.1f] sec", u8ArrivalSec / 10.0);
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Get robot_deceleration [%6.1f] RPM/s", robot_dec / 10.0);

    // ser_.Write("!R 2\r"); // Restart Script
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    // ser_.FlushIOBuffers();

    // ser_.Write("!B 3 1\r");
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(100));
    // ser_.FlushIOBuffers();


    // auto conf_str = boost::format("!AC 1 %1%\r") % int(robot_acc);
    // ser_.Write(conf_str.str());
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(100));
    // conf_str = boost::format("!AC 2 %1%\r") % int(robot_acc);
    // ser_.Write(conf_str.str());
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(100));

    // conf_str = boost::format("!DC 1 %1%\r") % int(robot_dec);
    // ser_.Write(conf_str.str());
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(100));
    // conf_str = boost::format("!DC 2 %1%\r") % int(robot_dec);
    // ser_.Write(conf_str.str());
    // ser_.DrainWriteBuffer();
    // rclcpp::sleep_for(std::chrono::milliseconds(100));

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

    // ser_.Write("?A_?AI_?C_?FF_?T 1_?V 2_?DI\r");
    // ser_.DrainWriteBuffer();

    double motor_current[2] = {0, };
    long int current_encoder[2] = {0, };

    while(rclcpp::ok())
    {
        recvFeedback[2] = false;
        feedbackHCCall();
        rclcpp::sleep_for(std::chrono::milliseconds(50));

        readHW();
        if (recvFeedback[2]) break;
    }

    while(rclcpp::ok())
    {
        recvFeedback[0] = false;
        feedbackCall(0);
        rclcpp::sleep_for(std::chrono::milliseconds(70));

        readHW();
        if (recvFeedback[0]) break;
    }

    while(rclcpp::ok())
    {
        recvFeedback[1] = false;
        feedbackCall(1);
        rclcpp::sleep_for(std::chrono::milliseconds(70));

        readHW();
        if (recvFeedback[1]) break;
    }    

    // while(rclcpp::ok())
    // {
    //     std::string recv_data;
    //     // try
    //     // {
    //     //     break;
    //     //     // ser_.ReadLine(recv_data, '\r', 100);
    //     //     RCLCPP_DEBUG(rclcpp::get_logger("NuriSystemHardwareInterface"), "%s", recv_data.c_str());
    //     //     if(recv_data[0] == 'A' && recv_data[1] == '=')
    //     //     {
    //     //         auto data = recv_data.substr(2);
    //     //         std::vector<std::string> data_array;
    //     //         boost::split(data_array, data, boost::algorithm::is_any_of(":"));
    //     //         motor_current[0] = atof(data_array[0].c_str()) / 10.0;
    //     //         motor_current[1] = atof(data_array[1].c_str()) / 10.0;
    //     //     }
    //     //     if(recv_data[0] == 'A' && recv_data[1] == 'I')
    //     //     {
    //     //         auto data = recv_data.substr(3);
    //     //         std::vector<std::string> data_array;
    //     //         boost::split(data_array, data, boost::algorithm::is_any_of(":"));
    //     //         charging_voltage_ = atof(data_array[2].c_str());
    //     //         user_power_current1_ = atof(data_array[3].c_str()) / 1000.0;
    //     //         user_power_current2_ = atof(data_array[4].c_str()) / 1000.0;
    //     //     }
    //     //     if(recv_data[0] == 'C' && recv_data[1] == '=')
    //     //     {
    //     //         auto data = recv_data.substr(2);
    //     //         std::vector<std::string> data_array;
    //     //         boost::split(data_array, data, boost::algorithm::is_any_of(":"));
    //     //         current_encoder[0] = atoi(data_array[0].c_str());
    //     //         current_encoder[1] = atoi(data_array[1].c_str());
    //     //     }
    //     //     if(recv_data[0] == 'F' && recv_data[1] == 'F')
    //     //     {
    //     //         auto data = recv_data.substr(3);
    //     //         enable_motor_state_ = atoi(data.c_str()) == 16 ? 0.0 : 1.0;
    //     //         fault_flags_ = atoi(data.c_str());
    //     //     }
    //     //     if(recv_data[0] == 'T' && recv_data[1] == '=')
    //     //     {
    //     //         auto data = recv_data.substr(2);
    //     //         current_temperature_ = atof(data.c_str());
    //     //     }
    //     //     if(recv_data[0] == 'V' && recv_data[1] == '=')
    //     //     {
    //     //         auto data = recv_data.substr(2);
    //     //         system_voltage_ = atof(data.c_str()) / 10.0;
    //     //     }
    //     //     if(recv_data[0] == 'D' && recv_data[1] == 'I')
    //     //     {
    //     //         auto data = recv_data.substr(3);
    //     //         std::vector<std::string> data_array;
    //     //         boost::split(data_array, data, boost::algorithm::is_any_of(":"));
    //     //         estop_button_state_ = atoi(data_array[0].c_str()) == 1 ? 0.0 : 1.0;
    //     //         break;
    //     //     }
    //     // }
    //     // catch(LibSerial::ReadTimeout &e)
    //     // {
    //     //     // assert(false && "Timeout for read motor states...");
    //     //     break;
    //     // }

    //     break;
    // }

    // clock_gettime(CLOCK_MONOTONIC, &end);
    // auto diff = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "READ: %lld", diff);

    hw_positions_[0] += (double)(l_last_enc_ - current_encoder[0]) / 16384.0 * (2.0 * M_PI) * -1.0;
    hw_positions_[1] += (double)(r_last_enc_ - current_encoder[1]) / 16384.0 * (2.0 * M_PI) * -1.0;
    hw_velocities_[0] = 0.0;
    hw_velocities_[1] = 0.0;
    hw_efforts_[0] = motor_current[0];
    hw_efforts_[1] = motor_current[1];

    l_last_enc_ = current_encoder[0];
    r_last_enc_ = current_encoder[1];

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type NuriSystemHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration & /*period*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "write");
    // write command to motor driver

    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
        // Simulate sending commands to the hardware
        RCLCPP_INFO(
            rclcpp::get_logger("NuriSystemHardwareInterface"), "Got command %.5f for '%s'!", hw_commands_[i],
            info_.joints[i].name.c_str());

        hw_velocities_[i] = hw_commands_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "Joints successfully written!");

    int16_t l_rpm = hw_commands_[0] / (2.0 * M_PI) * 60.0;
    int16_t r_rpm = hw_commands_[1] / (2.0 * M_PI) * 60.0;

    int16_t cmd_l_motor = l_rpm / 200.0 * 1000.0;
    int16_t cmd_r_motor = r_rpm / 200.0 * 1000.0;

    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "%f %f %d %d", hw_commands_[0], hw_commands_[1], l_rpm, r_rpm);

    auto cmd_str = boost::format("");
    if(enable_motor_cmd_ != is_enable_motor_processed_)
    {
        if(enable_motor_cmd_ == 1.0)
        {
            cmd_str = boost::format("!MG\r");
        }
        else
        {
            cmd_str = boost::format("!EX\r");
        }

        is_enable_motor_processed_ = enable_motor_cmd_;
    }

    if(is_estop_processed_ != estop_button_state_)
    {
        if(estop_button_state_ == 1.0)
        {
            cmd_str = boost::format("!EX\r");
        }
        else
        {
            cmd_str = boost::format("!MG\r");
        }
        is_estop_processed_ = estop_button_state_;
    }

    if(enable_motor_state_ == 1.0 && estop_button_state_ == 0 && enable_motor_cmd_ == 1.0)
    {
        cmd_str = boost::format("!G 1 %1%_!G 2 %2%\r") % cmd_l_motor % cmd_r_motor;
        RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "%s", cmd_str.str().c_str());
    }

    // ser_.Write(cmd_str.str());
    // ser_.DrainWriteBuffer();
    // ser_.Write("!B 3 1\r");
    // ser_.DrainWriteBuffer();
    // ser_.FlushIOBuffers();

    return hardware_interface::return_type::OK;
}
}

void nuri_humble_hwif::NuriSystemHardwareInterface::commandRemoteStop()
{
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
                    // hc_ctrl_pub_->publish(*msg);              

                    auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
                    float x = convertRange(tmp.x);
                    float y = convertRange(tmp.y);                

                    joy_msg->axes.push_back(x);
                    joy_msg->axes.push_back(y);
                    joy_msg->buttons.push_back(tmp.btn == 4 ? false : true);

                    joy_msg->header.stamp = clock_.now();

                    // hc_joy_pub_->publish(*joy_msg);

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

                    auto msgpos = std::make_unique<nurirobot_msgs::msg::NurirobotPos>();
                    msgpos->id = tmp1.id;
                    msgpos->pos = tmp1.getValuePos(); 
                    // pos_pub_->publish(*msgpos);

                    auto msglrpos = std::make_unique<nurirobot_msgs::msg::NurirobotPos>();
                    msglrpos->header.stamp = clock_.now();
                    msglrpos->header.frame_id = "pos";
                    msglrpos->id = tmp1.id;
                    msglrpos->pos = tmp1.getValuePos(); 

                    // if (tmp1.id == 0)
                    //     left_pos_pub_->publish(*msglrpos);
                    // else 
                    //     right_pos_pub_->publish(*msglrpos);

                    auto msgspeed = std::make_unique<nurirobot_msgs::msg::NurirobotSpeed>();
                    msgspeed->id = tmp1.id;
                    msgspeed->speed = tmp1.getValueSpeed() * 0.1f;
                    // speed_pub_->publish(*msgspeed);

                    RCLCPP_INFO(rclcpp::get_logger("NuriSystemHardwareInterface"), "msgpos->id : %d msgpos->pos : %d", msgpos->id, msgpos->pos);
                    recvFeedback[msg.id == 0? 0 : 1] = true;
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

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nuri_humble_hwif::NuriSystemHardwareInterface, 
    hardware_interface::SystemInterface)