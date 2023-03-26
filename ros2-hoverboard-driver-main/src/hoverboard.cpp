#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/joint_state_interface.hpp>
#include <hardware_interface/joint_command_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager/controller_manager.hpp>
#include <dynamic_reconfigure/server.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

class Hoverboard : public rclcpp::Node, public controller_interface::Controller<HwInterface>
{
public:
    Hoverboard() : Node("hoverboard"), controller_interface::Controller<HwInterface>()
    {
        hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel",
									 &joints[0].pos.data,
									 &joints[0].vel.data,
									 &joints[0].eff.data);
        hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel",
									  &joints[1].pos.data,
									  &joints[1].vel.data,
									  &joints[1].eff.data);
        joint_state_interface_.register_handle (left_wheel_state_handle);
        joint_state_interface_.register_handle (right_wheel_state_handle);
        register_interface(&joint_state_interface_);

        hardware_interface::JointHandle left_wheel_vel_handle(
            joint_state_interface_.get_handle("left_wheel"),
            &joints[0].cmd.data);
        hardware_interface::JointHandle right_wheel_vel_handle(
            joint_state_interface_.get_handle("right_wheel"),
            &joints[1].cmd.data);
        velocity_joint_interface_.register_handle (left_wheel_vel_handle);
        velocity_joint_interface_.register_handle (right_wheel_vel_handle);
        register_interface(&velocity_joint_interface_);
    
    vel_pub_[0]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    vel_pub_[1]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    cmd_pub_[0]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    cmd_pub_[1]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    voltage_pub_   = this->create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);
    temp_pub_      = this->create_publisher<std_msgs::msg::Float64>("hoverboard/temperature", 10);

    // Create the subscriber to receive speed setpoints
    speeds_sub_   = this->create_subscription<wheel_msgs::msg::WheelSpeeds>("wheel_vel_setpoints",
                    10, std::bind(&Hoverboard::setpoint_callback, this, std::placeholders::_1));
    
    // Convert m/s to rad/s
    max_velocity /= wheel_radius;

    low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    last_wheelcountR = last_wheelcountL = 0;
    multR = multL = 0;

    auto nh_left = std::make_shared<rclcpp::Node>("pid/left");
    auto nh_right = std::make_shared<rclcpp::Node>("pid/right");
    // Init PID controller
    pids[0].init(nh_left, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[0].setOutputLimits(-max_velocity, max_velocity);
    pids[1].init(nh_right, 1.0, 0.0, 0.0, 0.01, 1.5, -1.5, true, max_velocity, -max_velocity);
    pids[1].setOutputLimits(-max_velocity, max_velocity);


    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to hoverboard");
        exit(-1); // TODO : put this again
    }
    
    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    // TODO : understand this shit
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
}
}
Hoverboard::~Hoverboard() { // Destructor implementation
    if (port_fd != -1) 
        close(port_fd);
}

void Hoverboard::setpoint_callback(wheel_msgs::msg::WheelSpeeds::UniquePtr msg)
{
    setpoint[0] = msg->right_wheel;
    setpoint[1] = msg->left_wheel;

    RCLCPP_INFO(this->get_logger(), "I heard something: %f, %f", setpoint[0], setpoint[1]);
}

void Hoverboard::read() {
    if (port_fd != -1) {
        uint8_t c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024){
            //RCLCPP_INFO(this->get_logger(), "Reading UART");
            protocol_recv(c);
        }            

        // if (i > 0)
        // last_read = ros::Time::now();

        if (r < 0 && errno != EAGAIN)
            RCLCPP_ERROR(this->get_logger(), "Reading from serial %s failed: %d", PORT, r);
    }

    // if ((ros::Time::now() - last_read).toSec() > 1) {
    //     RCLCPP_ERROR(this->get_logger(), "Timeout reading from serial %s failed", PORT);
    // }
}

void Hoverboard::protocol_recv (uint8_t byte) {
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
    //RCLCPP_INFO(this->get_logger(), "Received a byte: %x",(uint8_t)byte);
    // if ((uint8_t)byte == 0xAB && (uint8_t)prev_byte == 0xCD){
    //     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x", start_frame);
    //     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x %x", (byte) << 8, (uint8_t)prev_byte);
    // }

    // Read the start frame
    if (start_frame == START_FRAME) {
        //RCLCPP_INFO(this->get_logger(), "Start frame recognised");
        p = (uint8_t*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
            msg.batVoltage ^
            msg.boardTemp ^
            msg.cmdLed);

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            std_msgs::msg::Float64 f;

            f.data = (double)msg.batVoltage/100.0;
            voltage_pub_->publish(f);

            f.data = (double)msg.boardTemp/10.0;
            temp_pub_->publish(f);

            f.data = (double)msg.speedL_meas;
            vel_pub_[0]->publish(f);
            f.data = (double)msg.speedR_meas;
            vel_pub_[1]->publish(f);

            f.data = (double)msg.cmd1;
            cmd_pub_[0]->publish(f);
            f.data = (double)msg.cmd2;
            cmd_pub_[1]->publish(f);
        } else {
            RCLCPP_INFO(this->get_logger(), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}

void Hoverboard::write() {
    if (port_fd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }
    // Calculate steering from difference of left and right //TODO : change this shiiit
    const double speed = (setpoint[0] + setpoint[1])/2.0;
    const double steer = (setpoint[0] - setpoint[1])*2.0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd, (const void*)&command, sizeof(command));
    if (rc < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to hoverboard serial port");
    }
}
void Hoverboard::on_encoder_update (int16_t right, int16_t left) {
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
    // This section accumulates ticks even if board shuts down and is restarted   
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    
    //IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
    //(the board seems to often report 1-3 ticks on startup instead of zero)
    //reset the last read ticks to the startup values
    if((rclcpp::Clock().now() - last_read).seconds() > 0.2
		&& abs(posL) < 5 && abs(posR) < 5){
            lastPosL = posL;
            lastPosR = posR;
	}
    double posLDiff = 0;
    double posRDiff = 0;

    //if node is just starting keep odom at zeros
	if(nodeStartFlag){
		nodeStartFlag = false;
	}else{
            posLDiff = posL - lastPosL;
            posRDiff = posR - lastPosR;
	}

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;
    
    // Convert position in accumulated ticks to position in radians
    joints[0].pos = 2.0*M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
    joints[1].pos = 2.0*M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;

    pos_pub[0]->publish(joints[0].pos);
    pos_pub[1]->publish(joints[1].pos);
}