#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <dynamic_reconfigure/server.hpp>
#include "hoverboard.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hoverboard_driver");

    Hoverboard hoverboard;
    controller_manager::ControllerManager cm(&hoverboard, node);

    rclcpp::AsyncRate rate(100.0, node->get_executor());
    auto prev_time = node->now();

    while (rclcpp::ok()) {
        const auto time = node->now();
        const auto period = time - prev_time;
        prev_time = time;

        hoverboard.read();
        cm.update(time, period);
        hoverboard.write(time, period);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
