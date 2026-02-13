#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "skeleton/srv/two_numbers.hpp"
#include "skeleton/srv/odd_or_even.hpp"
#include "skeleton/msg/square.hpp"

using namespace std::chrono_literals;

class CircleArea : public rclcpp::Node {
public:
    CircleArea() : rclcpp::Node("circle_area_node") {
        // subscriber to Topic_beta (int8)
        sub_shape_square_ = this->create_subscription<skeleton::msg::Square>(
        topic_name_shape_, 10,
        std::bind(&CircleArea::callback_circle_area, this, std::placeholders::_1)
        );

        // service for calc_two_nums
        service_two_num_ = this->create_service<skeleton::srv::TwoNumbers>(
            "calc_two_nums",
            std::bind(&CircleArea::service_two_nums, this, std::placeholders::_1, std::placeholders::_2) 
        );

        RCLCPP_INFO(this->get_logger(), "Service '/calc_two_nums' is ready.");

        // service for odd_or_even
        service_odd_even_ = this->create_service<skeleton::srv::OddOrEven>(
            "odd_or_even",
            std::bind(&CircleArea::service_odd_or_even, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service '/odd_or_even' is ready.");
  }

private:
    void callback_circle_area(const skeleton::msg::Square::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard from '%s': Area of a square with side '%.3f'is '%.3f'",
                    this->topic_name_shape_.c_str(), static_cast<double>(msg->side), static_cast<double>(msg->side * msg->side));
    }

    void service_odd_or_even(const skeleton::srv::OddOrEven::Request::SharedPtr request,
                                      skeleton::srv::OddOrEven::Response::SharedPtr response) {
        response->res = request->a;
        if (request->a % 2 == 0) {
            response->res = 0; // even
        } else {
            response->res = 1; // odd
        }

        RCLCPP_INFO(this->get_logger(), 
                    "Request: %d is odd: %d",
                    static_cast<int>(request->a),
                    static_cast<int>(response->res));
    }

    void service_two_nums(const skeleton::srv::TwoNumbers::Request::SharedPtr request,
                                   skeleton::srv::TwoNumbers::Response::SharedPtr response) {
        response->res = request->a + request->b;

        RCLCPP_INFO(this->get_logger(),
                    "Request: %.3f + %.3f = %.3f",
                    static_cast<double>(request->a),
                    static_cast<double>(request->b),
                    static_cast<double>(response->res));

        // std::this_thread::sleep_for(500ms); 
    }
    
    rclcpp::Service<skeleton::srv::TwoNumbers>::SharedPtr service_two_num_;
    rclcpp::Service<skeleton::srv::OddOrEven>::SharedPtr service_odd_even_;
    rclcpp::Subscription<skeleton::msg::Square>::SharedPtr sub_shape_square_;
    std::string topic_name_shape_ = "shape";
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleArea>());
    rclcpp::shutdown();
    return 0;
}