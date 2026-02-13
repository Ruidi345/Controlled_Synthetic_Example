#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "skeleton/srv/two_numbers.hpp"
#include "skeleton/msg/circle.hpp"
#include "skeleton/msg/square.hpp"
#include "skeleton/msg/num.hpp"

using namespace std::chrono_literals;

class PerimeterNode : public rclcpp::Node {
public:
  PerimeterNode() : Node("perimeter_node") {
    // subscriber to shape square
    sub_shape_square_ = this->create_subscription<skeleton::msg::Square>(
        topic_name_shape_, 10,
        std::bind(&PerimeterNode::callback_square_perimeter, this, std::placeholders::_1)
    );

    // subscriber to shape circle
    sub_shape_circle_ = this->create_subscription<skeleton::msg::Circle>(
        topic_name_shape_, 10,
        std::bind(&PerimeterNode::callback_circle_perimeter, this, std::placeholders::_1)
    );

    // client to request calc_two_nums
    client_two_nums_ = this->create_client<skeleton::srv::TwoNumbers>(service_name_two_nums_);

    timer_ = this->create_wall_timer(500ms, [this]() {
        if (!client_two_nums_->wait_for_service(0s)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Waiting for service '%s'...", service_name_two_nums_.c_str());
            return;
        }

        auto req = std::make_shared<skeleton::srv::TwoNumbers::Request>();
        req->a = a_fixed_;
        req->b = b_fixed_;
        sent_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Calling service '%s' with a=%.3f, b=%.3f",
                    service_name_two_nums_.c_str(), req->a, req->b);

        client_two_nums_->async_send_request(req,
            [this](rclcpp::Client<skeleton::srv::TwoNumbers>::SharedFuture resp) {
            RCLCPP_INFO(this->get_logger(),
                        "Service response: sum=%.3f",
                        static_cast<double>(resp.get()->res));
        });

    });

  }

private:
    void callback_circle_perimeter(const skeleton::msg::Circle::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard from '%s': circumference of a circle with radius '%.3f'is '%.3f'",
                    this->topic_name_shape_.c_str(), static_cast<double>(msg->radius), 
                    static_cast<double>(M_PI * msg->radius * 2.0));
    }

    void callback_square_perimeter(const skeleton::msg::Square::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard from '%s': perimeter of a square with side '%.3f'is '%.3f'",
                    this->topic_name_shape_.c_str(), static_cast<double>(msg->side), 
                    static_cast<double>(msg->side * 4.0));
    }

    rclcpp::Subscription<skeleton::msg::Square>::SharedPtr sub_shape_square_;
    rclcpp::Subscription<skeleton::msg::Circle>::SharedPtr sub_shape_circle_;
    std::string topic_name_shape_ = "shape";
    std::string topic_name_counting_ = "counting2";

    rclcpp::Client<skeleton::srv::TwoNumbers>::SharedPtr client_two_nums_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool sent_{false};

    const std::string service_name_two_nums_{"calc_two_nums"};
    const double a_fixed_{2.0};
    const double b_fixed_{3.5};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerimeterNode>());
    rclcpp::shutdown();

    return 0;
}