#include <chrono>
#include <string>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>
#include "rclcpp/rclcpp.hpp"
#include "skeleton/msg/num.hpp"
#include "skeleton/msg/circle.hpp"


using namespace std::chrono_literals;

class SquareArea : public rclcpp::Node {
public:
    explicit SquareArea(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : rclcpp::Node("square_area_node", options), count_(0)
    {
    // SquareArea() : Node("square_area_node", rclcpp::NodeOptions().namespace_("area")), count_(0) {
        // publisher to counting 
        pub_counting_ = this->create_publisher<skeleton::msg::Num>(topic_name_counting_, 10);
        timer_ = this->create_wall_timer(500ms, [this]() { this->tick(); });

        // subscriber to square circle
        sub_shape_circle_ = this->create_subscription<skeleton::msg::Circle>(
            topic_name_shape_, 10, std::bind(&SquareArea::callback_square_area, this, std::placeholders::_1)
        );

    }

private:
    void tick() {
        auto msg = skeleton::msg::Num();
        msg.num = static_cast<double>(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing a float number: '%s' to '%s'", 
                    std::string("0"  + std::to_string(msg.num) + " cnt").c_str(), topic_name_counting_.c_str());
        pub_counting_->publish(msg);
    }

    void callback_square_area(const skeleton::msg::Circle::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard from '%s': Area of a circle with radius '%.3f'is '%.3f'",
                    this->topic_name_shape_.c_str(), static_cast<double>(msg->radius), 
                    static_cast<double>(M_PI * msg->radius * msg->radius));
    }

    rclcpp::Publisher<skeleton::msg::Num>::SharedPtr pub_counting_;
    rclcpp::Subscription<skeleton::msg::Circle>::SharedPtr sub_shape_circle_;
    rclcpp::TimerBase::SharedPtr timer_;
    int32_t count_;
    std::string topic_name_counting_ = "counting";
    std::string topic_name_shape_ = "shape";

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareArea>());
    
    rclcpp::shutdown();
    return 0;
}
