#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "skeleton/srv/two_numbers.hpp"
#include "skeleton/msg/circle.hpp"
#include "skeleton/msg/age.hpp"

using namespace std::chrono_literals;

class SquareGenerator : public rclcpp::Node {
public:
    SquareGenerator() : Node("square_generating_node"), count_(0) {
        this->declare_parameter<bool>("use_service", false);
        bool use_service = this->get_parameter("use_service").as_bool();

        // publisher to shape circle
        pub_shape_circle_ = this->create_publisher<skeleton::msg::Circle>(topic_name_shape_, 10);
        timer_circle_ = this->create_wall_timer(500ms, [this]() { this->tick_circle(); });

        // publisher to Topic_gamma age
        pub_age_ = this->create_publisher<skeleton::msg::Age>(topic_name_person_age_, 10);
        timer_age_ = this->create_wall_timer(500ms, [this]() { this->tick_age(); });
       
    }

private:
    void tick_circle() {
        auto msg = skeleton::msg::Circle();
        msg.radius = 0.1 * count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: A circle with radius '%.3f' to '%s'", 
                    static_cast<double>(msg.radius * 0.1), topic_name_shape_.c_str());
        pub_shape_circle_->publish(msg);
    }

    void tick_age() {
        auto msg = skeleton::msg::Age();
        msg.age = count_++
        RCLCPP_INFO(this->get_logger(), "Publishing: Age '%d' to '%s'", static_cast<int>(msg.age), 
                    topic_name_person_age_.c_str());
        pub_age_->publish(msg);
    }

    rclcpp::Publisher<skeleton::msg::Circle>::SharedPtr pub_shape_circle_;
    rclcpp::Publisher<skeleton::msg::Age>::SharedPtr pub_age_;
    rclcpp::TimerBase::SharedPtr timer_circle_;
    rclcpp::TimerBase::SharedPtr timer_age_;
    int32_t count_;
    std::string topic_name_shape_ = "shape";
    std::string topic_name_person_age_ = "person_age";

    bool sent_{false};
    rclcpp::TimerBase::SharedPtr timer_ser_;
    const std::string service_name_two_nums_{"calc_two_nums"};
    const double a_fixed_{12.9};
    const double b_fixed_{13.0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareGenerator>());
    rclcpp::shutdown();
    return 0;
}