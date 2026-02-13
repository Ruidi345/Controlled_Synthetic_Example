#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include "rclcpp/rclcpp.hpp"
#include "skeleton/srv/odd_or_even.hpp"
#include "skeleton/msg/num.hpp"
#include "skeleton/msg/square.hpp"
#include "skeleton/msg/circle.hpp"

using namespace std::chrono_literals;

class CircleGenerator : public rclcpp::Node {
public:

    explicit CircleGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : rclcpp::Node("circle_generating_node", options), count_(0)
    {
        this->declare_parameter<bool>("use_service", false);
        bool use_service = this->get_parameter("use_service").as_bool();

        // subscriber to counting
        sub_counting_ = this->create_subscription<skeleton::msg::Num>(
            topic_name_counting_, 10, 
            std::bind(&CircleGenerator::callback_counting_logger, this, std::placeholders::_1)
        );

        // publisher to shape square
        pub_shape_square_ = this->create_publisher<skeleton::msg::Circle>(topic_name_beta_, 10);
        timer_ = this->create_wall_timer(500ms, [this]() { this->tick(); });

        if (use_service) {
            // client to request odd_or_even
            client_odd_even_ = this->create_client<skeleton::srv::OddOrEven>(service_name_);

            timer_2_ = this->create_wall_timer(500ms, [this]() {
            if (!client_odd_even_->wait_for_service(0s)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "Waiting for service '%s'...", service_name_.c_str());
                return;
            }

            auto req = std::make_shared<skeleton::srv::OddOrEven::Request>();
            req->a = a_fixed_;
            sent_ = true;

            RCLCPP_INFO(this->get_logger(),
                        "Calling service '%s' with a=%d",
                        service_name_.c_str(), static_cast<int>(req->a));

            client_odd_even_->async_send_request(req,
                [this](rclcpp::Client<skeleton::srv::OddOrEven>::SharedFuture resp) {
                RCLCPP_INFO(this->get_logger(),
                            "Service response: res=%d",
                            static_cast<int>(resp.get()->res));
                });
                
            });            
        }

    }

private:
    void tick() {
        auto msg = skeleton::msg::Circle();
        msg.radius = static_cast<double>(count_++); ;
        RCLCPP_INFO(this->get_logger(),"Publishing: A square with side '%.3f' to '%s'", msg.radius * 0.1, 
            topic_name_beta_.c_str());
        pub_shape_square_->publish(msg);
    }

    void callback_counting_logger(const skeleton::msg::Num::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Heard from '%s': '%.3f'",
                    this->topic_name_counting_.c_str(), static_cast<double>(msg->num));
    }

    std::string topic_name_counting_ = "counting";
    std::string topic_name_beta_ = "shape";
    rclcpp::TimerBase::SharedPtr timer_;
    int32_t count_;
    
    rclcpp::Subscription<skeleton::msg::Num>::SharedPtr sub_counting_;
    rclcpp::Publisher<skeleton::msg::Circle>::SharedPtr pub_shape_square_;

    rclcpp::Client<skeleton::srv::OddOrEven>::SharedPtr client_odd_even_;
    rclcpp::TimerBase::SharedPtr timer_2_;
    const std::string service_name_{"odd_or_even"};
    const double a_fixed_{4};
    bool sent_{false};
};


RCLCPP_COMPONENTS_REGISTER_NODE(CircleGenerator)

// int main(int argc, char * argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CircleGenerator>("asjklfhaso"));
//     rclcpp::shutdown();
//     return 0;
// }