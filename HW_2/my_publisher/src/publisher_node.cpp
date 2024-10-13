#include <rclcpp/rclcpp.hpp>
#include <my_interfaces/msg/my_message.hpp>
#include <iostream>

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("my_publisher") {
        // "my_topic" 발행하며, 큐 사이즈는 10으로 설정
        publisher_ = this->create_publisher<my_interfaces::msg::MyMessage>("my_topic", 10);
        // 1초마다 publish_message 함수를 호출
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MyPublisher::publish_message, this));
    }

private:
    // 메시지 발행
    void publish_message() {
        std::string input;
        std::cout << "Enter a message: ";
        std::getline(std::cin, input);

        // 메시지 생성, 사용자 입력으로 설정
        auto message = my_interfaces::msg::MyMessage();
        message.data = input;
       
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

  
    rclcpp::Publisher<my_interfaces::msg::MyMessage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}

