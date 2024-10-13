#include <rclcpp/rclcpp.hpp>
#include <my_interfaces/msg/my_message.hpp>

// 구독자 노드를 정의하는 클래스
class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("my_subscriber") {
        // "my_topic" 구독하고, 큐 사이즈는 10으로 설정
        subscription_ = this->create_subscription<my_interfaces::msg::MyMessage>(
            "my_topic", 10, std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // 메시지 수신 시 호출되는 콜백 
    void topic_callback(const my_interfaces::msg::MyMessage::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    // 구독 객체 저장
    rclcpp::Subscription<my_interfaces::msg::MyMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}

