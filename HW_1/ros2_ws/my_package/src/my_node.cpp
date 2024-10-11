#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/twist.hpp> 
#include <turtlesim/srv/teleport_absolute.hpp> 
#include <termios.h> 
#include <unistd.h> 
#include <iostream> 
#include <cmath> 

using namespace std::chrono_literals; 


class TurtleTeleop : public rclcpp::Node {
public:
    TurtleTeleop() : Node("my_node") { 노드 이름 설정
    
        
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

       
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleTeleop::keyLoop, this));
        RCLCPP_INFO(this->get_logger(), "Use arrow keys to control turtle, 'c' for circle, 'q' to quit, 'r' to reset, 's' for square, 't' for triangle.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; 
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_; 
    rclcpp::TimerBase::SharedPtr timer_; 

    
    int getch() {
        struct termios oldt, newt; 
        int ch; 

        
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt; 
        newt.c_lflag &= ~(ICANON | ECHO); 
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
        ch = getchar(); 
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
        return ch; 
    }

    void key() {
        geometry_msgs::msg::Twist twist; // 속도 메시지 초기화

        char c = getch();
        if (c == '\033') {  // 방향키 입력은 ANSI escape 시퀀스 사용
            getch();  //
            c = getch();  

         
            switch(c) {
                case 'A': // ↑ 방향키 
                    twist.linear.x = 1.0;
                    break;
                case 'B': // ↓ 방향키 
                    twist.linear.x = -1.0;
                    break;
                case 'C': // → 방향키 
                    twist.angular.z = -1.0;
                    break;
                case 'D': // ← 방향키
                    twist.angular.z = 1.0;
                    break;
                default:
                    break; 
            }
        } else {
            
            switch (c) {
                case 'q': // 종료 키
                    rclcpp::shutdown(); 
                    break;
                case 'r': 
                    teleportToCenter(); // 중앙으로 이동 
                    break;
                case 's': // 사각형
                    drawSquare();  
                    break;
                case 'c': // 원 
                    drawCircle(); 
                    break;
                case 't': // 삼각형 
                    drawTriangle(); 
                    break;
                default:
                    break; 
            }
        }

        
        pub_->publish(twist);
    }

    // 거북이 중앙으로 이동
    void teleportToCenter() {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>(); 
        request->x = 5.5; // 중앙 x좌표
        request->y = 5.5; // 중앙 y좌표
        request->theta = 0.0; // 방향

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return; 
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for teleport service...");
        }

        
        auto result = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Turtle reset to center."); 
    }

    // 사각형 
    void drawSquare() {
        geometry_msgs::msg::Twist twist; 
        for (int i = 0; i < 4; i++) { 
            twist.linear.x = 2.0; // 직진 속도
            twist.angular.z = 0.0; // 회전 속도 없음
            pub_->publish(twist); 
            rclcpp::sleep_for(2s); // 2초 동안 전진

            twist.linear.x = 0.0; // 속도 0
            twist.angular.z = M_PI_2; // 90도 회전
            pub_->publish(twist); 
            rclcpp::sleep_for(1s); 
        }
    }

    // 원 
    void drawCircle() {
        geometry_msgs::msg::Twist twist; 
        twist.linear.x = 3.0;  // 원 그리기 속도
        twist.angular.z = 3.0;  // 회전 속도
        
        // 원 그리기 
        for (int i = 0; i < 20; ++i) { 
            pub_->publish(twist); 
            rclcpp::sleep_for(100ms); 
        }
        
        // 다 그리면 멈춤
        twist.linear.x = 0.0; // 속도 0
        twist.angular.z = 0.0; // 회전 속도 0
        pub_->publish(twist); // 메시지 발행
    }

    // 삼각형 
    void drawTriangle() {
        geometry_msgs::msg::Twist twist; 
        for (int i = 0; i < 3; i++) { 
            twist.linear.x = 2.0; // 직진 속도
            twist.angular.z = 0.0; // 회전 속도 없음
            pub_->publish(twist); 
            rclcpp::sleep_for(2s); 

            twist.linear.x = 0.0; // 속도 0
            twist.angular.z = 2.0 * M_PI / 3.0; // 120도 회전
            pub_->publish(twist); 
            rclcpp::sleep_for(1s); 
        }
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    auto node = std::make_shared<TurtleTeleop>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 
    return 0; 
}

