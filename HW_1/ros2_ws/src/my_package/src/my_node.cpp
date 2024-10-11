#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/twist.hpp> 
#include <turtlesim/srv/teleport_absolute.hpp> 
#include <termios.h> 
#include <unistd.h> 
#include <iostream> 
#include <cmath> 

using namespace std::chrono_literals; // 시간 단위 사용을 위한 네임스페이스

// 거북이를 조종하는 노드 클래스 정의
class TurtleTeleop : public rclcpp::Node {
public:
    TurtleTeleop() : Node("my_node") { 노드 이름 설정
    
        // 퍼블리셔 생성: 'turtle1/cmd_vel' 토픽으로 속도 메시지 전송
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle1/teleport_absolute");

        // 100ms 주기로 키 입력을 처리하는 타이머 설정
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleTeleop::keyLoop, this));
        RCLCPP_INFO(this->get_logger(), "Use arrow keys to control turtle, 'c' for circle, 'q' to quit, 'r' to reset, 's' for square, 't' for triangle.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_; // 퍼블리셔 포인터
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_; // 클라이언트 포인터
    rclcpp::TimerBase::SharedPtr timer_; // 타이머 포인터

    
    int getch() {
        struct termios oldt, newt; // 터미널 설정을 위한 구조체
        int ch; 

        // 현재 터미널 속성 저장
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt; 
        newt.c_lflag &= ~(ICANON | ECHO); 
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
        ch = getchar(); // 문자 입력 받기
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
        return ch; // 읽은 문자 반환
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
                    break; // 예외 처리
            }
        } else {
            // 특수 키 처리
            switch (c) {
                case 'q': // 종료 키
                    rclcpp::shutdown(); 
                    break;
                case 'r': 
                    teleportToCenter(); // 중앙으로 이동 
                    break;
                case 's': // 사각형 그리기
                    drawSquare(); // 사각형 
                    break;
                case 'c': // 원 그리기
                    drawCircle(); // 원 
                    break;
                case 't': // 삼각형 그리기
                    drawTriangle(); // 삼각형 
                    break;
                default:
                    break; 
            }
        }

        
        pub_->publish(twist);
    }

    // 거북이를 중앙으로 이동
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

    // 사각형 그리기 함수
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

    // 원 그리기 함수 
    void drawCircle() {
        geometry_msgs::msg::Twist twist; 
        twist.linear.x = 3.0;  // 원 그리기 속도
        twist.angular.z = 3.0;  // 회전 속도
        
        // 원 그리기 
        for (int i = 0; i < 20; ++i) { // 반복 횟수를 통해 원 그리기.
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

// 메인 함수
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = std::make_shared<TurtleTeleop>(); // 노드 생성
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // 종료
    return 0; // 프로그램 종료
}

