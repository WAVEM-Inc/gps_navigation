#include "rclcpp/rclcpp.hpp"
#include "route_msgs/msg/drive_break.hpp"
#include <unistd.h>
#include <termios.h>

class DriveBreakPublisher : public rclcpp::Node
{
public:
    DriveBreakPublisher() : Node("drive_break_publisher"), break_pressure_(0)
    {
        // Publisher 설정
        publisher_ = this->create_publisher<route_msgs::msg::DriveBreak>("/drive/break", 10);

        // Timer 설정
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DriveBreakPublisher::publishBreak, this));

        // 키보드 입력 처리를 위한 설정
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= (~ICANON & ~ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
        setbuf(stdin, NULL);

        // 키보드 입력 감지를 위한 스레드 시작
        key_thread_ = std::thread(&DriveBreakPublisher::keyboardInputThread, this);
    }

private:
    void publishBreak()
    {
        auto message = route_msgs::msg::DriveBreak();
        message.break_pressure = break_pressure_;
        publisher_->publish(message);
	RCLCPP_INFO(this->get_logger(), "Current break pressure: %d", break_pressure_);
   
    }

    void keyboardInputThread()
    {
        char c;
        while (rclcpp::ok())
        {
            c = getchar();
            if (c == 'j')
            {
                break_pressure_++;
            }
            else if (c == 'k')
            {
                if (break_pressure_ > 0)
                {
                    break_pressure_--;
                }
            }
        }
    }

    rclcpp::Publisher<route_msgs::msg::DriveBreak>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread key_thread_;
    int32_t break_pressure_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveBreakPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

