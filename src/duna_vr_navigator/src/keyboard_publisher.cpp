#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <threads.h>
#include <unistd.h>
#include <termios.h>
// Keyboard publisher class
unsigned char ch = 0;

class KeyboardPublisher : public rclcpp::Node
{
public:
    void keyboardPoll()
    {
        RCLCPP_INFO(this->get_logger(), "Polling started...");
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        while (true)
        {
            // Make required changes and apply the settings
            newt.c_lflag &= ~(ICANON | ECHO);
            newt.c_iflag |= IGNBRK;
            newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
            newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
            newt.c_cc[VMIN] = 1;
            newt.c_cc[VTIME] = 0;
            tcsetattr(fileno(stdin), TCSANOW, &newt);

            // Get the current character
            ch = getchar();

            if (ch == '\x03')
            {
                tcsetattr(fileno(stdin), TCSANOW, &oldt);
                exit(0);
            }
            // Reapply old settings
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            RCLCPP_INFO(this->get_logger(), "Key: %c", ch);

            std_msgs::msg::String pub_data;
            pub_data.data =  std::string(1,ch);
            keyboard_msg_publisher->publish(pub_data);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    KeyboardPublisher() : Node("keyboard_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Keyboard publisher starting...");

        keyboard_msg_publisher = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
        // pub_timer = this->create_wall_timer(std::chrono::milliseconds(m_timer_period_msg), std::bind(&KeyboardPublisher::timerCallback,this));

        // Start polling thread
        polling_thread = new std::thread(&KeyboardPublisher::keyboardPoll, this);
    }

    virtual ~KeyboardPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "goodbye");
    }


private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keyboard_msg_publisher;
    rclcpp::TimerBase::SharedPtr pub_timer;
    std::thread *polling_thread;

    int m_timer_period_msg = 1000;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KeyboardPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}