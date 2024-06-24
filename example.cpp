#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// �@�ӵo���r�Ŧ�������o���̸`�I
class RePublisher : public rclcpp::Node {
public:
    RePublisher()
    : Node("republisher")  // �ϥΦW�� 'republisher' ��l�Ƹ`�I
    {
        // �Ыؤ@�ӦW�� 'topic' �åB���������� 'String' ���o���̹�H
        pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }

    void publish(const std_msgs::msg::String::SharedPtr msg)
    {
        // �Ыؤ@�� 'String' ������������H
        auto message = std_msgs::msg::String();
        // ���t�n�o��������
        message.data = "Hello, World!";
        // �o������
        pub_->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// �@�ӭq�\�r�Ŧ��������o�Ӯ�����t�@�ӥD�D���q�\�̸`�I
class SubscribeForward : public rclcpp::Node {
public:
    SubscribeForward()
    : Node("subscribe_forward")  // �ϥΦW�� 'subscribe_forward' ��l�Ƹ`�I
    {
        // �Ыؤ@�ӦW�� 'topic' �åB���������� 'String' ���q�\�̹�H
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SubscribeForward::callback, this, std::placeholders::_1)
        );

        // �Ω󭫷s�o���������o���̹�H
        pub_object_ = std::make_shared<RePublisher>();

        // �Ыؤ@�ӥΩ�o���������u�{
        thread_ = std::make_shared<std::thread>(&SubscribeForward::publish_thread, this);
    }

    ~SubscribeForward()
    {
        // �P���u�{
        thread_->join();
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // ���L�����쪺����
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        // �s�x����
        msg_ = msg;
    }

    void publish_thread()
    {
        while (rclcpp::ok()) {
            // �o������
            pub_object_->publish(msg_);

            // ����0.05��
            std::this_thread::sleep_for(50ms);
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std_msgs::msg::String::SharedPtr msg_;
    std::shared_ptr<RePublisher> pub_object_;
    std::shared_ptr<std::thread> thread_;
};

int main(int argc, char * argv[])
{
    // ��l��ROS2�t��
    rclcpp::init(argc, argv);

    // �Ыؤ@�ӭq�\�̸`�I
    auto node = std::make_shared<SubscribeForward>();

    // �O���`�I�B��
    rclcpp::spin(node);

    // �㦡�P���`�I
    rclcpp::shutdown();
    return 0;
}
