#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// 一個發布字符串消息的發布者節點
class RePublisher : public rclcpp::Node {
public:
    RePublisher()
    : Node("republisher")  // 使用名稱 'republisher' 初始化節點
    {
        // 創建一個名為 'topic' 並且消息類型為 'String' 的發布者對象
        pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }

    void publish(const std_msgs::msg::String::SharedPtr msg)
    {
        // 創建一個 'String' 類型的消息對象
        auto message = std_msgs::msg::String();
        // 分配要發布的消息
        message.data = "Hello, World!";
        // 發布消息
        pub_->publish(message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// 一個訂閱字符串消息並轉發該消息到另一個主題的訂閱者節點
class SubscribeForward : public rclcpp::Node {
public:
    SubscribeForward()
    : Node("subscribe_forward")  // 使用名稱 'subscribe_forward' 初始化節點
    {
        // 創建一個名為 'topic' 並且消息類型為 'String' 的訂閱者對象
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SubscribeForward::callback, this, std::placeholders::_1)
        );

        // 用於重新發布消息的發布者對象
        pub_object_ = std::make_shared<RePublisher>();

        // 創建一個用於發布消息的線程
        thread_ = std::make_shared<std::thread>(&SubscribeForward::publish_thread, this);
    }

    ~SubscribeForward()
    {
        // 銷毀線程
        thread_->join();
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 打印接收到的消息
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        // 存儲消息
        msg_ = msg;
    }

    void publish_thread()
    {
        while (rclcpp::ok()) {
            // 發布消息
            pub_object_->publish(msg_);

            // 等待0.05秒
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
    // 初始化ROS2系統
    rclcpp::init(argc, argv);

    // 創建一個訂閱者節點
    auto node = std::make_shared<SubscribeForward>();

    // 保持節點運行
    rclcpp::spin(node);

    // 顯式銷毀節點
    rclcpp::shutdown();
    return 0;
}
