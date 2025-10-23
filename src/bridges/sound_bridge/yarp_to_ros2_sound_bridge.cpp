#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Sound.h>

using namespace std::chrono_literals;

class YarpToRos2SoundBridge : public rclcpp::Node
{
public:
    YarpToRos2SoundBridge()
        : Node("yarp_to_ros2_sound_bridge")
    {
        // init YARP
        if (!yarp::os::Network::checkNetwork())
        {
            throw std::runtime_error("YARP non found");
        }

        // open YARP port
        inputPort.open("/sound:i");

        // Publisher ROS2
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("/sound_array", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&YarpToRos2SoundBridge::publishSound, this));
    }

    ~YarpToRos2SoundBridge() override
    {
        inputPort.close();
        yarp::os::Network::fini();
    }

private:
    void publishSound()
    {
        yarp::sig::Sound *sound = inputPort.read(false);
        if (!sound) return;  // no new data

        std_msgs::msg::Int16MultiArray msg;

        size_t samples = sound->getSamples();
        size_t channels = sound->getChannels();

        msg.data.reserve(samples * channels);

        for (size_t i = 0; i < samples; ++i) {
            for (size_t c = 0; c < channels; ++c) {
                int val = static_cast<int>(sound->get(i, c));
                msg.data.push_back(val);
            }
        }

        publisher_->publish(msg);
    }

    yarp::os::BufferedPort<yarp::sig::Sound> inputPort;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    yarp::os::Network yarp;
    auto node = std::make_shared<YarpToRos2SoundBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
