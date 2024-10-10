#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <boost/asio.hpp>
#include <string>
#include <regex>
#include <limits>  // 추가된 헤더

using boost::asio::serial_port;
using boost::asio::io_service;
using boost::asio::buffer;

class UltraSonicReader : public rclcpp::Node
{
public:
    UltraSonicReader(const std::string& port_name)
        : Node("ultrasonic_reader"),
          io_service_(),
          serial_port_(io_service_),
          port_name_(port_name)
    {
        // 시리얼 포트 열기
        try
        {
            serial_port_.open(port_name_);
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));

            RCLCPP_INFO(this->get_logger(), "Opened serial port: %s", port_name_.c_str());
        }
        catch (boost::system::system_error& e)  // 예외 타입 수정
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 디바이스 이름에서 접미사 결정
        if (port_name_.find("ttyUSLeft") != std::string::npos)
        {
            suffix_ = "_Left";
            frame_id_ = "sensor_left";
        }
        else if (port_name_.find("ttyUSRight") != std::string::npos)
        {
            suffix_ = "_Right";
            frame_id_ = "sensor_right";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown device suffix for port: %s", port_name_.c_str());
            rclcpp::shutdown();
            return;
        }

        // 토픽 생성
        front_pub_ = this->create_publisher<sensor_msgs::msg::Range>("Front" + suffix_, 10);
        front_side_pub_ = this->create_publisher<sensor_msgs::msg::Range>("FrontSide" + suffix_, 10);
        rear_side_pub_ = this->create_publisher<sensor_msgs::msg::Range>("RearSide" + suffix_, 10);
        rear_pub_ = this->create_publisher<sensor_msgs::msg::Range>("Rear" + suffix_, 10);

        // 비동기 읽기 시작
        start_async_read();

        // 스레드 시작
        io_thread_ = std::thread([this]() { io_service_.run(); });
    }

    ~UltraSonicReader()
    {
        io_service_.stop();
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }
    }

private:
    void start_async_read()
    {
        boost::asio::async_read_until(
            serial_port_,
            buffer_,
            '\n',
            [this](const boost::system::error_code& ec, std::size_t bytes_transferred)
            {
                this->on_read(ec, bytes_transferred);
            }
        );
    }

    void on_read(const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", ec.message().c_str());
            return;
        }

        std::istream is(&buffer_);
        std::string line;
        std::getline(is, line);

        // 데이터 파싱
        parse_line(line);

        // 다음 읽기 시작
        start_async_read();
    }

    void parse_line(const std::string& line)
    {
        // 정규식을 사용하여 숫자 추출
        std::regex regex_pattern(R"(I\s+\(\d+:\d{2}:\d{2}\.\d{3}\)\s+UART_(Left|Right):\s*(-?\d+),(-?\d+),(-?\d+),(-?\d+))");
        std::smatch match;

        if (std::regex_search(line, match, regex_pattern))
        {
            if (match.size() == 6)
            {
                try
                {
                    float front = std::stof(match[2]);
                    float front_side = std::stof(match[3]);
                    float rear_side = std::stof(match[4]);
                    float rear = std::stof(match[5]);

                    // 거리 값을 mm에서 m로 변환
                    if (front != -1.0f)
                        front /= 1000.0f;
                    if (front_side != -1.0f)
                        front_side /= 1000.0f;
                    if (rear_side != -1.0f)
                        rear_side /= 1000.0f;
                    if (rear != -1.0f)
                        rear /= 1000.0f;

                    // 데이터 발행
                    publish_data(front, front_side, rear_side, rear);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing numbers: %s", e.what());
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Line did not match expected format: %s", line.c_str());
        }
    }

    void publish_data(float front, float front_side, float rear_side, float rear)
    {
        auto now = this->get_clock()->now();

        // Front 센서 메시지 생성 및 발행
        auto front_msg = sensor_msgs::msg::Range();
        front_msg.header.stamp = now;
        front_msg.header.frame_id = frame_id_;
        front_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        front_msg.field_of_view = 0.1;  // 예시 값 (라디안)
        front_msg.min_range = 0.02;     // 최소 측정 거리 (m)
        front_msg.max_range = 4.0;      // 최대 측정 거리 (m)
        if (front == -1.0f)
            front_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            front_msg.range = front;
        front_pub_->publish(front_msg);

        // FrontSide 센서 메시지 생성 및 발행
        auto front_side_msg = sensor_msgs::msg::Range();
        front_side_msg.header.stamp = now;
        front_side_msg.header.frame_id = frame_id_;
        front_side_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        front_side_msg.field_of_view = 0.1;
        front_side_msg.min_range = 0.02;
        front_side_msg.max_range = 4.0;
        if (front_side == -1.0f)
            front_side_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            front_side_msg.range = front_side;
        front_side_pub_->publish(front_side_msg);

        // RearSide 센서 메시지 생성 및 발행
        auto rear_side_msg = sensor_msgs::msg::Range();
        rear_side_msg.header.stamp = now;
        rear_side_msg.header.frame_id = frame_id_;
        rear_side_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        rear_side_msg.field_of_view = 0.1;
        rear_side_msg.min_range = 0.02;
        rear_side_msg.max_range = 4.0;
        if (rear_side == -1.0f)
            rear_side_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            rear_side_msg.range = rear_side;
        rear_side_pub_->publish(rear_side_msg);

        // Rear 센서 메시지 생성 및 발행
        auto rear_msg = sensor_msgs::msg::Range();
        rear_msg.header.stamp = now;
        rear_msg.header.frame_id = frame_id_;
        rear_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        rear_msg.field_of_view = 0.1;
        rear_msg.min_range = 0.02;
        rear_msg.max_range = 4.0;
        if (rear == -1.0f)
            rear_msg.range = std::numeric_limits<float>::quiet_NaN();
        else
            rear_msg.range = rear;
        rear_pub_->publish(rear_msg);
    }

    // 멤버 변수
    io_service io_service_;
    serial_port serial_port_;
    boost::asio::streambuf buffer_;
    std::thread io_thread_;

    std::string port_name_;
    std::string suffix_;
    std::string frame_id_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_side_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rear_side_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rear_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // 파라미터로 디바이스 경로 받기
    std::string port_name = "/dev/ttyUSLeft";  // 기본값

    if (argc > 1)
    {
        port_name = argv[1];
    }

    auto node = std::make_shared<UltraSonicReader>(port_name);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
