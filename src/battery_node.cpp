#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "fit.hpp"

enum class DischargeModel
{
    Exponential,
    Linear,
    Inverse
};

class BatterySimulator : public rclcpp::Node 
{
private:
    // Node parameters
    std::string discharge_model_;
    float max_voltage_;
    float min_voltage_;
    float base_voltage_;
    int initial_percent_;
    int discharge_current_;
    int recharge_current_;
    int base_power_consumption_;
    int motors_power_consumption_;
    int num_batteries_;
    std::string cmd_vel_topic_;
    bool verbose_;

    // Battery variables
    int total_discharge_current_;
    float total_power_;
    float voltage_;
    float percent_;
    float power_;
    bool recharging_;
    float discharge_rate_;
    float recharge_rate_;
    DischargeModel model_;
    std::string model_name_;
    Equation eq_;

    // Publishers and Subscribers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percent_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recharging_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recharge_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr replace_battery_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float map_number(float x, float in_min, float in_max, float out_min, float out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void recharge_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        recharging_ = msg->data;
        if (recharging_)
            recharge_rate_ = recharge_current_ / (float)(60 * 60 * 1000);
        else
            recharge_rate_ = 0;
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y + msg->linear.z * msg->linear.z);
        double angular = std::sqrt(msg->angular.x * msg->angular.x + msg->angular.y * msg->angular.y + msg->angular.z * msg->angular.z);

        if (linear > 0 || angular > 0)
            discharge_rate_ = (motors_power_consumption_ + base_power_consumption_) / (float)(60 * 60 * 1000);
        else
            discharge_rate_ = base_power_consumption_ / (float)(60 * 60 * 1000);
    }

    void update_battery()
    {
        power_ -= discharge_rate_ * base_voltage_;
        power_ += recharge_rate_ * base_voltage_;

        power_ = std::max(0.0f, std::min(power_, total_power_));

        voltage_ = eq_.evaluate(power_);
        voltage_ = std::max(min_voltage_, std::min(voltage_, max_voltage_));
        percent_ = map_number(voltage_, min_voltage_, max_voltage_, 0, 100);

        if (verbose_)
        {
            RCLCPP_INFO(this->get_logger(), "Discharge rate = %fmAs", (discharge_rate_ * 1000));
            RCLCPP_INFO(this->get_logger(), "Recharge rate = %fmAs", (recharge_rate_ * 1000));
            RCLCPP_INFO(this->get_logger(), "Power = %dWh", (int)power_);
            RCLCPP_INFO(this->get_logger(), "Voltage = %fV", voltage_);
            RCLCPP_INFO(this->get_logger(), "Percent = %f%%", percent_);
        }

        auto power_msg = std_msgs::msg::Int32();
        power_msg.data = (int)power_;

        auto voltage_msg = std_msgs::msg::Float32();
        voltage_msg.data = voltage_;

        auto percent_msg = std_msgs::msg::Float32();
        percent_msg.data = percent_;

        auto recharging_msg = std_msgs::msg::Bool();
        recharging_msg.data = recharging_;

        power_pub_->publish(power_msg);
        voltage_pub_->publish(voltage_msg);
        percent_pub_->publish(percent_msg);
        recharging_pub_->publish(recharging_msg);
    }

    void setup_discharge_model()
    {
        float d_voltage = max_voltage_ - min_voltage_;
        std::vector<FloatPoint> data;
        int eq_deg = 1;

        switch (model_)
        {
            case DischargeModel::Linear:
                eq_deg = 1;
                data.push_back(FloatPoint(total_power_, max_voltage_));
                data.push_back(FloatPoint(total_power_ * 0.75f, min_voltage_ + d_voltage * 0.75f));
                data.push_back(FloatPoint(total_power_ * 0.5f, min_voltage_ + d_voltage * 0.5f));
                data.push_back(FloatPoint(total_power_ * 0.25f, min_voltage_ + d_voltage * 0.25f));
                data.push_back(FloatPoint(0, min_voltage_));
                break;
            case DischargeModel::Exponential:
                eq_deg = 4;
                data.push_back(FloatPoint(total_power_, max_voltage_));
                data.push_back(FloatPoint(total_power_ * 0.9f, min_voltage_ + d_voltage * 0.98f));
                data.push_back(FloatPoint(total_power_ * 0.8f, min_voltage_ + d_voltage * 0.95f));
                data.push_back(FloatPoint(total_power_ * 0.7f, min_voltage_ + d_voltage * 0.90f));
                data.push_back(FloatPoint(total_power_ * 0.6f, min_voltage_ + d_voltage * 0.85f));
                data.push_back(FloatPoint(total_power_ * 0.5f, min_voltage_ + d_voltage * 0.80f));
                data.push_back(FloatPoint(total_power_ * 0.4f, min_voltage_ + d_voltage * 0.75f));
                data.push_back(FloatPoint(total_power_ * 0.3f, min_voltage_ + d_voltage * 0.70f));
                data.push_back(FloatPoint(total_power_ * 0.2f, min_voltage_ + d_voltage * 0.30f));
                data.push_back(FloatPoint(total_power_ * 0.1f, min_voltage_ + d_voltage * 0.01f));
                data.push_back(FloatPoint(0, min_voltage_));
                break;
            case DischargeModel::Inverse:
                eq_deg = 7;
                data.push_back(FloatPoint(total_power_, max_voltage_));
                data.push_back(FloatPoint(total_power_ * 0.9f, min_voltage_ + d_voltage * 0.70f));
                data.push_back(FloatPoint(total_power_ * 0.8f, min_voltage_ + d_voltage * 0.50f));
                data.push_back(FloatPoint(total_power_ * 0.7f, min_voltage_ + d_voltage * 0.30f));
                data.push_back(FloatPoint(total_power_ * 0.6f, min_voltage_ + d_voltage * 0.20f));
                data.push_back(FloatPoint(total_power_ * 0.5f, min_voltage_ + d_voltage * 0.10f));
                data.push_back(FloatPoint(total_power_ * 0.4f, min_voltage_ + d_voltage * 0.05f));
                data.push_back(FloatPoint(total_power_ * 0.3f, min_voltage_ + d_voltage * 0.03f));
                data.push_back(FloatPoint(total_power_ * 0.2f, min_voltage_ + d_voltage * 0.02f));
                data.push_back(FloatPoint(total_power_ * 0.1f, min_voltage_ + d_voltage * 0.01f));
                data.push_back(FloatPoint(0, min_voltage_));
                break;
        }

        eq_.fit(eq_deg, data);
    }

    void replace_battery_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // Reset battery to initial state
            percent_ = initial_percent_;
            power_ = map_number(percent_, 0, 100, 0, total_power_);
            recharging_ = false;
            discharge_rate_ = base_power_consumption_ / (float)(60 * 60 * 1000);
            recharge_rate_ = 0;
            voltage_ = base_voltage_;

            RCLCPP_INFO(this->get_logger(), "Battery replaced. Reset to initial state.");
        }
    }

public:
    BatterySimulator() : Node("battery_simulator")
    {
        // Declare parameters
        this->declare_parameter("discharge_model", "linear");
        this->declare_parameter("max_voltage", 12.5f);
        this->declare_parameter("min_voltage", 11.5f);
        this->declare_parameter("base_voltage", 12.0f);
        this->declare_parameter("initial_percent", 100);
        this->declare_parameter("discharge_current", 7000);
        this->declare_parameter("recharge_current", 2400);
        this->declare_parameter("base_power_consumption", 200);
        this->declare_parameter("motors_power_consumption", 5000);
        this->declare_parameter("num_batteries", 1);
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("verbose", false);

        // Get parameters
        discharge_model_ = this->get_parameter("discharge_model").as_string();
        max_voltage_ = this->get_parameter("max_voltage").as_double();
        min_voltage_ = this->get_parameter("min_voltage").as_double();
        base_voltage_ = this->get_parameter("base_voltage").as_double();
        initial_percent_ = this->get_parameter("initial_percent").as_int();
        discharge_current_ = this->get_parameter("discharge_current").as_int();
        recharge_current_ = this->get_parameter("recharge_current").as_int();
        base_power_consumption_ = this->get_parameter("base_power_consumption").as_int();
        motors_power_consumption_ = this->get_parameter("motors_power_consumption").as_int();
        num_batteries_ = this->get_parameter("num_batteries").as_int();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        verbose_ = this->get_parameter("verbose").as_bool();

        // Validate and fix parameter values (similar to ROS1 code)
        if (discharge_model_ == "exponential")
        {
            model_ = DischargeModel::Exponential;
            model_name_ = "Exponential";
        }
        else if (discharge_model_ == "linear")
        {
            model_ = DischargeModel::Linear;
            model_name_ = "Linear";
        }
        else if (discharge_model_ == "inverse")
        {
            model_ = DischargeModel::Inverse;
            model_name_ = "Inverse";
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid discharge model (%s), setting to default", discharge_model_.c_str());
            model_ = DischargeModel::Linear;
            model_name_ = "Linear";
        }

        // Similar parameter validation as in ROS1 code (omitted for brevity)

        total_discharge_current_ = num_batteries_ * discharge_current_;
        total_power_ = (total_discharge_current_ / 1000.0f) * base_voltage_;

        // Setup discharge model
        setup_discharge_model();

        // Logging initial parameters
        RCLCPP_INFO(this->get_logger(), "Battery model = %s", model_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Max voltage = %fV", max_voltage_);
        // ... other parameter logging

        // Publishers
        power_pub_ = this->create_publisher<std_msgs::msg::Int32>("power", 1);
        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("voltage", 1);
        percent_pub_ = this->create_publisher<std_msgs::msg::Float32>("percent", 1);
        recharging_pub_ = this->create_publisher<std_msgs::msg::Bool>("recharging", 1);

        // Subscribers
        recharge_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "recharge", 1, std::bind(&BatterySimulator::recharge_callback, this, std::placeholders::_1)
        );
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 1, std::bind(&BatterySimulator::cmd_vel_callback, this, std::placeholders::_1)
        );
        replace_battery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "replace_battery", 1, 
            std::bind(&BatterySimulator::replace_battery_callback, this, std::placeholders::_1)
        );

        // Initialize battery state
        percent_ = initial_percent_;
        power_ = map_number(percent_, 0, 100, 0, total_power_);
        recharging_ = false;
        discharge_rate_ = base_power_consumption_ / (float)(60 * 60 * 1000);
        recharge_rate_ = 0;

        // Create a timer to update battery state every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&BatterySimulator::update_battery, this)
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatterySimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}