#include <cstdio>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>
#include "rclcpp/timer.hpp"
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mavros_msgs/srv/param_set.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>


// Demos: https://github.com/ros2/demos/blob/06f25e9c8801ea95a0c25260cb38f2f0c6af3ea0/demo_nodes_cpp/src/services/add_two_ints_client_async.cpp#L62-L73

using std::placeholders::_1;
using namespace std::chrono_literals;

class Skid_Steering : public rclcpp::Node
{
  public:

  Skid_Steering(): Node("skid_steering")
  {

    auto cmd_vel_callback =
    [this](geometry_msgs::msg::Twist msg) -> void{

      if(paddle_mode){
        mavros_msgs::msg::OverrideRCIn rcOverride;

        // lineal funtin y = m*x + c; if m = 0; y = 1500, therefore c = 1500
        // if x = -1; y = 1050; therefore m = (1050 - 1500)/(-1) = 450;
        // y = 450*x + 1500;
        int16_t x_speed     = msg.linear.x * 450 + 1500; //doulbe
        int16_t z_angul_dif = msg.angular.z * 450;
        RCLCPP_INFO(this->get_logger(), "x_speed %d; z_angul_dif %d. ", x_speed, z_angul_dif);

        rcOverride.channels[0] = x_speed + z_angul_dif; //1575; / min 1050 - 1950 // steering
        if(rcOverride.channels[0] < 1050) rcOverride.channels[0] = 1050;
        if(rcOverride.channels[0] > 1950) rcOverride.channels[0] = 1950;
        rcOverride.channels[2] = x_speed - z_angul_dif; //1500; / min 1050 - 1950 // speed
        if(rcOverride.channels[2] < 1050) rcOverride.channels[2] = 1050;
        if(rcOverride.channels[2] > 1950) rcOverride.channels[2] = 1950;
        this->publisher_->publish(rcOverride);
      }

    };

    auto timer_callback =
    [this]() -> void {
      if(!get_parameters_service_available){
        while(!client_get_parameters_->wait_for_service(1s)){
          if (!rclcpp::ok()){
           RCLCPP_ERROR(
             this->get_logger(), "Client interrumped while waiting for service %s. Terminating...",client_get_parameters_->get_service_name());
           return;
          }
         RCLCPP_INFO(this->get_logger(), "Service '%s' Unavailable. Waiting for service... ", std::string(client_get_parameters_->get_service_name()).c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Service '%s' ON-LINE. ", client_get_parameters_->get_service_name());
        get_parameters_service_available = true;
        response_get_pilot_steer_type_received = false;
        call_async_get_pilot_steer_type();
      }

      if(!set_parameters_service_available){
        while(!client_set_parameters_->wait_for_service(1s)){
          if (!rclcpp::ok()){
           RCLCPP_ERROR(
             this->get_logger(), "Client interrumped while waiting for service %s. Terminating... ",client_set_parameters_->get_service_name());
           return;
          }
         RCLCPP_INFO(this->get_logger(), "Service '%s' Unavailable. Waiting for service... ", std::string(client_set_parameters_->get_service_name()).c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Service '%s' ON-LINE. ", client_set_parameters_->get_service_name());
        set_parameters_service_available = true;
      }

      if (desired_pilot_steer_type != current_pilot_steer_type && response_get_pilot_steer_type_received == true)
      {
        RCLCPP_INFO(this->get_logger(), "desired_pilot_steer_type '%s' different than current '%s'. ", to_string(desired_pilot_steer_type).c_str(), to_string(current_pilot_steer_type).c_str());
        if( response_set_pilot_steer_type_received == true)
        {
          response_set_pilot_steer_type_received = false;
          call_async_set_pilot_steer_type(desired_pilot_steer_type);
        }
      }
    };

    auto joy_callback = 
    [this] (sensor_msgs::msg::Joy msg) -> void{

      if(msg.buttons[5] == 0)
      {
        desired_pilot_steer_type = 2; // ardupilot_steer_steering
      }

      if(msg.buttons[5] == 1)
      {
        desired_pilot_steer_type = 1; // ardupilot_paddle_mode
      }

      if(pilot_steer_button_last != msg.buttons[5])
      {
        RCLCPP_INFO(this->get_logger(), "Gamepad steering state changed from %d to %d. ", pilot_steer_button_last , msg.buttons[5]);
        pilot_steer_button_last = msg.buttons[5];
      }

    };

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, cmd_vel_callback);
    gamepad_      = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
    publisher_    = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override",10);

    client_get_parameters_       = this->create_client<rcl_interfaces::srv::GetParameters>("mavros/param/get_parameters");
    client_set_parameters_       = this->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");
    timer_        = this->create_wall_timer(3s, timer_callback);

  }

  private:
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_;
  
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_get_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_set_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool paddle_mode = false;
  int  desired_pilot_steer_type;
  int  current_pilot_steer_type;
  bool get_parameters_service_available = false;
  bool set_parameters_service_available = false;
  //bool pilot_steer_type_change_rqst = false;
  int  pilot_steer_button_last = 0;

  bool response_get_pilot_steer_type_received = true;
  bool response_set_pilot_steer_type_received = true;

  void call_async_get_pilot_steer_type()
  {
    while(!client_get_parameters_->wait_for_service(1s)){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(), "Client interrumped while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service '%s' Unavailable. Waiting for service... ", std::string(client_get_parameters_->get_service_name()).c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Service '%s' will be called async... ", client_get_parameters_->get_service_name());

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("PILOT_STEER_TYPE");  // ToDo, perhaps first ask the type of values, and check if PILOT_STEER_TYPE exist
    RCLCPP_INFO(this->get_logger(), "Sending request to check PILOT_STEER_TYPE ");

    auto result = client_get_parameters_->async_send_request(request, std::bind(&Skid_Steering::get_parameters_response_received_callback, this, std::placeholders::_1));
  }

  void get_parameters_response_received_callback (rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) 
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Result arrived... size: %ld, type: %o, value: %ld (%s). ", result->values.size(), result->values[0].type, result->values[0].integer_value,to_string(result->values[0].integer_value).c_str());
    current_pilot_steer_type = result->values[0].integer_value;
    response_get_pilot_steer_type_received = true;
    if(current_pilot_steer_type == 1)
    {
      paddle_mode = true;
    }else{
      paddle_mode = false;
    }
  }

  void call_async_set_pilot_steer_type(int _type)
  {
    while(!client_set_parameters_->wait_for_service(1s)){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(), "Client interrumped while waiting for service %s. Terminating...",client_set_parameters_->get_service_name());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service '%s' Unavailable. Waiting for service... ", std::string(client_set_parameters_->get_service_name()).c_str());
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter _desired_pilot_steer_type;
    _desired_pilot_steer_type.name = "PILOT_STEER_TYPE";
    _desired_pilot_steer_type.value.type = 2;
    _desired_pilot_steer_type.value.integer_value = _type; 

    request->parameters.push_back(_desired_pilot_steer_type);

    RCLCPP_INFO(this->get_logger(), "Service '%s' will be called async to change PILOT_STEER_TYPE from '%s' to '%s'... ", client_set_parameters_->get_service_name(), to_string(current_pilot_steer_type).c_str(), to_string(desired_pilot_steer_type).c_str());

    auto result = client_set_parameters_->async_send_request(request, std::bind(&Skid_Steering::set_parameters_response_received_callback, this, std::placeholders::_1));

  }

  void set_parameters_response_received_callback (rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) 
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "set_parameters sesult arrived with success: %d, reason: '%s'. ", result->results[0].successful, result->results[0].reason.c_str());
    response_set_pilot_steer_type_received = true;

    response_get_pilot_steer_type_received = false;
    call_async_get_pilot_steer_type();
  }

  std::string const & to_string(int _type)
  {
    switch(_type)
    {
      case 0:
      {
        static std::string str = "Default";
        return str;
      }
      case 1:
      {
        static std::string str = "Two paddles Input";
        return str;
      }
      case 2:
      {
        static std::string str = "Direction reversed when backing up";
        return str;
      }
      case 3:
      {
        static std::string str = "Direction unchanged when backing up";
        return str;
      }
      default:
      {
        static std::string str = "unknown";
        return str;
      }
    }
  }

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Skid_Steering>());
  rclcpp::shutdown();

  printf("hello world cat_steering package\n");
  return 0;
}
