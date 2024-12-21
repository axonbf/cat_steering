#include <cstdio>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>
#include "rclcpp/timer.hpp"
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/srv/param_set.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <vector>

#ifndef LEFT
#define LEFT 0
#endif

#ifndef RIGHT
#define RIGHT 2
#endif

// Left: 3470 Full speed;  Right: 3445 Full speed
// Dilwe RC F2838 350Kv; 27.April 2020 3000 RPM 
// 11.1 V; No Load Current 0.25A; No Load Speed (rpm) 3950; Max Current 7.5 A; Max Power (w)
// Question 1.-: At which PWM do we arrive 1 m/s. Perhaps due to cavitation and eficiency the properler is not design for the motor Kv
// Question 2.-: What is the speed = Actual speed - Theoretichal speed; Vx_the = rpm/60*pp in [m/s]
// Kv left mean:  303.6356
// Kv right mean: 301.7117

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
          if(str_mode == 0){ //gamepad skid
            // lineal funtin y = m*x + c; if m = 0; y = 1500, therefore c = 1500
            // if x = -1; y = 1050; therefore m = (1050 - 1500)/(-1) = 450;
            // y = 450*x + 1500;
            // from RC_In to Servo remapping ->  RC(1050 - 1950) Servo(1100 - 1950)
            // y = m*x + c; if x = 1500; y = 1500; 
            int16_t x_speed     = (fabs(msg.linear.x )<0.01 ? 0 : msg.linear.x ) * 450 + 1500; //doulbe
            int16_t z_angul_dif = (fabs(msg.angular.z)<0.01 ? 0 : msg.angular.z) * 450;
            //RCLCPP_INFO(this->get_logger(), "x_speed %d; z_angul_dif %d. ", x_speed, z_angul_dif);

            rcOverride.channels[0] = x_speed + z_angul_dif; //1575; / min 1050 - 1950 // steering
            if(rcOverride.channels[0] < 1050) rcOverride.channels[0] = 1050;
            if(rcOverride.channels[0] > 1950) rcOverride.channels[0] = 1950;
            rcOverride.channels[2] = x_speed - z_angul_dif; //1500; / min 1050 - 1950 // speed
            if(rcOverride.channels[2] < 1050) rcOverride.channels[2] = 1050;
            if(rcOverride.channels[2] > 1950) rcOverride.channels[2] = 1950;
            this->rc_override_pub_->publish(rcOverride);
          }
          if(str_mode == 1){ //gamepad paddle
            int16_t rc_left  = (fabs(gamepad.axes[1])<0.01 ? 0 : gamepad.axes[1])*450 + 1500;
            //int16_t rc_right = (fabs(gamepad.axes[4])<0.01 ? 0 : gamepad.axes[4])*450 + 1500;
            int16_t rc_right = (fabs(gamepad.axes[3])<0.01 ? 0 : gamepad.axes[3])*450 + 1500;
            rcOverride.channels[LEFT] = rc_left;
            rcOverride.channels[RIGHT] = rc_right;
            this->rc_override_pub_->publish(rcOverride);
          }
      }
    };

/*
    auto raw_servo_callback =
    [this](mavros_msgs::msg::RCOut msg) -> void{
      this->servo_output->channels[RIGHT] = msg.channels[RIGHT];
      this->servo_output->channels[LEFT]  = msg.channels[LEFT];
      //RCLCPP_INFO(this->get_logger(), "servo_left: %d; servo_right: %d",servo_output->channels[LEFT], servo_output->channels[RIGHT]);
    };
    */

    auto battery_state_callback =
    [this](sensor_msgs::msg::BatteryState msg) ->void{
      //battery_state = msg;
      battery_voltage = msg.voltage;
      //RCLCPP_INFO(this->get_logger(), "Battery Voltage: %f.",battery_state.voltage);
    };

    auto str_timer_callback =
    [this]() -> void{
      //V = [RPM x PP x (1- S) x GR] / 60 
      float_t servo_right_percent = (servo_raw_right-1500)/400.0; // from 1100 to 1900
      float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
      float_t mot_rpm_th_r = Kv_right*battery_voltage*servo_right_percent; //Kv*Voltage;
      float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
      float_t th_Vl = mot_rpm_th_l/60 * prop_p;
      float_t th_Vr = mot_rpm_th_r/60 * prop_p;
      RCLCPP_INFO(this->get_logger(), "Vx_l(Kv*Volt*servo_l): %f(%f*%f*%f); Vx_r(Kv*Volt*servo_r): %f(%f*%f*%f)", th_Vl, Kv_left, battery_voltage, servo_left_percent, th_Vr, Kv_right, battery_voltage, servo_right_percent);

      RCLCPP_INFO(this->get_logger(), "servo_left: %d; servo_right: %d; battery_voltage: %f; rpm_left: %f; rpm_right: %f; th_Vx_l: %f; th_Vx_r: %f; th_curvature_: %f; th_radius_: %f; diameter: %f ",servo_raw_left, servo_raw_right, battery_voltage, th_motor_rpm_left_, th_motor_rpm_right_, th_Vx_l, th_Vx_r, th_curvature_,th_radius_,2*th_radius_);
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
      gamepad = msg;

      if(msg.buttons[5] == 0)
      {
        desired_pilot_steer_type = 2; // ardupilot_steer_steering
      }

      if(msg.buttons[5] == 1)
      {
        desired_pilot_steer_type = 1; // ardupilot_paddle_mode
      }
      if(msg.buttons[7] == 0)
      {
        str_mode = 0;
      }
      if(msg.buttons[7] == 1)
      {
        str_mode = 1;
      }

      if(pilot_steer_button_last != msg.buttons[5])
      {
        RCLCPP_INFO(this->get_logger(), "Gamepad steering state changed from %d to %d. ", pilot_steer_button_last , msg.buttons[5]);
        pilot_steer_button_last = msg.buttons[5];
      }

    };

    // Subscribers
    cmd_vel_sub_       = this->create_subscription<geometry_msgs::msg::Twist     >("cmd_vel",        10, cmd_vel_callback);
    gamepad_           = this->create_subscription<sensor_msgs::msg::Joy         >("joy",            10, joy_callback);
    //raw_servo_sub_     = this->create_subscription<mavros_msgs::msg::RCOut       >("mavros/rc/out",  10, raw_servo_callback);
    raw_servo_sub_     = this->create_subscription<mavros_msgs::msg::RCOut       >("mavros/rc/out",  10, std::bind(&Skid_Steering::raw_servo_callback,this,_1));
    battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("mavros/battery", rclcpp::SensorDataQoS(), battery_state_callback);
    double th_motor_rpm_left(uint16_t rc_in_left);
    double th_motor_rpm_right(uint16_t rc_in_right);
    double th_curvature(void);
    double th_motor_linear_speed_left (void);
    double th_motor_linear_speed_right(void);
    double th_veh_linear_speed(void);
    double th_veh_angular_speed(void);

    // Publishers
    rc_override_pub_  = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override",10);
    th_curvature_pub_ = this->create_publisher<std_msgs::msg::Float64>("cat/th_curvature",10);
    th_Vx_pub_        = this->create_publisher<std_msgs::msg::Float64>("cat/th_Vx",10);
    th_Vl_pub_        = this->create_publisher<std_msgs::msg::Float64>("cat/th_Vl",10);
    th_Vr_pub_        = this->create_publisher<std_msgs::msg::Float64>("cat/th_Vr",10);
    th_w_pub_         = this->create_publisher<std_msgs::msg::Float64>("cat/th_w",10);
    th_radius_pub_    = this->create_publisher<std_msgs::msg::Float64>("cat/th_radius",10);
    th_rpm_left_pub_  = this->create_publisher<std_msgs::msg::Float64>("cat/th_rpm_left",10);
    th_rpm_right_pub_ = this->create_publisher<std_msgs::msg::Float64>("cat/th_rpm_right",10);

    // Services
    client_get_parameters_       = this->create_client<rcl_interfaces::srv::GetParameters>("mavros/param/get_parameters");
    client_set_parameters_       = this->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");

    // Timers
    timer_                       = this->create_wall_timer(1s, timer_callback);
    str_timer_                   = this->create_wall_timer(50ms, str_timer_callback);

  }

  private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_;
  rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr raw_servo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;

  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_curvature_pub_; // th_curvature
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vx_pub_; // th_Vx
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vl_pub_; // th_Vl
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vr_pub_; // th_Vr
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_w_pub_; // th_w
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_radius_pub_; // th_radius
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_rpm_left_pub_; // th_rpm_left
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_rpm_right_pub_; // th_rpm_right
  
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_get_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_set_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr str_timer_;

  bool paddle_mode = false;
  int  str_mode    = 0; // 0 gamepad skid steering; 1 gamepad paddle; 2 speed diff 
  int  desired_pilot_steer_type;
  int  current_pilot_steer_type;
  bool get_parameters_service_available = false;
  bool set_parameters_service_available = false;
  //bool pilot_steer_type_change_rqst = false;
  int  pilot_steer_button_last = 0;
  // Kv left mean:  303.6356
  // Kv right mean: 301.7117
  float_t Kv_left  = 303.6535;// 350;  //
  float_t Kv_right = 301.7117;// 350;  //
  //float V_nom = 11.1; // nominal voltage from Battery LiPo 3S
  //float V_mes = 0.0;

  // PP propeller pitch and diameter
  float_t prop_d = 0.06; // [m] Diameter from the center of to the tip of the propeller 
  float_t prop_p = 0.022; //0.03; // [m] measuring the mark on the motor we have 3.5; // cm from pbo.couk/choose-right-boat-propeller-6205 I take 4x(1/2*1.5cm) = 0.75*4 = 3 cm pitch
  // uint16_t prop_p = 7; // cm according to the formulas atan(1.5/2) = 36.8. Making a triangle where Ca = perimeter using 3 cm  for diameter (the half), Ca will be the pitch
  //V = [RPM x PP x (1- S) x GR] / 60 
  float_t th_Vx_l{};
  float_t th_Vx_r{};

  double_t th_motor_rpm_left_{};
  double_t th_motor_rpm_right_{};

  double_t th_curvature_{};
  double_t th_Vx_{};
  double_t th_w_{};
  double_t th_radius_{};
  double_t th_sleep_ = 1.0;//0.873;
  
  //double_t th_linear_speed_left{};
  //double_t th_linear_speed_rith{};

  //std::vector<uint16_t> servo_output_raw = {0,0,0,0,0,0,0,0};

  //mavros_msgs::msg::RCOut servo_output;
  uint16_t servo_raw_left{1500};
  uint16_t servo_raw_right{1500};
  sensor_msgs::msg::Joy   gamepad;
  //sensor_msgs::msg::BatteryState battery_state;
  float_t battery_voltage;//{};


  bool response_get_pilot_steer_type_received = true;
  bool response_set_pilot_steer_type_received = true;

  float trackWith_ = 0.28; // 28 [cm]

  double th_motor_rpm_left (uint16_t rc_in_left)
  {
    // ToDo -> utils calculate sign
    // ToDo -> map between two lineal function: map(value, low1, high1, low2, high2) -> low2 + (value - low1) * (high2 - low2) / (high1 - low1) https://stackoverflow.com/questions/3451553/value-remapping
    // RC   % = |   1559 |   1560 |   1564 |   1600 |   1700 |   1800 |   1900 |   1950 |
    // rpms % = | 0.0000 | 0.0762 | 0.0907 | 0.3109 | 0.7596 | 0.9508 | 0.9861 | 1.0000 |
    // rpm_left_percent = -1.0753 + 0.03446*x - 0.00038607*x^2 + 2.537e^-6*x^3 - 8.8022e^-9*x^4  + 1.5002e^-11*x^5 -9.9132e^-15*x^6// this is on relation to rc input and normalized for 11.42 volts
    // rpm_left = 4.0973e+08 -1.4509e+06 2.1376e+03 -1.6772e+00  7.3926e-04 -1.7356e-07 1.6958e-11               
    // double x =  rc_in_left - 1500; // RC input, but we should adjust to the row_servo for eachm otor
    // double rpm_left_percent_rc = - 1.0753 + 0.03446*x          - 0.00038607*pow(x,2)
    //                              + 2.5370*pow(10, -6)*pow(x,3) - 8.8022*pow(10, -9)*pow(x,4)  
    //                              + 1.5002*pow(10,-11)*pow(x,5) - 9.9132*pow(10,-15)*pow(x,6);

    // rpm_left_percent = -1.0605e-01 + 3.8462e-03*X + 9.2575e-05*X^2 - 1.0532e-06*X^3 + 4.7222e-09*X^4 - 9.8371e-12*X^5 + 7.8128e-15*X^6 // based on the servo output and normalized for 11.42 volts
     int16_t  x = rc_in_left - 1500;
    uint16_t ux = abs(x);
    double rpm_left_percent = - 0.10605 + 0.0038462*ux       + 9.2575*pow(10, -5)*pow(ux,2)
                              - 1.0532*pow(10, -6)*pow(ux,3) + 4.7222*pow(10, -9)*pow(ux,4)  
                              - 9.8371*pow(10,-12)*pow(ux,5) + 7.8128*pow(10,-15)*pow(ux,6);
    if( (0 < ux ) && (ux < 28) ) rpm_left_percent = 0.0;
    else rpm_left_percent = rpm_left_percent*(x/ux);

    double rpm_left = battery_voltage*Kv_left*rpm_left_percent;
    return rpm_left;
  }

  double th_motor_rpm_right (uint16_t rc_in_right)
  {
    // RC   % = |   1559 |   1560 |   1564 |   1600 |   1700 |   1800 |   1900 |   1950 |
    // rpms % = | 0.0000 | 0.0000 | 0.0874 | 0.3150 | 0.7592 | 0.9563 | 0.9949 | 1.0000 |
    // rpm_right_percent = -0.48545  + 0.0086215*x + 1.2704e^-5x^2 - 2.9581e^-7*x^3 +1.2696e^-9*x^4 - 2.4067e^-12*x^5 + 1.7269e^-15*x^6 // based on rc input and normalized for 11.52volts
    //double rpm_right_percent_rc = - 0.48545 + 0.0086215*x       + 1.2704*pow(10, -5)*pow(x,2) 
    //                              - 2.9581*pow(10, -7)*pow(x,3) + 1.2696*pow(10, -9)*pow(x,4) 
    //                              - 2.4067*pow(10,-12)*pow(x,5) + 1.7269*pow(10,-15)*pow(x,6);

    // rpm_right_percent = -1.2498e-01 + 3.8527e-03*X + 9.9540e-05*X^2 -1.1564e-06*X^3 + 5.2962e-09*X^4 - 1.1200e-11*x^5 + 8.9769e-15*X^6
     int16_t  x = rc_in_right - 1500;
    uint16_t ux = abs(x);
    double rpm_right_percent = - 0.12498 + 0.0038527*ux       + 9.9540*pow(10, -5)*pow(ux,2) 
                               - 1.1564*pow(10, -6)*pow(ux,3) + 5.2962*pow(10, -9)*pow(ux,4) 
                               - 1.1200*pow(10,-11)*pow(ux,5) + 8.9769*pow(10,-15)*pow(ux,6);
    if( (0 < ux ) && (ux < 28) ) rpm_right_percent = 0.0;
    else rpm_right_percent = rpm_right_percent*(x/ux);

    double rpm_right = battery_voltage*Kv_right*rpm_right_percent;
    return rpm_right;
  }

  double th_motor_linear_speed_left(void)
  {
    // V = [RPM x PP x (1- S) x GR] / 60 
    //  float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
    //  th_Vx_l = mot_rpm_th_l/60 * prop_p;
    double linear_speed_left = th_motor_rpm_left_/60*prop_p;
    return linear_speed_left;
  }

  double th_motor_linear_speed_right(void)
  { 
    // V = [RPM x PP x (1- S) x GR] / 60 
    //  float_t servo_right_percent = (servo_raw_right-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_r = Kv_right*battery_voltage*servo_right_percent; //Kv*Voltage;
    //  th_Vx_r = mot_rpm_th_r/60 * prop_p;
    double linear_speed_right = th_motor_rpm_right_/60*prop_p; 
    return linear_speed_right;
  }

  double th_curvature(void)
  {
    // I guess 3cm radius should be enough 3cm radius is 0.03m
    // which is 33333
    double th_curvature = 0.0;
    double th_radius   = 0.0;
    double th_left_speed  = th_motor_rpm_left_;  
    double th_right_speed = th_motor_rpm_right_; 
    if(fabs(th_left_speed) < 0.005)  th_left_speed = 0.0;
    if(fabs(th_right_speed) < 0.005) th_right_speed = 0.0;
    if( fabs(th_left_speed - th_right_speed) == 0.0 )
    {
      th_curvature = 0;

    }
    else
    {
      th_radius = ((th_left_speed*th_sleep_ + th_right_speed*th_sleep_)/2) * (16*trackWith_) / ( (th_left_speed*th_sleep_-th_right_speed*th_sleep_)/2);
      th_curvature = 1000/th_radius;
      if(th_curvature >  33333) th_curvature = 33333;
      if(th_curvature < -33333) th_curvature = 33333;
      th_radius = 1000/th_curvature;
    }
    th_radius_ = th_radius;
    return th_curvature;
  }

  double th_veh_linear_speed(void)
  {
    return (th_Vx_l + th_Vx_r)/2;
  }

  double th_veh_angular_speed(void)
  {
    return (th_Vx_r - th_Vx_l)/trackWith_; // ToDo check (th_Vx_r - th_Vx_l)/(trackWicth_/2);
    // which should be the same as: steering_rate = ( (th_Vx_r + th_Vx_l)/2 )*trackWidth_)/ th_radius;
    // 
  }

  void raw_servo_callback (const mavros_msgs::msg::RCOut msg) 
  {
    servo_raw_left  = msg.channels[LEFT];
    servo_raw_right = msg.channels[RIGHT];
    //RCLCPP_INFO(this->get_logger(), "servo_left: %d; servo_right: %d",servo_raw_left, servo_raw_right);
    th_motor_rpm_left_  = th_motor_rpm_left (servo_raw_left);
    th_motor_rpm_right_ = th_motor_rpm_right(servo_raw_right);
    this->th_rpm_left_pub_ ->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_motor_rpm_left_ ));
    this->th_rpm_right_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_motor_rpm_right_));

    th_Vx_l = th_motor_linear_speed_left ();
    th_Vx_r = th_motor_linear_speed_right();
    this->th_Vl_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_l));
    this->th_Vr_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_r));

    th_curvature_ = th_curvature();
    this->th_curvature_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_curvature_));
    this->th_radius_pub_   ->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_radius_)   );

    th_Vx_ = th_veh_linear_speed();
    th_w_  = th_veh_angular_speed();
    this->th_Vx_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_));
    this->th_w_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_w_));
  }

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

  printf("good bye world from cat_steering package\n");
  return 0;
}
