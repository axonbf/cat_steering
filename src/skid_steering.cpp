#include <cstdio>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>
#include "rclcpp/timer.hpp"
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/srv/param_set.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
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
///mavros/vfr_hud    -> air speed/ ground speed
///mavros/gps1/raw   -> vel or epv

//angular speed:
///mavros/local_position/odom
///mavros/local_position/velocity_body
///mavros/imu/data

// Demos: https://github.com/ros2/demos/blob/06f25e9c8801ea95a0c25260cb38f2f0c6af3ea0/demo_nodes_cpp/src/services/add_two_ints_client_async.cpp#L62-L73

// ToDos:
// [ ] Finish and tests KvAsParameter
// [ ] merge to main
// [ ] update the correct equations for angular speed using the paper of skid steerin
// [ ] clean up the code and repetions and calls and what is theorerital and what is measured etc

using std::placeholders::_1;
using namespace std::chrono_literals;

class Skid_Steering : public rclcpp::Node
{
  public:

  Skid_Steering(): Node("skid_steering")
  {

    // parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "this is the Kv of the left motor. Meaning Kv*Voltage = motor rpms!";
    this->declare_parameter("Kv_left", 303.6535, param_desc);
    param_desc.description = "this is the Kv of the right motor. Meaning Kv*Voltage = motor rpms!";
    this->declare_parameter("Kv_right", 301.7117, param_desc);
    param_desc.description = "This is the desired slip for the speed to adapt the linear speed of the boat";
    this->declare_parameter("Th_slip", 0.132, param_desc); // th_slip_ 0.12
    param_desc.description = "This is the propeller pitch Prop_p";
    this->declare_parameter("Prop_p", 0.0218, param_desc); // float_t prop_p = 0.022; //0.03; // [m] measuring the mark on the motor we have 3.5; // cm from pbo.couk/choose-right-boat-propeller-6205 I take 4x(1/2*1.5cm) = 0.75*4 = 3 cm pitch
    param_desc.description = "This is the propeller diameter Prop_d";
    this->declare_parameter("Prop_d", 0.06, param_desc); // prop_d              = 0.06; // [m] Diameter from the center of to the tip of the propeller 
    param_desc.description = "This is the minimum voltate, this is used to calculate the rpms and the max speed ";
    this->declare_parameter("Min_voltage", 11.15, param_desc); // min_voltage_        = 11.15;
    param_desc.description = "This is the maximum percent used for the motor. This can also be used to use more efficently up to 70 or 85 percent of the motor ";
    this->declare_parameter("Motor_limit", 0.98, param_desc); // motor_limit_        = 0.98;
    param_desc.description = "This is the instantaneus center of radious coeficient for a skid-steering, which in theory dependes of the left and right speeds. It is 1 when no splippage occurs (ideal differential drive)";
    this->declare_parameter("ICR_x", 1.0, param_desc); 
    param_desc.description = "Track width of the boat, or distance between the hulls";
    this->declare_parameter("Track_width", 0.28, param_desc); // float trackWith_ = 0.28; // 28 [cm]
    param_desc.description = "Drag factor to be used for the calculation of the linear speed as Vx = Vl + Vr / (2*Drag_factor). It should never be 0.0";
    this->declare_parameter("Drag_factor", 1.0, param_desc); 

    auto cmd_vel_callback =
    [this](geometry_msgs::msg::Twist msg) -> void{

      if(paddle_mode){
        mavros_msgs::msg::OverrideRCIn rcOverride;
          if(str_mode == 0){ //gamepad skid
            // ****lineal funtin y = m*x + c; if m = 0; y = 1500, therefore c = 1500
            // if x = -1; y = 1050; therefore m = (1050 - 1500)/(-1) = 450;
            // y = 450*x + 1500;
            // from RC_In to Servo remapping ->  RC(1050 - 1950) Servo(1100 - 1950)
            // y = m*x + c; if x = 1500; y = 1500; 

            set_differential_steering_skid(msg.linear.x*max_linear_speed_, msg.angular.z*max_angular_speed_); 
          }
          if(str_mode == 1){ //gamepad paddle
            uint16_t rc_left  = (uint16_t)((fabs(gamepad.axes[1])<0.01 ? 0 : gamepad.axes[1])*450 + 1500);
            //int16_t rc_right = (fabs(gamepad.axes[4])<0.01 ? 0 : gamepad.axes[4])*450 + 1500;
            uint16_t rc_right = (uint16_t)((fabs(gamepad.axes[3])<0.01 ? 0 : gamepad.axes[3])*450 + 1500);
            rcOverride.channels[LEFT] = rc_left;
            rcOverride.channels[RIGHT] = rc_right;
            this->rc_override_pub_->publish(rcOverride);
          }
      }
    };

    auto battery_state_callback =
    [this](sensor_msgs::msg::BatteryState msg) ->void{
      //battery_voltage = msg.voltage; 
      battery_voltage = 11.61; // FixME -> for simulation to our talbe lest use 11.
      //RCLCPP_INFO(this->get_logger(), "Battery Voltage: %f.",battery_state.voltage);
    };

    auto str_timer_callback =
    [this]() -> void{
      //V = [RPM x PP x (1- S) x GR] / 60 
      /**/
      //float_t servo_right_percent = (servo_raw_right-1500)/400.0; // from 1100 to 1900
      //float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
      //float_t mot_rpm_th_r = Kv_right*battery_voltage*servo_right_percent; //Kv*Voltage;
      //float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
      //float_t th_Vl = mot_rpm_th_l/60 * prop_p;
      //float_t th_Vr = mot_rpm_th_r/60 * prop_p;
      
      //
      //RCLCPP_INFO(this->get_logger(), "Vx_l(Kv*Volt*servo_l): %f(%f*%f*%f); Vx_r(Kv*Volt*servo_r): %f(%f*%f*%f)", th_Vl, Kv_left, battery_voltage, servo_left_percent, th_Vr, Kv_right, battery_voltage, servo_right_percent);
      //RCLCPP_INFO(this->get_logger(), "servo_left: %d; servo_right: %d; battery_voltage: %f; rpm_left: %f; rpm_right: %f; th_Vx_l: %f; th_Vx_r: %f; th_curvature_: %f; th_radius_: %f; diameter: %f ",servo_raw_left, servo_raw_right, battery_voltage, th_motor_rpm_left_, th_motor_rpm_right_, th_Vx_l, th_Vx_r, th_curvature_,th_radius_,2*th_radius_);
      /**/
    };

    auto timer_callback =
    [this]() -> void {
    //RCLCPP_INFO(this->get_logger(),"[timer 01]");
      if(!get_parameters_service_available)
      {
        while(!client_get_parameters_->wait_for_service(1s)){
          if (!rclcpp::ok()){
           RCLCPP_ERROR(
             this->get_logger(), "[timer_callback get_param] Client interrumped while waiting for service %s. Terminating...",client_get_parameters_->get_service_name());
           return;
          }
         RCLCPP_INFO(this->get_logger(), "[timer_callback get_param] Service '%s' Unavailable. Waiting for service... ", std::string(client_get_parameters_->get_service_name()).c_str());
        }
        RCLCPP_INFO(this->get_logger(), "[timer_callback get_param] Service '%s' ON-LINE. ", client_get_parameters_->get_service_name());
        get_parameters_service_available = true;
        response_get_pilot_steer_type_received = false;
        init_values();
        call_async_get_pilot_steer_type(); 
      }

      if(!set_parameters_service_available)
      {
        while(!client_set_parameters_->wait_for_service(1s)){
          if (!rclcpp::ok()){
           RCLCPP_ERROR(
             this->get_logger(), "[timer_callback set_param] Client interrumped while waiting for service %s. Terminating... ",client_set_parameters_->get_service_name());
           return;
          }
         RCLCPP_INFO(this->get_logger(), "[timer_callback set_param] Service '%s' Unavailable. Waiting for service... ", std::string(client_set_parameters_->get_service_name()).c_str());
        }
        RCLCPP_INFO(this->get_logger(), "[timer_callback set_param] Service '%s' ON-LINE. ", client_set_parameters_->get_service_name());
        set_parameters_service_available = true;
      }

      if (desired_pilot_steer_type != current_pilot_steer_type && response_get_pilot_steer_type_received == true)
      {
        RCLCPP_INFO(this->get_logger(), "[timer_callback] desired_pilot_steer_type '%s' different than current '%s'. ", to_string(desired_pilot_steer_type).c_str(), to_string(current_pilot_steer_type).c_str());
        if( response_set_pilot_steer_type_received == true)
        {
          response_set_pilot_steer_type_received = false;
          call_async_set_manual_mode(); // Setting manual mode
          call_async_set_pilot_steer_type(desired_pilot_steer_type); //***
        }
      }
    };

    auto joy_callback = 
    [this] (sensor_msgs::msg::Joy msg) -> void{
      //RCLCPP_INFO(this->get_logger(),"joy_callback");
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
        RCLCPP_INFO(this->get_logger(), "[joy_callback] Gamepad steering state changed from %d to %d. ", pilot_steer_button_last , msg.buttons[5]);
        pilot_steer_button_last = msg.buttons[5];
      }

    };

    // Subscribers
    cmd_vel_sub_       = this->create_subscription<geometry_msgs::msg::Twist     >("cmd_vel",        10, cmd_vel_callback);
    gamepad_           = this->create_subscription<sensor_msgs::msg::Joy         >("joy",            10, joy_callback);
    ////raw_servo_sub_     = this->create_subscription<mavros_msgs::msg::RCOut   >("mavros/rc/out",  10, raw_servo_callback);
    raw_servo_sub_     = this->create_subscription<mavros_msgs::msg::RCOut       >("mavros/rc/out",  10,                      std::bind(&Skid_Steering::raw_servo_callback,this,_1));
    battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("mavros/battery", rclcpp::SensorDataQoS(), battery_state_callback);
    //vfr_sub_           = this->create_subscription<mavros_msgs::msg::VfrHud    >("/mavros/vfr_hud",rclcpp::SensorDataQoS(), vfr_callback);
    vfr_sub_           = this->create_subscription<mavros_msgs::msg::VfrHud      >("/mavros/vfr_hud",rclcpp::SensorDataQoS(), std::bind(&Skid_Steering::vfr_callback      ,this,_1));
    imu_data_          = this->create_subscription<sensor_msgs::msg::Imu         >("/mavros/imu/data",rclcpp::SensorDataQoS(),std::bind(&Skid_Steering::imu_data_callback,this,_1));

    //general functions
    void init_values(void);
    double_t servo2percent_left(uint16_t rc_in_left);
    double_t percent2rpm_left(double_t rpm_left_percent);
    double_t rpm2speed_left (double_t rpm_left);
    ///////
    double_t speed2rpm_left(double_t speed_left);
    double_t rpm2percent_left(double_t rpm_left);
    double_t percent2servo_left(double_t rpm_left_percent);
    uint16_t servo2rc_left(double_t servo_left); // since RC is the output that I am sending e.g. rc=1950

    double_t servo2percent_right(uint16_t rc_in_right);     // and servo is what a read, e.g. servo=1900 and rc=1950
    double_t percent2rpm_right(double_t rpm_right_percent);
    double_t rpm2speed_right(double_t rpm_right);
    ///////
    double_t speed2rpm_right(double_t speed_right);
    double_t rpm2percent_right(double_t rpm_right);
    double_t percent2servo_right(double_t rpm_right_percent);
    uint16_t servo2rc_right(double_t servo_right); // since RC is the output that I am sending e.g. rc=1950

    //uint16_t rc2servo_right(double_t rc_right);

    double_t th_curvature(void);
    //double_t th_motor_linear_speed_left (void);

    double_t th_veh_linear_speed(void);
    double_t th_veh_angular_speed(void);
    void     set_differential_steering_skid(double_t linear_speed, double_t angular_speed);
    void         set_speed_and_radious_skid(double_t linear_speed, double_t radious);
    uint16_t left_speed_2_rc(double_t left_speed);
    uint16_t rith_speed_2_rc(double_t righ_speed);

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
    slip_pub_         = this->create_publisher<std_msgs::msg::Float64>("cat/slip",10);
    // desired values
    set_Vl_pub_        = this->create_publisher<std_msgs::msg::Float64>("cat/set_Vl",10);
    set_Vr_pub_        = this->create_publisher<std_msgs::msg::Float64>("cat/set_Vr",10);
    is_radius_pub_     = this->create_publisher<std_msgs::msg::Float64>("cat/is_radius",10);
    is_curvature_pub_  = this->create_publisher<std_msgs::msg::Float64>("cat/is_curvature",10);

    // Services
    client_get_parameters_       = this->create_client<rcl_interfaces::srv::GetParameters>("mavros/param/get_parameters");
    client_set_parameters_       = this->create_client<rcl_interfaces::srv::SetParameters>("mavros/param/set_parameters");

    client_set_mode_             = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Timers
    timer_                       = this->create_wall_timer(1s, timer_callback);
    str_timer_                   = this->create_wall_timer(50ms, str_timer_callback);
  

  }

  private:


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_;
  rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr raw_servo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::VfrHud>::SharedPtr vfr_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_;

  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_curvature_pub_; // th_curvature
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vx_pub_; // th_Vx
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vl_pub_; // th_Vl
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_Vr_pub_; // th_Vr
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_w_pub_; // th_w
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_radius_pub_; // th_radius
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_rpm_left_pub_; // th_rpm_left
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr th_rpm_right_pub_; // th_rpm_right
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_pub_; 
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_Vl_pub_; // set_Vl
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr set_Vr_pub_; // set_Vr
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr is_radius_pub_; // is_radius
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr is_curvature_pub_; // is_curvature
  
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_get_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_set_parameters_; // td_srvs::srv::SetBoll>::SharedPtr client_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_set_mode_; 

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


  // PP propeller pitch and diameter
  // uint16_t prop_p = 7; // cm according to the formulas atan(1.5/2) = 36.8. Making a triangle where Ca = perimeter using 3 cm  for diameter (the half), Ca will be the pitch
  // V = [RPM x PP x (1- S) x GR] / 60 
  //float_t prop_d = 0.06; // [m] Diameter from the center of to the tip of the propeller 
  //float_t prop_p = 0.022; //0.03; // [m] measuring the mark on the motor we have 3.5; // cm from pbo.couk/choose-right-boat-propeller-6205 I take 4x(1/2*1.5cm) = 0.75*4 = 3 cm pitch
  //float_t min_voltage_ = 11.15;
  float_t slip_{}; // real time measured slip
  //float_t motor_limit_ = 0.98;

  float_t max_linear_speed_ {};
  float_t max_angular_speed_{}; 

  float_t th_Vx_l{};
  float_t th_Vx_r{};

  double_t th_motor_rpm_left_{};
  double_t th_motor_rpm_right_{};

  double_t th_curvature_{};
  double_t is_curvature_{};

  double_t th_Vx_{};
  float_t ground_speed_{};

  double_t th_w_{};
  double_t yaw_speed_{};

  double_t th_radius_{};
  double_t is_radius_{};
  
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

  //float trackWith_ = 0.28; // 28 [cm]

  void init_values(void)
  {
    th_curvature_       = 0.0;
    th_Vx_              = 0.0;
    th_w_               = 0.0;
    th_radius_          = 0.0;
    th_motor_rpm_left_  = 0.0;
    th_motor_rpm_right_ = 0.0;
    slip_               = 0.0;
    //prop_d              = 0.06; // [m] Diameter from the center of to the tip of the propeller 
    //prop_p              = 0.022; //0.03; // [m] measuring the mark on the motor we have 3.5; // cm from pbo.couk/choose-right-boat-propeller-6205 I take 4x(1/2*1.5cm) = 0.75*4 = 3 cm pitch
    //min_voltage_        = 11.15;
    slip_               = 0.0; // real time measured slip
    //motor_limit_        = 0.98;

    RCLCPP_INFO(this->get_logger(), "Init values: ");
    RCLCPP_INFO(this->get_logger(), "Kv_left: %f; Kv_right: %f", this->get_parameter("Kv_left").as_double(), this->get_parameter("Kv_right").as_double());
    max_linear_speed_   = (this->get_parameter("Kv_left").as_double() 
                          > this->get_parameter("Kv_right").as_double() 
                          ? this->get_parameter("Kv_right").as_double()*this->get_parameter("Min_voltage").as_double() /60*( (1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() ) 
                          : this->get_parameter("Kv_left").as_double()* this->get_parameter("Min_voltage").as_double() /60*(  (1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() ));
    max_angular_speed_  = 2*max_linear_speed_/this->get_parameter("Track_width").as_double() ; // 2*Vx/radius FixME, I am producing up to 5 times the linear speed

  }

  double_t servo2percent_left (uint16_t rc_in_left)
  {
    // ToDo -> utils calculate sign
    // RC   % = |   1559 |   1560 |   1564 |   1600 |   1700 |   1800 |   1900 |   1950 |
    // rpms % = | 0.0000 | 0.0762 | 0.0907 | 0.3109 | 0.7596 | 0.9508 | 0.9861 | 1.0000 |
    int16_t  x = rc_in_left - 1500;
    uint16_t ux = abs(x);
    // rpm_right_percetn = -5.832325307760911e-14   4.920713129609684e-11   1.029277435759187e-08  -2.430456929159689e-05   9.378150682378419e-03  -1.867135238096345e-01
    double_t rpm_left_percent = -5.832325307760911*pow(10,-14)*pow(ux,5)+4.920713129609684*pow(10,-11)*pow(ux,4)+1.029277435759187*pow(10,-8)*pow(ux,3)-2.430456929159689*pow(10,-5)*pow(ux,2)+9.378150682378419*pow(10,-3)*pow(ux,1)-1.867135238096345*pow(10,-1);
    if( ((0 < ux ) && (ux < 28)) || (ux == 0) ) rpm_left_percent = 0.0;
    else rpm_left_percent = rpm_left_percent*(double_t)(x/ux);

    //double_t rpm_left = battery_voltage*Kv_left*rpm_left_percent;
    //RCLCPP_INFO(this->get_logger(), "rc_in_left: %d; rpm_percent_left: %f; rpm_left: %f",ux,rpm_left_percent,rpm_left );
    //RCLCPP_INFO(this->get_logger(), " ux: %d; 5.5110e-21*x^9: %f;  -1.0048e-17*x^8: %f;  7.6943e-15*x^7: %f;  -3.2069e-12*x^6: %f;   7.8690e-10*x^5: %f;  -1.1463e-07*x^4: %f;   9.5073e-06*x^3: %f;  -4.2487e-04*x^2: %f;   1.6236e-02*x^1: %f;  -2.1602e-01: %f;",
    //xx,5.5110*pow(10,-21)*pow(xx,9),-1.0048*pow(10,-17)*pow(xx,8),7.6943*pow(10,-15)*pow(xx,7),-3.2069*pow(10,-12)*pow(xx,6),+7.8690*pow(10,-10)*pow(xx,5),-1.1463*pow(10,-7)*pow(xx,4),+9.5073*pow(10,-6)*pow(xx,3),-4.2487*pow(10,-4)*pow(xx,2),+1.6236*pow(10,-2)*pow(xx,1),-2.1602*pow(10,-1));
    //return rpm_left;
    return rpm_left_percent;
  }

  double_t percent2rpm_left(double_t rpm_left_percent)
  {
      return battery_voltage*this->get_parameter("Kv_left").as_double()*rpm_left_percent;
  }

  double_t rpm2speed_left(double_t rpm_left)
  {
    // V = [RPM x PP x (1- S) x GR] / 60 
    // V = ( Kv * Volgate * DuttyCycle * PP * (1-slipage) * Gear ) / 60
    //  float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
    //  th_Vx_l = mot_rpm_th_l/60 * prop_p;
    //double_t linear_speed_left = th_motor_rpm_left_/60*prop_p; 
    //Note rpm_left = th_motor_rpm_left_, the second is put in to the function to be more clear
    //double_t linear_speed_left = rpm_left/60*prop_p;
    double_t linear_speed_left = rpm_left/60*((1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() );
    return linear_speed_left;
  }

  double_t speed2rpm_left(double_t speed_left)
  {
    // V = [RPM x PP x (1- S) x GR] / 60 
    // V = ( Kv * Volgate * DuttyCycle * PP * (1-slipage) * Gear ) / 60
    //  float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
    //  th_Vx_l = mot_rpm_th_l/60 * prop_p;
    //double_t linear_speed_left = th_motor_rpm_left_/60*prop_p;
    //double_t linear_speed_left = rpm_left/60*prop_p;
    double_t rpm_left{};
    if( fabs( this->get_parameter("Prop_p").as_double() ) < 0.001) rpm_left = 0.0;
    else rpm_left = speed_left*60/((1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() );
    return rpm_left;
  }

  double_t rpm2percent_left(double_t rpm_left)
  {
    //double_t rpm_left = battery_voltage*Kv_left*rpm_left_percent;
    //if ( fabs(battery_voltage*Kv_left) < 0.001) return 0.0;
    //else return (rpm_left/ (battery_voltage*Kv_left));

      if ( fabs(battery_voltage*(this->get_parameter("Kv_left").as_double())) < 0.001) return 0.0;
      else return (rpm_left/ (battery_voltage*this->get_parameter("Kv_left").as_double()));
  }

  double_t percent2servo_left(double_t rpm_left_percent)
  {


    // curve calculation
    double_t  x = rpm_left_percent;
    double_t ux = fabs(x);
    double_t sign = x/ux;
    uint16_t servo_left{};
    double_t deadZone = 0.05;
    uint16_t y{};
    //if( ux > 0.994){ ux = 0.994;} // RCLCPP_INFO(this->get_logger(),"************ux is bigger");} /// creo que 0.988 es el mayor
    if( ux > this->get_parameter("Motor_limit").as_double()){ ux = this->get_parameter("Motor_limit").as_double();} // RCLCPP_INFO(this->get_logger(),"************ux is bigger");} /// creo que 0.988 es el mayor
    if( ux < deadZone) servo_left = 1500;
    else
    {
      bool rc_found = false;
      //RCLCPP_INFO(this->get_logger(),"MAX value: %f; unsignd: %f",x,ux);
      for(uint16_t i = 1 ; i<= 400; i++)
      {
        double_t rpm_left_percent = -5.832325307760911*pow(10,-14)*pow(i,5)+4.920713129609684*pow(10,-11)*pow(i,4)+1.029277435759187*pow(10,-8)*pow(i,3)-2.430456929159689*pow(10,-5)*pow(i,2)+9.378150682378419*pow(10,-3)*pow(i,1)-1.867135238096345*pow(10,-1);
        //RCLCPP_INFO(this->get_logger(),"rpm_left_percent found: %f at RC: %d; bigger than: %f",rpm_left_percent,i,ux);
        if( (rpm_left_percent >= ux)   && !rc_found )
        {
          //RCLCPP_INFO(this->get_logger(),"$$$$$$$$$$$$$$$$4 rpm_left_percent found: %f at RC: %d; bigger than: %f",rpm_left_percent,i,ux);
          y = i;
          rc_found = true;
        }
      }
      servo_left = (uint16_t)(1500 + (sign*y));
      //RCLCPP_INFO(this->get_logger()," y to be used: %d; and servo_left: %d",y,servo_left);
    }

    return servo_left;
  }

  uint16_t servo2rc_left(double_t servo_left)
  {

    double_t low1  = 1100;
    double_t high1 = 1900;
    double_t low2  = 1050;
    double_t high2  = 1950;
    uint16_t rc_out{};

    rc_out = (uint16_t)(low2 + (servo_left - low1) * (high2 - low2) / (high1 - low1)); // or rc_left_linear
    //**** checa aqui ToDo */
    RCLCPP_INFO(this->get_logger(),"[servo2rc_left] servo_left: %f; rc_out: %d; ", servo_left, rc_out);
    if(rc_out < low2)  rc_out = low2;
    if(rc_out > high2) rc_out = high2;

    return rc_out;
  }

  /// @brief This function reprecents the basic percen fo the servo input 
  /// @param rc_in_right 
  /// @return rpm_right_percent
  double_t servo2percent_right (uint16_t rc_in_right)
  {
    // RC   % = |   1559 |   1560 |   1564 |   1600 |   1700 |   1800 |   1900 |   1950 |
    // rpms % = | 0.0000 | 0.0000 | 0.0874 | 0.3150 | 0.7592 | 0.9563 | 0.9949 | 1.0000 |
    int16_t   x = rc_in_right - 1500;
    uint16_t ux = abs(x);
    // pp_r_6 = 3.942613160784577e-14  -7.547772093367737e-11   6.600020035107555e-08  -3.477702363777966e-05   1.020791539102076e-02  -2.176141967499420e-01
    double_t rpm_right_percent = 3.942613160784577*pow(10,-14)*pow(ux,5)-7.547772093367737*pow(10,-11)*pow(ux,4)+6.600020035107555*pow(10,-8)*pow(ux,3)-3.477702363777966*pow(10,-5)*pow(ux,2)+1.020791539102076*pow(10,-2)*pow(ux,1)-2.176141967499420*pow(10,-1);
    if( ((0 < ux ) && (ux < 28)) || (ux == 0) ) rpm_right_percent = 0.0;
    else rpm_right_percent = rpm_right_percent*(x/ux);

    //double_t rpm_right = battery_voltage*Kv_right*rpm_right_percent;
    //RCLCPP_INFO(this->get_logger(), "rc_in_right: %d; rpm_percent_right: %f; rpm_right: %f",ux,rpm_right_percent,rpm_right );
    //return rpm_right;
    return rpm_right_percent;
  }

  double_t percent2rpm_right(double_t rpm_right_percent)
  {
      return battery_voltage*this->get_parameter("Kv_right").as_double()*rpm_right_percent;
  }

  /// @brief Converts the rpms to a percentage of the total possible rpms of the motor base on the dutty cycle. 
  /// rpm = (Voltage*percent)*Kv
  /// @param rpm_right 
  /// @return percetage 
  double_t rpm2percent_right(double_t rpm_right)
  {
    //double_t rpm_left = battery_voltage*Kv_left*rpm_left_percent;
      if ( fabs(battery_voltage*this->get_parameter("Kv_right").as_double()  ) < 0.001) return 0.0;
      else return (rpm_right/ (battery_voltage*this->get_parameter("Kv_right").as_double() ));
  }

  double_t percent2servo_right(double_t rpm_right_percent)
  {
    // curve calculation
    // xp_r_6 = -2.3627e+09   1.3064e+10  -3.0647e+10   3.9561e+10  -3.0432e+10   1.4069e+10  -3.7221e+09   4.9110e+08  -2.2070e+07   2.4000e+01
    double_t  x = rpm_right_percent;
    double_t ux = fabs(x);
    double_t sign = x/ux;
    uint16_t servo_right{};
    double_t deadZone = 0.05;
    uint16_t y{};
    if( ux > this->get_parameter("Motor_limit").as_double()){ ux = this->get_parameter("Motor_limit").as_double() ;} 
    if( ux < deadZone) servo_right = 1500;
    else
    {
      bool rc_found = false;
      for(uint16_t i = 1 ; i<= 400; i++)
      {
        double_t rpm_right_percent = 3.942613160784577*pow(10,-14)*pow(i,5)-7.547772093367737*pow(10,-11)*pow(i,4)+6.600020035107555*pow(10,-8)*pow(i,3)-3.477702363777966*pow(10,-5)*pow(i,2)+1.020791539102076*pow(10,-2)*pow(i,1)-2.176141967499420*pow(10,-1);
        if( (rpm_right_percent >= ux) && !rc_found)
        {
          //RCLCPP_INFO(this->get_logger(),"rpm_right_percent found: %f at RC: %d",rpm_right_percent,i);
          y = i;
          rc_found = true;
        }
      }
      servo_right = (uint16_t)(1500 + ((sign)*y));
    }

    return servo_right;
  }

  uint16_t servo2rc_right(double_t servo_right)
  {
    // linear map
    double_t low1  = 1100;// -1*max_value;
    double_t high1 = 1900;// max_value;
    double_t low2  = 1050;
    double_t high2  = 1950;
    uint16_t rc_out{};

    rc_out = (uint16_t)(low2 + (servo_right - low1) * (high2 - low2) / (high1 - low1)); // or rc_right_linear
    if(rc_out < low2)  rc_out = low2;
    if(rc_out > high2) rc_out = high2;

    return rc_out;
  }

  double_t rpm2speed_right(double_t rpm_right)
  { 
    // V = [RPM x PP x (1- S) x GR] / 60 
    //  float_t servo_right_percent = (servo_raw_right-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_r = Kv_right*battery_voltage*servo_right_percent; //Kv*Voltage;
    //  th_Vx_r = mot_rpm_th_r/60 * prop_p;
    double_t linear_speed_right = rpm_right/60*((1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() ); 
    //double_t linear_speed_right = rpm_right/60*prop_p; 
    return linear_speed_right;
  }

  double_t speed2rpm_right(double_t speed_right)  
  {
    // V = [RPM x PP x (1- S) x GR] / 60 
    // V = ( Kv * Volgate * DuttyCycle * PP * (1-slipage) * Gear ) / 60
    //  float_t servo_left_percent  = (servo_raw_left-1500)/400.0; // from 1100 to 1900
    //  float_t mot_rpm_th_l = Kv_left*battery_voltage*servo_left_percent;
    //  th_Vx_l = mot_rpm_th_l/60 * prop_p;
    //double_t linear_speed_left = th_motor_rpm_left_/60*prop_p;
    //double_t linear_speed_left = rpm_left/60*prop_p;
    double_t rpm_right{};
    if( fabs( this->get_parameter("Prop_p").as_double() ) < 0.001) rpm_right = 0.0;
    else rpm_right = speed_right*60/((1-this->get_parameter("Th_slip").as_double())* this->get_parameter("Prop_p").as_double() );
    return rpm_right;
  }

  // FixME -> use v = r*omega, and than transform the radius
  double_t th_curvature(void)
  {
    // I guess 3cm radius should be enough 3cm radius is 0.03m
    // which is 33333
    double_t th_curvature = 0.0;
    double_t th_radius   = 0.0;
    double_t th_left_speed  = th_motor_rpm_left_;  
    double_t th_right_speed = th_motor_rpm_right_; 
    if(fabs(th_left_speed) < 0.005)  th_left_speed = 0.0;
    if(fabs(th_right_speed) < 0.005) th_right_speed = 0.0;
    if( fabs(th_left_speed - th_right_speed) == 0.0 )
    {
      th_curvature = 0;

    }
    else
    {
      // FixME: check if the radious and curvature calculation is the same everywhere
      //th_radius = ((th_left_speed*th_sleep_ + th_right_speed*th_sleep_)/2) * (16*trackWith_) / ( (th_left_speed*th_sleep_-th_right_speed*th_sleep_)/2);
      th_radius = ((th_left_speed + th_right_speed)/2) * (16* this->get_parameter("Track_width").as_double() ) / ( (th_left_speed-th_right_speed)/2);
      th_curvature = 1000/th_radius;
      if(th_curvature >  33333) th_curvature = 33333;
      if(th_curvature < -33333) th_curvature = 33333;
      th_radius = 1000/th_curvature;
    }
    // FixME what is the proper place to assign values to the variables?
    th_radius_ = th_radius;
    return th_curvature;
  }

  double_t th_veh_linear_speed(void)
  {
    return (th_Vx_l + th_Vx_r)/ (2*  this->get_parameter("Drag_factor").as_double() );
  }

  double_t th_veh_angular_speed(void)
  {
    // Cosidering that the radious is equivlent to the center of mass of the vehicle, 
    // this will correspond to the trackWidth/2, since the track width is the distance between the wheels
    // return (th_Vx_r - th_Vx_l)/(trackWicth_/2);

    // According to "Analysis and Experimental Kinematics of a Skid-Steering Wheeled Robot Based on a Laser Scanner Sensor"
    // omega_z = (-th_Vx_l + th_Vx_r)/(2*yo), where yo is the instantaneous center of rotation (ICR), which is different from the track width
    // lambda = (th_Vx_l + th_Vx_r)/(-th_Vx_l + th_Vx_r)
    // ICR_x = (th_Vx_l + th_Vx_r)/track_with = 2*yo/trackWidth_2 // asumming Vl and Vr are simetrical

    // Thereofre  wz = (-vl + vr)/(2*B*ICR_x); where ICR_x is related to the sleepage, and B is the track width
    // see equations 46 and 47
    return (th_Vx_r - th_Vx_l)/(2* this->get_parameter("Drag_factor").as_double()* this->get_parameter("Track_width").as_double() *this->get_parameter("ICR_x").as_double()); // ToDo change for a lookup table
  }
  
  void set_speed_and_radious_skid(double_t linear_speed, double_t radious)
  {
    double_t angular_speed = linear_speed/radious;
    set_differential_steering_skid(linear_speed, angular_speed);
  }

  void set_differential_steering_skid(double_t desired_linear_speed, double_t desired_angular_speed)
  {
    // Vx = (Vr + Vl)/2; 
    // Vr = Vx*2 - Vl
    // Vl = Vx*2 - Vr
    // dV = Vx*2
    
    // update linear speed -- really necesary?
    //double_t Vx   = (th_Vx_l + th_Vx_r)/2;

    double_t dV_e = desired_linear_speed - th_veh_linear_speed();//Vx;
    double_t new_Vx_l = th_Vx_l + dV_e;
    double_t new_Vx_r = th_Vx_r + dV_e;

    // update turning speed
    //double_t isYawRate = (new_Vx_l - new_Vx_r);

    // **ToDo both methosds are the same, but the second one is quicker calculating. This will will increase the speed of the system in smaller steps
    double_t dYawRate_e = desired_angular_speed - th_veh_angular_speed();//isYawRate;
    double_t new_speed_left  = new_Vx_l + dYawRate_e/2.0;
    double_t new_speed_right = new_Vx_r - dYawRate_e/2.0;
    //**RCLCPP_INFO(this->get_logger(), "Thiemos: max_seed: %f; motor_limit_: %f; new_speed_left: %f; new_speed_right: %f, th_Vx_l: %f, th_Vx_r: %f", max_speed_, motor_limit_, new_speed_left, new_speed_right, th_Vx_l, th_Vx_r);

    // alternative write directly the new value:
    // Step 01 apply linear speed 
    //new_Vx_l = linear_speed;
    //new_Vx_r = linear_speed;

    // Step 02 update the turning rate
    new_speed_left  = desired_linear_speed + desired_angular_speed/2;
    new_speed_right = desired_linear_speed - desired_angular_speed/2; 
    //**RCLCPP_INFO(this->get_logger(), "Benjami: max_speed: %f; motor_limit_: %f; new_speed_left: %f; new_speed_right: %f", max_speed_, motor_limit_, new_speed_left, new_speed_right);
    // when is the track with used?
    uint16_t rc_left, rc_right;
    mavros_msgs::msg::OverrideRCIn rcOverride;

    // Error is that I am producing a percent based on the max speed allowed. That should not be the case.
    // I should calculate the rpms, and if then if the go above the allowed values, then set a limit
    // alternatively, I should calculate up to the rc and set the max value, 
    // I could also set the max at the sepeed,....
    //***rc_left  =  left_speed_2_rc(new_speed_left,  max_speed_);
    //***rc_right = right_speed_2_rc(new_speed_right, max_speed_);
    if(new_speed_left  >  max_linear_speed_) new_speed_left  =  max_linear_speed_;
    if(new_speed_left  < -max_linear_speed_) new_speed_left  = -max_linear_speed_;
    if(new_speed_right >  max_linear_speed_) new_speed_right =  max_linear_speed_;
    if(new_speed_right < -max_linear_speed_) new_speed_right = -max_linear_speed_;

    // ToDo publish the desired speed
    this->set_Vl_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(new_speed_left));
    this->set_Vr_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(new_speed_right));

    rc_left  = servo2rc_left( percent2servo_left (rpm2percent_left( speed2rpm_left (new_speed_left)))); 
    rc_right = servo2rc_right(percent2servo_right(rpm2percent_right(speed2rpm_right(new_speed_right))));


    rcOverride.channels[LEFT] = rc_left;
    rcOverride.channels[RIGHT] = rc_right;

    RCLCPP_INFO(this->get_logger(), "[raw_servo_callback]:             Readed  Servo_left: %d; Servo_right: %d,   is_speed_left: %f;   is_speed_right: %f",servo_raw_left, servo_raw_right,        th_Vx_l,         th_Vx_r);
    RCLCPP_INFO(this->get_logger(), "[set_differential_steering_skid]: Desired    RC_left: %d;    RC_right: %d, soll_speed_left: %f; soll_speed_right: %f",       rc_left,        rc_right, new_speed_left, new_speed_right);
    this->rc_override_pub_->publish(rcOverride);
  }

  uint16_t left_speed_2_rc(double_t left_speed, double_t max_value)
  {// the right sade begins to rotat at 29. do ideally both should be bigger thatn 29, or both should be 0 whne less than 29 which is 0.05 from 1.0
   // lineal remap 
   // ToDo make a function for a linear converstion if needed
    double_t low1  = -1*max_value;
    double_t high1 = max_value;
    double_t low2  = 1050;
    double_t high2  = 1950;
    uint16_t rc_left_linear{};
    rc_left_linear = (uint16_t)(low2 + (left_speed - low1) * (high2 - low2) / (high1 - low1)); // or rc_left_linear

    // curve calculation
    double_t  x = left_speed/max_value;
    double_t ux = fabs(left_speed/max_value);
    double_t sign = x/ux;
    uint16_t rc_left{};
    double_t deadZone = 0.05;
    uint16_t y{};
    //if( ux > 0.994){ ux = 0.994;} // RCLCPP_INFO(this->get_logger(),"************ux is bigger");} /// creo que 0.988 es el mayor
    if( ux > this->get_parameter("Motor_limit").as_double() ){ ux = this->get_parameter("Motor_limit").as_double() ;} // RCLCPP_INFO(this->get_logger(),"************ux is bigger");} /// creo que 0.988 es el mayor
    if( ux < deadZone) rc_left = 1500;
    else
    {
      bool rc_found = false;
      //RCLCPP_INFO(this->get_logger(),"MAX value: %f; unsignd: %f",x,ux);
      for(uint16_t i = 1 ; i<= 400; i++)
      {
        double_t rpm_left_percent = -5.832325307760911*pow(10,-14)*pow(i,5)+4.920713129609684*pow(10,-11)*pow(i,4)+1.029277435759187*pow(10,-8)*pow(i,3)-2.430456929159689*pow(10,-5)*pow(i,2)+9.378150682378419*pow(10,-3)*pow(i,1)-1.867135238096345*pow(10,-1);
        //RCLCPP_INFO(this->get_logger(),"rpm_left_percent found: %f at RC: %d; bigger than: %f",rpm_left_percent,i,ux);
        if( (rpm_left_percent >= ux)   && !rc_found )
        {
          //RCLCPP_INFO(this->get_logger(),"$$$$$$$$$$$$$$$$4 rpm_left_percent found: %f at RC: %d; bigger than: %f",rpm_left_percent,i,ux);
          y = i;
          rc_found = true;
        }
      }
      rc_left = (uint16_t)(1500 + (sign*y));
      //RCLCPP_INFO(this->get_logger()," y to be used: %d; and rc_left: %d",y,rc_left);
    }
    low1  = 1100;
    high1 = 1900;
    uint16_t rc_out{};

    rc_out = (uint16_t)(low2 + (rc_left - low1) * (high2 - low2) / (high1 - low1)); // or rc_left_linear
    //**** chieca aqui ToDo */
    RCLCPP_INFO(this->get_logger()," rc_left: %d; rc_out: %d; desired speed: %f", rc_left, rc_out,left_speed);
    if(rc_out < low2)  rc_out = low2;
    if(rc_out > high2) rc_out = high2;

    return rc_out;
  }

  uint16_t right_speed_2_rc(double_t right_speed, double_t max_value)
  {
    // linear map
    double_t low1  = -1*max_value;
    double_t high1 = max_value;
    double_t low2  = 1050;
    double_t high2  = 1950;
    uint16_t rc_right_linear{};
    rc_right_linear = (uint16_t)(low2 + (right_speed - low1) * (high2 - low2) / (high1 - low1)); // or rc_left_linear
    
    // curve calculation
    // xp_r_6 = -2.3627e+09   1.3064e+10  -3.0647e+10   3.9561e+10  -3.0432e+10   1.4069e+10  -3.7221e+09   4.9110e+08  -2.2070e+07   2.4000e+01
    double_t  x = right_speed/max_value;
    double_t ux = fabs(right_speed/max_value);
    double_t sign = x/ux;
    uint16_t rc_right{};
    double_t deadZone = 0.05;
    uint16_t y{};
    if( ux > this->get_parameter("Motor_limit").as_double() ){ ux = this->get_parameter("Motor_limit").as_double();} 
    if( ux < deadZone) rc_right = 1500;
    else
    {
      bool rc_found = false;
      for(uint16_t i = 1 ; i<= 400; i++)
      {
        double_t rpm_right_percent = 3.942613160784577*pow(10,-14)*pow(i,5)-7.547772093367737*pow(10,-11)*pow(i,4)+6.600020035107555*pow(10,-8)*pow(i,3)-3.477702363777966*pow(10,-5)*pow(i,2)+1.020791539102076*pow(10,-2)*pow(i,1)-2.176141967499420*pow(10,-1);
        if( (rpm_right_percent >= ux) && !rc_found)
        {
          //RCLCPP_INFO(this->get_logger(),"rpm_right_percent found: %f at RC: %d",rpm_right_percent,i);
          y = i;
          rc_found = true;
        }
      }
      rc_right = (uint16_t)(1500 + ((sign)*y));
    }
    low1  = 1100;
    high1 = 1900;
    uint16_t rc_out{};

    rc_out = (uint16_t)(low2 + (rc_right - low1) * (high2 - low2) / (high1 - low1)); // or rc_right_linear
    if(rc_out < low2)  rc_out = low2;
    if(rc_out > high2) rc_out = high2;

    return rc_out;
  }

  void raw_servo_callback (const mavros_msgs::msg::RCOut msg) 
  {
    servo_raw_left  = msg.channels[LEFT];
    servo_raw_right = msg.channels[RIGHT];
    //**RCLCPP_INFO(this->get_logger(), "[raw_servo_callback]:             Received RC_Out_left: %d; RC_Out_right: %d",servo_raw_left, servo_raw_right);
    //**RCLCPP_INFO(this->get_logger(), "[set_differential_steering_skid]: Desired  RC_Out_left: %d; RC_Out_right: %d", rc_left, rc_right);
    //th_motor_rpm_left_  = servo2percent_left (servo_raw_left);
    //th_motor_rpm_right_ = servo2percent_right(servo_raw_right);
    th_motor_rpm_left_  = percent2rpm_left(servo2percent_left (servo_raw_left));
    th_motor_rpm_right_ = percent2rpm_right(servo2percent_right(servo_raw_right));

    this->th_rpm_left_pub_ ->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_motor_rpm_left_ ));
    this->th_rpm_right_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_motor_rpm_right_));

    th_Vx_l = rpm2speed_left (th_motor_rpm_left_);
    th_Vx_r = rpm2speed_right(th_motor_rpm_right_);

    this->th_Vl_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_l));
    this->th_Vr_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_r));

    th_Vx_ = th_veh_linear_speed();
    th_w_  = th_veh_angular_speed();
    this->th_Vx_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_Vx_));
    this->th_w_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_w_));

    //th_curvature_ = th_curvature(); FixME: not used, erase the function or do it differntly
    if(fabs(th_w_) > 0.00001)
    {
      th_radius_    = 999999.9;
    }
    else
    {
      // v = w*r
      th_radius_    = th_Vx_/th_w_;
    }
    th_curvature_ = 1000/th_radius_;

    this->th_curvature_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_curvature_));
    this->th_radius_pub_   ->publish(std_msgs::build<std_msgs::msg::Float64>().data(th_radius_)   );

  }

  void vfr_callback( const mavros_msgs::msg::VfrHud msg)
  {
    ground_speed_ = msg.groundspeed;
    slip_ = (th_Vx_ - ground_speed_)/th_Vx_;
    this->slip_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(slip_));

    //RCLCPP_INFO(this->get_logger(),"th_Vx: %f; ground_seed: %f; slip: %f",th_Vx_,ground_speed_,slip_);
  }
  
  void imu_data_callback( const sensor_msgs::msg::Imu msg)
  {
    yaw_speed_    = msg.angular_velocity.z;
    // v = w*r
    if(fabs(yaw_speed_) < 0.00001) is_radius_ = 999999.9;
    else
      is_radius_ = ground_speed_/yaw_speed_;

    is_curvature_ = 1000/is_radius_;

    this->is_curvature_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(is_curvature_));
    this->is_radius_pub_->publish(std_msgs::build<std_msgs::msg::Float64>().data(is_radius_));
  }

  void call_async_get_pilot_steer_type()
  {
    while(!client_get_parameters_->wait_for_service(1s)){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(), "[call_async_get_pilot_steer_type] Client interrumped while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "[call_async_get_pilot_steer_type] Service '%s' Unavailable. Waiting for service... ", std::string(client_get_parameters_->get_service_name()).c_str());
    }
    RCLCPP_INFO(this->get_logger(), "[call_async_get_pilot_steer_type] Service '%s' will be called async... ", client_get_parameters_->get_service_name());

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("PILOT_STEER_TYPE");  // ToDo, perhaps first ask the type of values, and check if PILOT_STEER_TYPE exist
    RCLCPP_INFO(this->get_logger(), "[call_async_get_pilot_steer_type] Sending request to check PILOT_STEER_TYPE ");

    auto result = client_get_parameters_->async_send_request(request, std::bind(&Skid_Steering::get_parameters_response_received_callback, this, std::placeholders::_1));
  }

  void get_parameters_response_received_callback (rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) 
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "[get_parameters_response_received_callback] Result arrived... size: %ld, type: %o, value: %ld (%s). ", result->values.size(), result->values[0].type, result->values[0].integer_value,to_string(result->values[0].integer_value).c_str());
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
          this->get_logger(), "[call_async_set_pilot_steer_type] Client interrumped while waiting for service %s. Terminating...",client_set_parameters_->get_service_name());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "[call_async_set_pilot_steer_type] Service '%s' Unavailable. Waiting for service... ", std::string(client_set_parameters_->get_service_name()).c_str());
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter _desired_pilot_steer_type;
    _desired_pilot_steer_type.name = "PILOT_STEER_TYPE";
    _desired_pilot_steer_type.value.type = 2;
    _desired_pilot_steer_type.value.integer_value = _type; 

    request->parameters.push_back(_desired_pilot_steer_type);

    RCLCPP_INFO(this->get_logger(), "[call_async_set_pilot_steer_type] Service '%s' will be called async to change PILOT_STEER_TYPE from '%s' to '%s'... ", client_set_parameters_->get_service_name(), to_string(current_pilot_steer_type).c_str(), to_string(desired_pilot_steer_type).c_str());

    auto result = client_set_parameters_->async_send_request(request, std::bind(&Skid_Steering::set_parameters_response_received_callback, this, std::placeholders::_1));

  }

  void call_async_set_manual_mode()
  {
    while(!client_set_mode_->wait_for_service(1s)){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(
          this->get_logger(), "[call_async_set_manual_mode] Client interrumped while waiting for service %s. Terminating...",client_set_mode_->get_service_name());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "[call_async_set_manual_mode] Service '%s' Unavailable. Waiting for service... ", std::string(client_set_mode_->get_service_name()).c_str());
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "MANUAL";

    RCLCPP_INFO(this->get_logger(), "[call_async_set_manual_mode] Service '%s' will be called async to change mode to MANUAL... ", client_set_mode_->get_service_name());
    auto result = client_set_mode_->async_send_request(request, std::bind(&Skid_Steering::set_manual_mode_response_callback,          this, std::placeholders::_1));
  }

  void set_manual_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "[set_manual_mode_response_callback] sesult arrived with success: %d. ", result->mode_sent);
  }

  void set_parameters_response_received_callback (rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) 
  {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "[set_parameter_response_received_callback] sesult arrived with success: %d, reason: '%s'. ", result->results[0].successful, result->results[0].reason.c_str());
    response_set_pilot_steer_type_received = true;

    response_get_pilot_steer_type_received = false;
    call_async_get_pilot_steer_type();
  }

  //void set_mode_response_received_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
  //{
  //  auto result = future.get();
  //  RCLCPP_INFO(this->get_logger(), "[set_mode_response_received_callback] sesult arrived with success: %d, reason: '%s'. ", result->results[0].successful, result->results[0].reason.c_str());
  //}


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
