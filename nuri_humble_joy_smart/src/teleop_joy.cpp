#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "nuri_humble_joy/teleop_joy.hpp"
#include "nurirobot_msgs/msg/hc_control.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED
#define M_PI 3.14159265358979323846

struct TeleopJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  // void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string &which_map);
  void cb_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cb_frontl(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_frontr(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_left(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_right(const sensor_msgs::msg::Range::SharedPtr msg);

  void cb_rleft(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_rright(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_bleft(const sensor_msgs::msg::Range::SharedPtr msg);
  void cb_bright(const sensor_msgs::msg::Range::SharedPtr msg);
  // void cb_back(const sensor_msgs::msg::Range::SharedPtr msg);
  // void cb_bottom(const sensor_msgs::msg::Range::SharedPtr msg);

  void cb_hc(const nurirobot_msgs::msg::HCControl::SharedPtr msg);

  void laser_scan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  void check_stop();
  void sendControlOn();
  double gen_profile(double v_ref, double vout, double dt = 0.12, double amax = 0.8);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_raw;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_fl;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_fr;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_l;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_r;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_rl;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_rr;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_bl;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_br;
  // rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_bk;
  // rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_bt;

  rclcpp::Subscription<nurirobot_msgs::msg::HCControl>::SharedPtr sub_hc;

  TeleopJoy *node_;

  void cb_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TwistStamped twist_;

  double target_linear_ = 0.0, target_angular_ = 0.0;
  double current_linear_ = 0.0, current_angular_ = 0.0;
  double easeInSine(double value);
  std::chrono::steady_clock::time_point last_time_;
  bool haveSameSign(double a, double b);

  double max_fwd_vel = 1.0;
  double max_rev_m_s = 0.4;
  double max_deg_s = 0.3;

  /*
    max_fwd_vel = 2.4;
  max_rev_m_s = 2.0;
  max_deg_s = 1.5;

  */

  bool chk_forward = false;
  bool chk_left_side = false;
  bool chk_right_side = false;

  double cos_theta_ = std::cos(M_PI / 2); // cos(90°)
  double sin_theta_ = std::sin(M_PI / 2);
  int width = 1080;
  int height = 2102;
  int centerX = 1080 / 2 - 100;
  int centerY = 2102 / 2 - 194;
  double fow = 0;
  double lastfow = 0;
  double rot = 0;
  double forw_prof = 0;

  bool chk_ul_fl = false;
  bool chk_ul_fr = false;
  bool chk_ul_l = false;
  bool chk_ul_r = false;

  bool chk_ul_rl = false;
  bool chk_ul_rr = false;
  bool chk_ul_bl = false;
  bool chk_ul_br = false;    
  // bool chk_ul_bt = false;
  // bool chk_ul_bk = false;

  bool first = true;
  std::vector<geometry_msgs::msg::Point> points_;
  int speedstep = 1;

  double kf_state_fl = 0.0; // 칼만 필터 상태 변수 (front left)
  double kf_state_fr = 0.0; // 칼만 필터 상태 변수 (front right)
  double kf_state_left = 0.0;
  double kf_state_right = 0.0;
  double kf_state_rleft = 0.0;
  double kf_state_rright = 0.0;
  double kf_state_bleft = 0.0;
  double kf_state_bright = 0.0;    
  // double kf_state_back = 0.0;
  // double kf_state_bottom = 0.0;

  // 필터 매개변수
  double kf_process_variance = 0.01; // 프로세스 분산
  double kf_measurement_variance = 0.1; // 측정 분산
  double kf_estimated_error = 1.0; // 추정 오차  
};

TeleopJoy::TeleopJoy(const rclcpp::NodeOptions &options) : Node("nuri_joy_node", options)
{
  pimpl_ = new Impl;
  pimpl_->node_ = this;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("diffbot_base_controller/cmd_vel", 10);
  pimpl_->pub_raw = this->create_publisher<std_msgs::msg::ByteMultiArray>("mc_rawdata", 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("hc/joy_smart", rclcpp::QoS(10),
                                                                     std::bind(&TeleopJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10),
                                                                            std::bind(&TeleopJoy::Impl::cb_scan, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_fl = this->create_subscription<sensor_msgs::msg::Range>(
      "Front_Left", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_frontl, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_fr = this->create_subscription<sensor_msgs::msg::Range>(
      "Front_Right", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_frontr, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_l = this->create_subscription<sensor_msgs::msg::Range>(
      "FrontSide_Left", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_left, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_r = this->create_subscription<sensor_msgs::msg::Range>(
      "FrontSide_Right", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_right, this->pimpl_, std::placeholders::_1));

  pimpl_->sub_rl = this->create_subscription<sensor_msgs::msg::Range>(
      "RearSide_Left", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_rleft, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_rr = this->create_subscription<sensor_msgs::msg::Range>(
      "RearSide_Right", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_rright, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_bl = this->create_subscription<sensor_msgs::msg::Range>(
      "Rear_Left", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_bleft, this->pimpl_, std::placeholders::_1));
  pimpl_->sub_br = this->create_subscription<sensor_msgs::msg::Range>(
      "Rear_Right", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_bright, this->pimpl_, std::placeholders::_1));      
  // pimpl_->sub_bk = this->create_subscription<sensor_msgs::msg::Range>(
  //     "back", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_back, this->pimpl_, std::placeholders::_1));
  // pimpl_->sub_bt = this->create_subscription<sensor_msgs::msg::Range>(
  //     "bottom", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_bottom, this->pimpl_, std::placeholders::_1));

  pimpl_->sub_hc = this->create_subscription<nurirobot_msgs::msg::HCControl>(
      "hc/control", rclcpp::QoS(10), std::bind(&TeleopJoy::Impl::cb_hc, this->pimpl_, std::placeholders::_1));

  // max_fwd_vel = 2.4;
  // max_rev_m_s = 2.0;
  // max_deg_s = 1.5;

  pimpl_->twist_ = geometry_msgs::msg::TwistStamped();
  pimpl_->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TeleopJoy::Impl::cb_timer, this->pimpl_));
}

TeleopJoy::~TeleopJoy()
{
  delete pimpl_;
}

void TeleopJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // sendCmdVelMsg(joy_msg, "normal");
  if (first)
  {
    first = false;
    sendControlOn();
  }

  fow = joy_msg->axes[1];
  rot = joy_msg->axes[0] * -1;

  if ((fow >= 0 && fow <= 0.05) || (fow < 0 && fow >= -0.05)) {
    if ((rot >= 0 && rot <= 0.05) || (rot < 0 && rot >= -0.05)) {
      fow = 0;
    }
  }

  if ((rot >= 0 && rot <= 0.05) || (rot < 0 && rot >= -0.05)) {
    if ((fow >= 0 && fow <= 0.05) || (fow < 0 && fow >= -0.05)) {
      rot = 0;  
    }
  }
}

void TeleopJoy::Impl::cb_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_scan_to_points(msg);
  check_stop();
}

void TeleopJoy::Impl::cb_frontl(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_fl;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_fl = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_fl < 1.0) {
    chk_ul_fl = true;
  } else {
    chk_ul_fl = false;
  }  
}

void TeleopJoy::Impl::cb_frontr(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_fr;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_fr = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_fr < 0.5) {
    chk_ul_fr = true;
  } else {
    chk_ul_fr = false;
  }  
}

// void TeleopJoy::Impl::cb_back(const sensor_msgs::msg::Range::SharedPtr msg)
// {
//   double priori_estimate = kf_state_back;
//   double priori_error = kf_estimated_error + kf_process_variance;
//   double blending_factor = priori_error / (priori_error + kf_measurement_variance);
//   double residual = msg->range - priori_estimate;
//   double corrected_estimate = priori_estimate + blending_factor * residual;
//   kf_state_back = corrected_estimate;
//   kf_estimated_error = (1 - blending_factor) * priori_error;

//   if (kf_state_back < 0.25) {
//     chk_ul_bk = true;
//   } else {
//     chk_ul_bk = false;
//   }   
//   // printf("back===================================%f\n", msg->range );
// }

void TeleopJoy::Impl::cb_right(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_right;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_right = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_right < 0.5) {
    chk_ul_r = true;
  } else {
    chk_ul_r = false;
  }     
  // printf("cb_right===================================%f %d\n", msg->range, chk_ul_r);
}

void TeleopJoy::Impl::cb_left(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_left;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_left = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_left < 0.5) {
    chk_ul_l = true;
  } else {
    chk_ul_l = false;
  }       
  // printf("cb_left===================================%f %d\n", msg->range, chk_ul_l );
}

// void TeleopJoy::Impl::cb_bottom(const sensor_msgs::msg::Range::SharedPtr msg)
// {
//   double priori_estimate = kf_state_bottom;
//   double priori_error = kf_estimated_error + kf_process_variance;
//   double blending_factor = priori_error / (priori_error + kf_measurement_variance);
//   double residual = msg->range - priori_estimate;
//   double corrected_estimate = priori_estimate + blending_factor * residual;
//   kf_state_bottom = corrected_estimate;
//   kf_estimated_error = (1 - blending_factor) * priori_error;

//   if (kf_state_bottom > 0.25) {
//     chk_ul_bt = true;
//   } else {
//     chk_ul_bt = false;
//   }      

// }


void TeleopJoy::Impl::cb_rleft(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_rleft;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_rleft = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_rleft < 0.5) {
    chk_ul_rl= true;
  } else {
    chk_ul_rl = false;
  }       
  // printf("cb_left===================================%f %d\n", msg->range, chk_ul_l );
}

void TeleopJoy::Impl::cb_rright(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_rright;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_rright = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_rright < 0.5) {
    chk_ul_rr = true;
  } else {
    chk_ul_rr = false;
  }     
  // printf("cb_right===================================%f %d\n", msg->range, chk_ul_r);
}

void TeleopJoy::Impl::cb_bleft(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_bleft;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_bleft = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_bleft < 0.5) {
    chk_ul_bl = true;
  } else {
    chk_ul_bl = false;
  }       
  // printf("cb_left===================================%f %d\n", msg->range, chk_ul_l );
}

void TeleopJoy::Impl::cb_bright(const sensor_msgs::msg::Range::SharedPtr msg)
{
  double priori_estimate = kf_state_right;
  double priori_error = kf_estimated_error + kf_process_variance;
  double blending_factor = priori_error / (priori_error + kf_measurement_variance);
  double residual = msg->range - priori_estimate;
  double corrected_estimate = priori_estimate + blending_factor * residual;
  kf_state_right = corrected_estimate;
  kf_estimated_error = (1 - blending_factor) * priori_error;

  if (kf_state_right < 0.5) {
    chk_ul_br = true;
  } else {
    chk_ul_br = false;
  }     
  // printf("cb_right===================================%f %d\n", msg->range, chk_ul_r);
}

void TeleopJoy::Impl::cb_hc(const nurirobot_msgs::msg::HCControl::SharedPtr msg)
{
  speedstep = msg->speed;

  // switch (speedstep)
  // {
  // case 1:
  //   /* code */
  //   max_fwd_vel = 1.0;
  //   max_rev_m_s = 0.8;
  //   max_deg_s = 1.0;
  //   break;
  // case 2:
  //   /* code */
  //   max_fwd_vel = 1.4;
  //   max_rev_m_s = 1.2;
  //   max_deg_s = 1.2;
  //   break;
  // case 3:
  //   /* code */
  //   max_fwd_vel = 1.8;
  //   max_rev_m_s = 1.6;
  //   max_deg_s = 1.5;
  //   break;
  // case 4:
  //   /* code */
  //   max_fwd_vel = 2.4;
  //   max_rev_m_s = 2.0;
  //   max_deg_s = 1.5;
  //   break;
  // case 5:
  //   /* code */
  //   max_fwd_vel = 2.8;
  //   max_rev_m_s = 2.0;
  //   max_deg_s = 1.5;
  //   break;
  // default:
  //   max_fwd_vel = 1.0;
  //   max_rev_m_s = 0.8;
  //   max_deg_s = 1.0;
  //   break;  
  // }
}

void TeleopJoy::Impl::cb_timer()
{

  // printf("fow : %f, rot : %f %s %s\n", fow, rot, chk_ul_fl ? "true": "false", chk_ul_fr ? "true": "false");
  printf("chk_ul_l: %s chk_ul_r: %s chk_left_side:%s chk_right_side:%s chk_forward:%s\n", 
  chk_ul_l ? "true": "false", 
  chk_ul_r ? "true": "false", 
  chk_left_side ? "true": "false", 
  chk_right_side ? "true": "false",
  chk_forward ? "true": "false");
  if ((lastfow > 0 && fow < 0) || (lastfow < 0 && fow > 0))
  {
    fow = 0;
  }

  if (fow > 0 && !(chk_ul_fl || chk_ul_fr))
  {
    fow = 0;
  }

  if (fow < 0 && !(chk_ul_bl || chk_ul_br))
  {
    fow = 0;
  }

  if (rot > 0 && !(chk_ul_l || chk_ul_rl))
  {
    rot = 0;
  }

  if (rot < 0 && !(chk_ul_r || chk_ul_rr))
  {
    rot = 0;
  }


  lastfow = fow;

  if (fow != 0)
  {
    forw_prof = gen_profile(fow, forw_prof);
  }
  else
  {
    forw_prof = 0;
  }

  if (chk_forward && fow > 0)
  {
    forw_prof = 0;
  }

  if ((chk_left_side && rot > 0) || (chk_right_side && rot < 0))
  {
    rot = 0;
  }


  twist_.header.stamp = node_->now();

  twist_.twist.linear.x = (forw_prof >= 0) ? forw_prof * max_fwd_vel : forw_prof * max_rev_m_s;
  twist_.twist.angular.z = rot * max_deg_s;
  twist_.twist.linear.y = 0.0;
  twist_.twist.linear.z = 0.0;
  twist_.twist.angular.x = 0.0;
  twist_.twist.angular.y = 0.0;
  
  /*
   message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    // Fill in the Twist message
    message.twist.linear.x = 1.0;
    message.twist.linear.y = 0.0;
    message.twist.linear.z = 0.0;
    message.twist.angular.x = 0.0;
    message.twist.angular.y = 0.0;
    message.twist.angular.z = 0.5;
  */


  cmd_vel_pub->publish(twist_);
}

double TeleopJoy::Impl::easeInSine(double value)
{
  return (value == 0) ? 0 : 1 - std::cos((value * M_PI) / 2);
}

bool TeleopJoy::Impl::haveSameSign(double a, double b)
{
  return (a * b) >= 0.0;
}

void TeleopJoy::Impl::laser_scan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  points_.clear();
  double current_angle = laser_scan->angle_min;

  for (auto range_value : laser_scan->ranges)
  {
    if (std::isnan(range_value))
      continue;

    double x = range_value * std::cos(current_angle);
    double y = range_value * std::sin(current_angle);

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    points_.push_back(point);
    current_angle += laser_scan->angle_increment;
  }
}

void TeleopJoy::Impl::check_stop()
{
  chk_forward = false;
  chk_left_side = false;
  chk_right_side = false;

  for (auto &point : points_)
  {
    double x = point.x * 350; // 미터를 픽셀로 변환
    double y = point.y * 350; // 미터를 픽셀로 변환

    double rotated_x = x * cos_theta_ - y * sin_theta_;
    double rotated_y = x * sin_theta_ + y * cos_theta_;

    double delta_x = rotated_x - 100;
    double delta_y = rotated_y + 90;

    double dx = centerX + rotated_x;
    double dy = centerY - rotated_y;

    if (dx > 0.0 && dy > 0.0)
    {
      if (dx <= width && dy <= height)
      {
        double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double ratio = std::min(std::max(distance - 200, 0.0) / 200.0, 1.0);

        if (centerX - 20 <= dx && dx <= centerX + 220)
        {
          if (centerY >= dy && dy >= centerY - 200)
            chk_forward = true;
        }
        else
        {
          if (ratio < 0.10)
          {
            if (dx < centerX + 20)
              chk_left_side = true;
            else if (dx > centerX + 180)
              chk_right_side = true;
          }
        }
      }
    }
  }
}

void TeleopJoy::Impl::sendControlOn()
{
  auto msg0 = std_msgs::msg::ByteMultiArray();
  auto msg1 = std_msgs::msg::ByteMultiArray();

  // 바이트 데이터 설정
  msg0.data = {0xFF, 0xFE, 0x00, 0x03, 0xF0, 0x0C, 0x00};
  msg1.data = {0xFF, 0xFE, 0x01, 0x03, 0xEF, 0x0C, 0x00};

  // 메시지 발행
  pub_raw->publish(msg0);
  pub_raw->publish(msg0); // 필요에 따라 두 번 발행
  pub_raw->publish(msg1);
  pub_raw->publish(msg1); // 필요에 따라 두 번 발행
}

double TeleopJoy::Impl::gen_profile(double v_ref, double vout, double dt, double amax)
{
  double da = 0;
  double dv = 0;

  if (v_ref == vout)
  {
    dv = 0;
  }
  else
  {
    da = (v_ref - vout) / dt;
    if (std::abs(da) >= amax)
    {
      da = (da > 0) ? amax : -amax;
    }
  }

  dv = da * dt;
  vout += dv;
  return vout;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TeleopJoy)