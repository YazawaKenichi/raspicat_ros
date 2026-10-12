/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop_twist_joy/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  bool autorun_flag;
  bool experiment_flag;
  bool ems_enable;
  int64_t ems_button;
  int64_t enable_autorun_button;
  int64_t enable_experiment_button;
  int64_t autorun_forward_button;
  int64_t autorun_backward_button;
  int64_t autorun_buffer;
  int64_t experiment_buffer;

  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, int64_t> axis_angular_adjustment_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  float_t speed_x_max;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options) : Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->autorun_flag = true;

  pimpl_->enable_autorun_button = this->declare_parameter("enable_autorun_button", -1);

  //! for YAZAWA's Experiment
  pimpl_->ems_enable = false;
  pimpl_->ems_button = this->declare_parameter("ems_button", -1);
  pimpl_->enable_experiment_button = this->declare_parameter("enable_experiment_button", -1);
  pimpl_->autorun_forward_button = this->declare_parameter("autorun_forward_button", -1);
  pimpl_->autorun_backward_button = this->declare_parameter("autorun_backward_button", -1);

  pimpl_->autorun_buffer = 0;
  pimpl_->experiment_flag = false;
  pimpl_->experiment_buffer = 0;

  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_linear", default_linear_map);
  this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("axis_angular", default_angular_map);
  this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  std::map<std::string, int64_t> default_angular_adjustment_map{
      {"yaw", 3L},
          {"pitch", -1L},
          {"roll", -1L},
  };
  this->declare_parameters("axis_angular_adjustment", default_angular_adjustment_map);
  this->get_parameters("axis_angular_adjustment", pimpl_->axis_angular_adjustment_map);

  // autorun scale
  std::map<std::string, double> default_scale_linear_autorun_map{
    {"x", 1.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_autorun", default_scale_linear_autorun_map);
  this->get_parameters("scale_linear_autorun", pimpl_->scale_linear_map["autorun"]);

  // autorun scale
  std::map<std::string, double> default_scale_angular_autorun_map{
    {"yaw", 1.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_autorun", default_scale_angular_autorun_map);
  this->get_parameters("scale_angular_autorun", pimpl_->scale_angular_map["autorun"]);

  ////////// for YAZAWA's Experiment //////////
  std::map<std::string, double> default_scale_linear_experiment_map{
      {"x", 99999.9},
      {"y", 99999.9},
      {"z", 99999.9},
  };
  std::map<std::string, double> default_scale_angular_experiment_map{
      {"yaw", 99999.9},
      {"pitch", 99999.9},
      {"roll", 99999.9},
  };
  this->declare_parameters("scale_linear_experiment", default_scale_linear_experiment_map);
  this->declare_parameters("scale_angular_experiment", default_scale_angular_experiment_map);
  this->get_parameters("scale_linear_experiment", pimpl_->scale_linear_map["experiment"]);
  this->get_parameters("scale_angular_experiment", pimpl_->scale_angular_map["experiment"]);

  pimpl_->speed_x_max = 0;

  pimpl_->sent_disable_msg = false;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                              "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                              "axis_angular_adjustment.yaw", "axis_angular_adjustment.pitch", "axis_angular_adjustment.roll",
                                              "enable_autorun_button",
                                              "enable_experiment_button", "autorun_forward_button", "autorun_backward_button"
    };
    static std::set<std::string> doubleparams = {"scale_linear_autorun.x", "scale_linear_autorun.y", "scale_linear_autorun.z",
                                                 "scale_angular_autorun.yaw", "scale_angular_autorun.pitch", "scale_angular_autorun.roll",
                                                 "scale_linear_experiment.x", "scale_linear_experiment.y", "scale_linear_experiment.z",
                                                 "scale_angular_experiment.roll", "scale_angular_experiment.pitch", "scale_angular_experiment.yaw"
    };
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      if (parameter.get_name() == "enable_autorun_button")
      {
          this->pimpl_->enable_autorun_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.x")
      {
        this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.y")
      {
        this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.z")
      {
        this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular_adjustment.yaw")
      {
          this->pimpl_->axis_angular_adjustment_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular_adjustment.pitch")
      {
          this->pimpl_->axis_angular_adjustment_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular_adjustment.roll")
      {
          this->pimpl_->axis_angular_adjustment_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.yaw")
      {
        this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.pitch")
      {
        this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.roll")
      {
        this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "scale_linear_autorun.x")
      {
        this->pimpl_->scale_linear_map["autorun"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_autorun.y")
      {
        this->pimpl_->scale_linear_map["autorun"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_autorun.z")
      {
        this->pimpl_->scale_linear_map["autorun"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_autorun.yaw")
      {
        this->pimpl_->scale_angular_map["autorun"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_autorun.pitch")
      {
        this->pimpl_->scale_angular_map["autorun"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_autorun.roll")
      {
        this->pimpl_->scale_angular_map["autorun"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      ////////// for YAZAWA's Experiment //////////
      // 直進高速
      else if (parameter.get_name() == "scale_linear_experiment.x")
      {
        this->pimpl_->scale_linear_map["experiment"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_experiment.y")
      {
        this->pimpl_->scale_linear_map["experiment"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_experiment.z")
      {
        this->pimpl_->scale_linear_map["experiment"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_experiment.yaw")
      {
        this->pimpl_->scale_angular_map["experiment"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_experiment.pitch")
      {
        this->pimpl_->scale_angular_map["experiment"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_experiment.roll")
      {
        this->pimpl_->scale_angular_map["experiment"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "autorun_forward_button")
      {
        this->pimpl_->autorun_forward_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "autorun_backward_button")
      {
        this->pimpl_->autorun_backward_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopTwistJoy::~TeleopTwistJoy()
{
  delete pimpl_;
}

double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
    // fieldname が axis_map のどのインデックスにあるか 
  if (axis_map.find(fieldname) == axis_map.end() ||
      axis_map.at(fieldname) == -1L ||
      scale_map.find(fieldname) == scale_map.end() ||
      static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname)) // 
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

double experimentVal(const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
    if(scale_map.find(fieldname) == scale_map.end())
    {
        return 0.0;
    }
    return 1.0 * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  float_t speed_x_temporary = 0;
  float_t speed_yaw_temporary = 0;

  speed_x_temporary = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  speed_yaw_temporary = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");

  if(this->experiment_flag)
  {
      //! 既定値走行
      speed_x_temporary = experimentVal(scale_linear_map[which_map], "x");
      speed_yaw_temporary = experimentVal(scale_angular_map[which_map], "yaw");
  }

  if(this->autorun_flag)
  {
      //! 自動走行
      // 直進方向の値を更新
      float_t limit;
      // x の値の最大値を決定
      limit = 1.0f * scale_linear_map["autorun"]["x"];
      // 十字キーの値を / 10 してから足す
      this->speed_x_max += (float_t) speed_x_temporary / 10;
      // 出力値が最大値を超えないように制限
      this->speed_x_max = this->speed_x_max >  limit ?  limit : this->speed_x_max;
      this->speed_x_max = this->speed_x_max < -limit ? -limit : this->speed_x_max;
      // 前進ボタンが押されていたら直進速度を最大値にする
      if(autorun_forward_button >= 0 &&
         static_cast<int>(joy_msg->buttons.size()) > autorun_forward_button &&
         joy_msg->buttons[autorun_forward_button])
      {
          this->speed_x_max = limit;
      }
      // 後退ボタンが押されていたら直進速度を最小値にする
      if(autorun_backward_button >= 0 &&
         static_cast<int>(joy_msg->buttons.size()) > autorun_backward_button &&
         joy_msg->buttons[autorun_backward_button])
      {
          this->speed_x_max = -limit;
      }
      // 出力値の決定
      speed_x_temporary = this->speed_x_max;

      // 角度方向の値を更新
      float_t joystick = getVal(joy_msg, axis_angular_adjustment_map, scale_angular_map[which_map], "yaw");
      // Joystick の値と十字キーの値を加算
      float_t sum = (speed_yaw_temporary + joystick);
      // yaw の値の最大値を決定 ( 十字キーの最大値で制限 )
      limit = 1.0f * scale_angular_map["autorun"]["yaw"];
      // Joystick と十字キーを加算した値が最大値の範囲に収まるように制限
      sum = sum >  limit ?  limit : sum;
      sum = sum < -limit ? -limit : sum;
      // 出力値の決定
      speed_yaw_temporary = sum;
  }

  cmd_vel_msg->linear.x = speed_x_temporary;
  cmd_vel_msg->angular.z = speed_yaw_temporary;

  cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");
  cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");

  if(ems_enable)
  {
      cmd_vel_msg->linear.x = 0;
      cmd_vel_msg->linear.y = 0;
      cmd_vel_msg->linear.z = 0;
      cmd_vel_msg->angular.x = 0;
      cmd_vel_msg->angular.y = 0;
      cmd_vel_msg->angular.z = 0;
  }

  cmd_vel_pub->publish(std::move(cmd_vel_msg));
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
    //! 緊急停止ボタン
    if(ems_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > ems_button && joy_msg->buttons[ems_button])
    {
        if(!ems_enable) RCLCPP_INFO(rclcpp::get_logger("Joy_Callback"), "========== EMS Depress ==========");
        ems_enable = true;
        // 緊急停止後は autorun (速度 0) の状態にする
        autorun_flag = true;
        experiment_flag = false;
        speed_x_max = 0;
        // 押している間は停止指令を出し続ける
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel_pub->publish(std::move(cmd_vel_msg));
        sent_disable_msg = true;
        return;
    }
    else
    {
        if(ems_enable) RCLCPP_INFO(rclcpp::get_logger("Joy_Callback"), "========== EMS Release ==========");
        ems_enable = false;
    }

    if(enable_autorun_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > enable_autorun_button)
    {
        //! autorun ボタンが設定されているなら
        auto autorun_button = joy_msg->buttons[enable_autorun_button];
        if(autorun_button - this->autorun_buffer > 0)
        {
            //! autorun ボタンが押下されたら
            this->autorun_flag = this->autorun_flag ? false : true;
            // autorun を on にしたら experiment は止める
            if(this->autorun_flag) this->experiment_flag = false;
        }
        this->autorun_buffer = autorun_button;
    }

    ////////// for YAZAWA's Experiment //////////
    //! 実験動作ボタン
    bool experiment_button_assigned = (enable_experiment_button >= 0 && static_cast<int>(joy_msg->buttons.size()) > enable_experiment_button);
    if(experiment_button_assigned)
    {
        //! 実験動作ボタンが設定されているなら
        auto experiment_button = joy_msg->buttons[enable_experiment_button] ? 1 : 0;
        if(experiment_button - this->experiment_buffer > 0 && !this->autorun_flag)
        {
            //! experiment ボタンが押下されたら
            this->experiment_flag = !this->experiment_flag;
        }
        this->experiment_buffer = experiment_button;
        //! RCLCPP_INFO(rclcpp::get_logger("Joy_Callback"), "Autorun-Flag : %d, Experiment-Flag : %d", this->autorun_flag ? 1 : 0, this->experiment_flag ? 1 : 0);
    }

    if(!autorun_flag)
    {
        // 直進速度をリセット
        this->speed_x_max = 0;
    }

    if(experiment_flag)
    {
        sendCmdVelMsg(joy_msg, "experiment");
    }
    else if(autorun_flag)
    {
        sendCmdVelMsg(joy_msg, "autorun");
    }
    else
    {
        // RCLCPP_INFO(rclcpp::get_logger("joy_callback_logger"), "Stop The Robot : %d", !sent_disable_msg ? 1 : 0);
        // When enable button is released, immediately send a single no-motion command
        // in order to stop the robot.
        if (!sent_disable_msg)
        {
            // Initializes with zeros by default.
            auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_vel_pub->publish(std::move(cmd_vel_msg));
            sent_disable_msg = true;
        }
    }
}
}  // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)

