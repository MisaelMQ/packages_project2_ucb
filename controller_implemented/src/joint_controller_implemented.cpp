#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace controller_implemented {
class JointControllerImplemented : public controller_interface::Controller<hardware_interface::EffortJointInterface> {

  bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh) {
    // Saving Joint Name
    std::string joint_name;
    if (!nh.getParam("joint", joint_name)) {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // Getting initial Values
    joint_ = hw->getHandle(joint_name);
    command_ = joint_.getPosition();

    // Loading Proportional Gain
    if (!nh.getParam("p_gain", pgain_)) {
      ROS_ERROR("Could not find proportional gain value");
      return false;
    }

    // Loading Integral Gain
    if (!nh.getParam("i_gain", igain_)) {
      ROS_ERROR("Could not find integral gain value");
      return false;
    }

    // Loading Derivative Gain
    if (!nh.getParam("d_gain", dgain_)) {
      ROS_ERROR("Could not find derivative gain value");
      return false;
    }

    // Start subscriber
    sub_command_ = nh.subscribe<std_msgs::Float64>(
        "command", 1, &JointControllerImplemented::setCommandCB, this);

    // Initial Values
    previous_error_ = 0;
    sum_error_ = 0;
    dt_ = 0.000001;
    start_time_ = 0;
    end_time_ = 0;

    return true;
  }

  // Running Controller
  void update(const ros::Time &time, const ros::Duration &period) {
    // Reading Start Time
    start_time_ = clock();

    // Calculating Error
    double error = command_ - joint_.getPosition();
    sum_error_ += (error * dt_);

    // Calculating Proportional Control
    double p_out = pgain_ * error;
    // Calculating Integral Control
    double i_out = igain_ * sum_error_;
    // Calculating Derivative Control
    double d_out = dgain_ * ((error - previous_error_) * dt_);

    // Commanded Control
    double commanded_effort = p_out + i_out + d_out;

    // Updating Variables
    previous_error_ = error;
    // Commanded Effort
    joint_.setCommand(commanded_effort);

    // Reading End Time
    end_time_ = clock();
    dt_ = (double)(end_time_ - start_time_) / CLOCKS_PER_SEC;
  }

  // Callback for Subscriber
  void setCommandCB(const std_msgs::Float64ConstPtr &msg) {
    command_ = msg->data;
  }

  // Controller Startup
  void starting(const ros::Time &time) {}

  // Stopping Controller
  void stopping(const ros::Time &time) {}

private:
  hardware_interface::JointHandle joint_;
  double pgain_;
  double igain_;
  double dgain_;
  double previous_error_;
  double sum_error_;
  double dt_;
  clock_t start_time_;
  clock_t end_time_;
  double command_;
  ros::Subscriber sub_command_;
};

PLUGINLIB_EXPORT_CLASS(controller_implemented::JointControllerImplemented,
                       controller_interface::ControllerBase);
} // namespace controller_implemented