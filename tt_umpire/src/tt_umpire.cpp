#include <cmath>
#include <chrono>
#include <memory>
#include <algorithm>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TT_Umpire : public rclcpp::Node {
    public:
        TT_Umpire() : Node("tt_umpire") {
            dcnt_ = 0;
            state_ = 0;
            mean_d_ = 0;
            stop_ = false;
            curr_points_ = 0;
            prev_state_ = -1;

            std::string target_node_name;
            string_publisher_ = this->create_publisher<std_msgs::msg::String>("~/info", 1);
            target_node_name = this->declare_parameter("target_node_name", std::string("placeholder_node_name"));
            seconds_of_attention_ = this->declare_parameter("seconds_of_attention", 300);
            assignment_ = this->declare_parameter("assignment", 0);
            info(std::string("The umpire is now taking a close look into ").append(target_node_name).append(", for Assignment ").append(std::to_string(assignment_)).append("..."));
            info(std::string("The umpire's attention will last ").append(std::to_string(seconds_of_attention_)).append(" minutes... Good luck!"));
            lifecycle_checker_ = this->create_client<lifecycle_msgs::srv::GetState>(target_node_name+std::string("/get_state"));

            start_time_ = this->now();
            if (assignment_ == 1) {
                points_ = std::vector<int>{5,5,5,5,80};
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                topics_to_check_ = std::vector<std::string>{"/head_front_camera/depth/image_raw","/head_front_camera/depth_registered/image_raw","/head_front_camera/rgb/image_raw", "/head_front_camera/image_raw", "/head_front_camera/depth_registered/points"};
                odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/p3d/ball_odom", 1, std::bind(&TT_Umpire::odom_callback, this, _1));
                timer_ = rclcpp::create_timer(this, this->get_clock(), 0.5s, std::bind(&TT_Umpire::first_assignment_grading_system, this));
            }
            else if (assignment_ == 2) {
                topics_to_check_ = std::vector<std::string>{"/tf","/tf_static"};
                points_ = std::vector<int>{5,5,5,5,20,20,20,20};
                auto a2_goal = geometry_msgs::msg::Transform();
                a2_goal.translation.x = 2.607;
                a2_goal.translation.y = 1.4525;
                a2_goal.rotation.z = -0.707;
                a2_goal.rotation.w = 0.707;
                a2_goals_.push_back(a2_goal);
                a2_goal.translation.x = 2.607;
                a2_goal.translation.y = -1.4525;
                a2_goal.rotation.z = 0.707;
                a2_goal.rotation.w = 0.707;
                a2_goals_.push_back(a2_goal);
                a2_goal.translation.x = 4.7;
                a2_goal.translation.y = 0.0;
                a2_goal.rotation.z = 1.0;
                a2_goal.rotation.w = 0.0;
                a2_goals_.push_back(a2_goal);
                a2_goal.translation.x = 0.58;
                a2_goal.translation.y = 0.0;
                a2_goal.rotation.z = 0.0;
                a2_goal.rotation.w = 1.0;
                a2_goals_.push_back(a2_goal);
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                timer_ = rclcpp::create_timer(this, this->get_clock(), 0.5s, std::bind(&TT_Umpire::second_assignment_grading_system, this));
            }
            else if (assignment_ == 3) {
                timer_ = rclcpp::create_timer(this, this->get_clock(), 0.5s, std::bind(&TT_Umpire::third_assignment_grading_system, this));
            }
        }

    private:
        void award_points(const int s, const double w=1.0) {
            auto awarded_points = w * points_[s];
            info(std::string("\n*****\nHere, get ").append(std::to_string(awarded_points)).append(" points!\n*****"));
            curr_points_ += awarded_points;
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (stop_) { 
                return;
            }
            latest_odom_ = msg;
        }

        void e_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
          std::shared_ptr<std_srvs::srv::Empty::Response>) {
            if (dcnt_ >= 4) {
                return;
            }
            try {
                auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                auto x = t.transform.translation.x - a2_goals_[dcnt_].translation.x;
                auto y = t.transform.translation.y - a2_goals_[dcnt_].translation.y;
                auto z = t.transform.rotation.z - a2_goals_[dcnt_].rotation.z;
                auto w = t.transform.rotation.w - a2_goals_[dcnt_].rotation.w;
                auto distance = sqrt(x*x+y*y+(z*z+w*w)/2);
                auto dweight = distance > 0 ? std::min(1.0, 1.0/(6*distance)) : 1.0;
                info(std::string("Measured distance and weight ").append(std::to_string(distance)).append(",").append(std::to_string(dweight)));
                award_points(state_, dweight);
                dcnt_++;
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "%s!", ex.what());
            }
            state_++;
        }

        void info(const std::string s) const {
            RCLCPP_INFO(this->get_logger(), "%s", s.c_str());
            auto msg = std_msgs::msg::String();
            msg.data = s;
            string_publisher_->publish(msg);
        }

        void time_handler() {
            if (((this->now()-start_time_).seconds() > seconds_of_attention_) or (assignment_ == 2 and dcnt_ == 4)) {
                timer_->cancel();
                stop_ = true;
                if (assignment_ == 1) {
                    curr_points_ += (points_.back() - points_.back() * mean_d_) * (dcnt_ > 0);
                }
                info(std::string("\n***************\nFinal points awarded ").append(std::to_string(curr_points_)).append("/").append(std::to_string(std::reduce(points_.begin(), points_.end()))).append("...\n***************"));
                rclcpp::shutdown();
            }
        }

        bool lifecycle_tester() {
            if (prev_state_ != state_) {
                info(std::string("Testing if your node has implemented lifecycles..."));
                prev_state_ = state_;
            }
            if (lifecycle_checker_->service_is_ready()) {
                info(std::string("Good job, your node has implemented lifecycles!"));
                prev_state_ = state_;
                state_++;
                award_points(prev_state_);
            }
            else {
                info(std::string("Service not ready!"));
                return false;
            }
            return true;
        }

        void first_assignment_grading_system() {
            time_handler();
            switch (state_) {
                case 0:
                    if (lifecycle_tester()) {
                        break;
                    }
                    [[fallthrough]];
                case 1: [[fallthrough]]; case 2: [[fallthrough]]; case 3:
                        if (prev_state_ != state_) {
                            info(std::string("Testing if your node has the expected paired subscription count and state..."));
                            prev_state_ = state_;
                        }
                        {
                            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                            auto result = lifecycle_checker_->async_send_request(request, std::bind(&TT_Umpire::get_state_callback, this, _1));
                            // I would prefer a simple async_send_request(request) here, but there is a bug (PR not merged yet)
                            // https://github.com/ros2/rclcpp/issues/2039
                        }
                        break;
                case 4:
                        if (prev_state_ != state_) {
                            info(std::string("Testing the distance between the ball's real location and the estimation..."));
                            prev_state_ = state_;
                        }
                        {
                            try {
                                auto t = tf_buffer_->lookupTransform(latest_odom_->header.frame_id, latest_odom_->child_frame_id, tf2::TimePointZero);
                                auto x = t.transform.translation.x - latest_odom_->pose.pose.position.x;
                                auto y = t.transform.translation.y - latest_odom_->pose.pose.position.y;
                                auto z = t.transform.translation.z - latest_odom_->pose.pose.position.z;
                                auto distance = sqrt(x*x+y*y+z*z);
                                dcnt_++;
                                mean_d_ = distance / dcnt_;
                                if (dcnt_ % 10 == 0) {
                                    info(std::string("Mean distance = ").append(std::to_string(mean_d_)).append(" meters!"));
                                }
                            }
                            catch (const tf2::TransformException & ex) {
                                RCLCPP_ERROR(this->get_logger(), "%s!", ex.what());
                            }
                        }
                    break;
            }
        }

        void second_assignment_grading_system() {
            time_handler();
            switch (state_) {
                case 0:
                    if (lifecycle_tester()) {
                        break;
                    }
                    [[fallthrough]];
                case 1: [[fallthrough]]; case 2: [[fallthrough]]; case 3:
                        if (prev_state_ != state_) {
                            info(std::string("Testing if your node has the expected paired subscription count and state..."));
                            prev_state_ = state_;
                        }
                        {
                            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                            auto result = lifecycle_checker_->async_send_request(request, std::bind(&TT_Umpire::get_state_callback, this, _1));
                            // I would prefer a simple async_send_request(request) here, but there is a bug (PR not merged yet)
                            // https://github.com/ros2/rclcpp/issues/2039
                        }
                        break;
                case 4:
                        if (prev_state_ != state_) {
                            e_service_ = this->create_service<std_srvs::srv::Empty>("~/assignment2/i_feel_confident", std::bind(&TT_Umpire::e_callback, this, _1, _2));
                            info(std::string("Waiting for an empty request to test the robot's position..."));
                            prev_state_ = state_;
                        }
            }
        }

        void third_assignment_grading_system() {
            time_handler();
            switch (state_) {
                case 0:
                    if (lifecycle_tester()) {
                        break;
                    }
                    // [[fallthrough]];
            }
        }
        
        void get_state_callback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
            if (stop_) { 
                return;
            }
            auto result = future.get();
            int sub_cnt = 0;
            if ( (state_ == 1 and result->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) or (state_ == 2 and result->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) ) {
                for (auto topic : topics_to_check_) {
                    sub_cnt += this->count_subscribers(topic);
                }
                if ( (assignment_ == 1 and sub_cnt == 0) or (assignment_ == 2 and sub_cnt/2<9) ) {
                    info(std::string("Good job, you have not subscribed to any suspicious topics while being unconfigured/inactive!"));
                    prev_state_ = state_;
                    award_points(prev_state_);
                }
                state_++;
            }
            else if (state_ == 3 and result->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                for (auto topic : topics_to_check_) {
                    sub_cnt += this->count_subscribers(topic);
                }
                if ( (assignment_ == 1 and sub_cnt > 0) or (assignment_ == 2 and sub_cnt/2>=9) ) {
                    info(std::string("Good job, you have subscribed to at least one interesting topic while activated!"));
                    prev_state_ = state_;
                    award_points(prev_state_);
                }
                state_++;
            }
        }

        std::atomic<bool> stop_;
        std::vector<int> points_;
        rclcpp::Time start_time_;
        double mean_d_, curr_points_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::string> topics_to_check_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::vector<geometry_msgs::msg::Transform> a2_goals_;
        std::shared_ptr<nav_msgs::msg::Odometry> latest_odom_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr e_service_;
        int seconds_of_attention_, state_, prev_state_, assignment_, dcnt_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr lifecycle_checker_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TT_Umpire>());
    rclcpp::shutdown();
    return 0;
}
