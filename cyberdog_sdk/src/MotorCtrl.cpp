// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// ACHIEVED: Control Single Motor and publish jointstate topic
// NEW TARGET:
//   1. Real time simulation (Cyberdog control virtual dog)
//      Finished, waiting for test
//   2. Calculate target angle and control the dog (virtual dog control Cyberdog)


#include <CustomInterface.h>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class SubPub : public rclcpp::Node, CustomInterface
{
public:
    SubPub(const double& loop_rate);
private:
    Motor_Cmd   target;
    void FSM_control();
    void joy_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_JointState;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_Imu;
    int state = 0;
    int mode_transfer = 0;
    bool first_run = true;
    long long count = 0;
    float stable_q[12];
    float init_q[12];
    float target1_q[3] = {0 / 57.3, 80 / 57.3, -135 / 57.3};
    float target2_q[3] = {0 / 57.3, 45 / 57.3, -90 / 57.3};
};



SubPub::SubPub(const double& loop_rate) : Node("SubPub"), CustomInterface(loop_rate)
{
    std::cout << "Entering custructor function \n" << std::endl;
    subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&SubPub::joy_control, this, std::placeholders::_1));
    std::cout << "Establishing Joy topic Subscriber \n" << std::endl;
    publisher_JointState = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    std::cout << "Establishing Jointstate topic Publisher \n" << std::endl;
    publisher_Imu = this->create_publisher<sensor_msgs::msg::Imu>("Imu",10);
    std::cout << "Establishing Imu topic Publisher" << std::endl;
    timer_1 = this->create_wall_timer(
        2ms, std::bind(&SubPub::FSM_control, this));
    // timer_2 = this->create_wall_timer(
    //     2ms, std::bind(&SubPub::FSM_modeset, this));
    std::cout << "Setting timer for Control & Publish function \n" << std::endl;
}

void SubPub::FSM_control() {

    //std::cout << "count: \n" << count << std::endl;

    if ( flag ) {
      //std::cout << "Control Mode Enabled \n" << std::endl;
        if ( state == STAND ){
            float t =  (count/1000)  > 2 ? 2 :  (count/1000) ;
            if ( first_run ) {
                for ( int i = 0; i < 12; i++ )
                    init_q[ i ] = robot_data.q[ i ];
                if ( init_q[ 2 ] < -0.1 && init_q[ 5 ] < -0.1 && init_q[ 8 ] < -0.1 && init_q[ 11 ] < -0.1 ) {
                    first_run = false;
                    count     = 0;
                }
            }
            else {
                for ( int i = 0; i < 12; i++ ) {
                    if ( t < 1.0 )
                    {   motor_cmd.q_des[ i ]   = target1_q[ i % 3 ] * t + init_q[ i ] * ( 1 - t );
                    }
                    else if ( t < 2.0 )
                    {   motor_cmd.q_des[ i ]   = target2_q[ i % 3 ] * ( t - 1 ) + target1_q[ i % 3 ] * ( 2 - t );
                    }
                    else
                    {   state = STABLE;
                        stable_q[ i ] = target2_q[ i % 3 ];
                    }
                    motor_cmd.kp_des[ i ]  = 100;
                    motor_cmd.kd_des[ i ]  = 3;
                    motor_cmd.qd_des[ i ]  = 0;
                    motor_cmd.tau_des[ i ] = 0;
                }
            }
        }

        if ( state == STABLE ) {
            if ( mode_transfer ) { state = JOYSTICK; }  //transfer to joy_control mode
            else{
              for (int i = 0; i < 12; i++){
                motor_cmd.q_des[ i ] = stable_q[ i ];
                target.q_des[ i ] = stable_q[ i ];
              }
            }
        }

        if ( state == JOYSTICK ){
            if ( mode_transfer == 0 )
                state = STABLE;
            else
                for (int i = 0; i < 12; i++){
                    motor_cmd.q_des[ i ]  = target.q_des[ i ];
                    stable_q[ i ] = robot_data.q[ i ];}
        }

        if ( state == RESET ){
          float t = ( count % 1000 ) / 1000;
          for ( int i = 0; i < 12 ; i++ ){
            motor_cmd.q_des[ i ]   = target2_q[ i % 3 ] * t + stable_q[ i ] * ( 1 - t );
          }
        }

        if ((count++) % 50 == 0) {
          if ( state == STAND )
            std::cout << "STATE ID: " << state << "  STAND STATE\n" << std::endl;
          else if ( state == STABLE )
            std::cout << "STATE ID: " << state << "  STABLE STATE\n" << std::endl;
          else
            std::cout << "STATE ID: " << state << "  JOYSTICK STATE\n" << std::endl;
          }

        // if ((count++) % 10 == 0) {
        //     printf("interval:---------%.4f-------------\n", robot_data.ctrl_topic_interval);
        //     printf("rpy [3]:");
        //     for (int i = 0; i < 3; i++)
        //         printf(" %.2f", robot_data.rpy[i]);
        //     printf("\nacc [3]:");
        //     for (int i = 0; i < 3; i++)
        //         printf(" %.2f", robot_data.acc[i]);
        //     printf("\nquat[4]:");
        //     for (int i = 0; i < 4; i++)
        //         printf(" %.2f", robot_data.quat[i]);
        //     printf("\nomeg[3]:");
        //     for (int i = 0; i < 3; i++)
        //         printf(" %.2f", robot_data.omega[i]);
        //     printf("\nq  [12]:");
        //     for (int i = 0; i < 12; i++)
        //         printf(" %.2f", robot_data.q[i]);
        //     printf("\nqd [12]:");
        //     for (int i = 0; i < 12; i++)
        //         printf(" %.2f", robot_data.qd[i]);
        //     printf("\ntau[12]:");
        //     for (int i = 0; i < 12; i++)
        //         printf(" %.2f", robot_data.tau[i]);
        //     printf("\nctrl[12]:");
        //     for (int i = 0; i < 12; i++)
        //         printf(" %.2f", motor_cmd.q_des[i]);
        //     printf("\n\n");
        // }
    }

    //Publishing robot data to ROS2 TOPIC Channel
    auto motor_data = sensor_msgs::msg::JointState();
    motor_data.header.frame_id = "motor";
    motor_data.header.stamp = now();
    motor_data.name.resize(12);
    motor_data.position.resize(12);
    motor_data.velocity.resize(12);
    motor_data.effort.resize(12);
    for ( int i = 0; i < 12; i++ ) {
      motor_data.name[ i ] = builtin_name[ i ];
      motor_data.position[ i ] = robot_data.q[ i ];
      motor_data.velocity[ i ] = robot_data.qd[ i ];
      motor_data.effort[ i ] = robot_data.tau[ i ];
    }
    publisher_JointState->publish(motor_data);

    auto imu_data = sensor_msgs::msg::Imu();
    imu_data.header.frame_id = "imu";
    imu_data.header.set__stamp(now());
    imu_data.angular_velocity.set__x(robot_data.omega[ 0 ]);
    imu_data.angular_velocity.set__y(robot_data.omega[ 1 ]);
    imu_data.angular_velocity.set__z(robot_data.omega[ 2 ]);
    imu_data.linear_acceleration.set__x(robot_data.acc[ 0 ]);
    imu_data.linear_acceleration.set__y(robot_data.acc[ 1 ]);
    imu_data.linear_acceleration.set__z(robot_data.acc[ 2 ]);
    imu_data.orientation.set__w(robot_data.quat[ 0 ]);
    imu_data.orientation.set__x(robot_data.quat[ 1 ]);
    imu_data.orientation.set__y(robot_data.quat[ 2 ]);
    imu_data.orientation.set__z(robot_data.quat[ 3 ]);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++){
        imu_data.angular_velocity_covariance[ i * j ] = robot_data.omega[ i ] * robot_data.omega[ j ];
        imu_data.linear_acceleration_covariance[ i * j ] = robot_data.acc[ i ] * robot_data.acc[ j ];
        imu_data.orientation_covariance[ i * j ] = robot_data.quat[ i + 1 ] * robot_data.quat[ j + 1 ];
      }
    }
    publisher_Imu->publish(imu_data);
}

void SubPub::joy_control(const sensor_msgs::msg::Joy::SharedPtr msg) {
    //std::cout<<"Joy Control Node Entered \n"<<std::endl;
    mode_transfer = 0;
    double cmd;
    int motor_id;
    int base = 0;
    int bias = 0;
    if (msg->buttons[3] == 1) { bias = 1; }
    if (msg->buttons[4] == 1) { bias = 2; }
    motor_id = base + bias;

    std::cout<<"Motor_ID: "<< motor_id << std::endl;

    if (msg->buttons[0]) {
        mode_transfer = 1;
        cmd = robot_data.q[motor_id] + msg->axes[0] * 0.75;
        if (cmd > -0.75 && cmd < 0.75) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 0.75; }
    }
    if (msg->buttons[3]) {
        mode_transfer = 1;
        cmd = robot_data.q[motor_id] + msg->axes[0] * 2.81;
        if (cmd > -1.257 && cmd < 4.363) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 2.81 - 1.257; }
    }
    if (msg->buttons[4]) {
        mode_transfer = 1;
        cmd = robot_data.q[motor_id] + msg->axes[0] * 0.986;
        if (cmd > -2.478 && cmd < -0.506) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 0.986 - 2.478; }
    }
    std::cout<<"mode_transfer: "<<mode_transfer<<std::endl;
}

std::shared_ptr< CustomInterface >   io;
void signal_callback_handler( int signum ) {
  io->Stop();
  ( void )signum;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    signal( SIGINT, signal_callback_handler );
    rclcpp::spin(std::make_shared< SubPub >(500));
    rclcpp::shutdown();
    return 0;
}