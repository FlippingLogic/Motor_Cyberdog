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

#include <CustomInterface.h>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

Motor_Cmd   target;
int         state;
int         mode_transfer = 0;

std::shared_ptr< CustomInterface >   io;

void signal_callback_handler( int signum ) {
    io->Stop();
    ( void )signum;
}

class SubPub : public rclcpp::Node, CustomInterface
{
public:
    SubPub(const double& loop_rate);
private:
    void FSM_control();
    void joy_control(const sensor_msgs::msg::Joy::SharedPtr msg);
    void pub_JointState();
    void pub_Imu();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_JointState;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_Imu;
};

SubPub::SubPub(const double &loop_rate) : Node("SubPub"), CustomInterface(loop_rate)
{
    this->create_subscription<sensor_msgs::msg::Joy>(
            "topic", 10, std::bind(&SubPub::joy_control, this, std::placeholders::_1));
    this->create_wall_timer(
            100ms, std::bind(&SubPub::FSM_control, this));
    publisher_JointState = this->create_publisher<sensor_msgs::msg::JointState>("JointState",10);
    publisher_Imu = this->create_publisher<sensor_msgs::msg::Imu>("Imu",10);
    this->create_wall_timer(
            100ms, std::bind(&SubPub::pub_JointState, this));
}

void SubPub::FSM_control() {
    bool first_run = true;
    long long count = 0;
    float stable_q[12];
    float init_q[12];
    float target1_q[3] = {0 / 57.3, 80 / 57.3, -135 / 57.3};
    float target2_q[3] = {0 / 57.3, 45 / 57.3, -90 / 57.3};

    if ( flag ) {
        if ( state == 0 ){
            float t = ( count / 1500.0 ) > 2 ? 2 : ( count / 1500.0 );
            if ( first_run ) {
                for ( int i = 0; i < 12; i++ )
                    init_q[ i ] = io->robot_data.q[ i ];
                if ( init_q[ 2 ] < -0.1 && init_q[ 5 ] < -0.1 && init_q[ 8 ] < -0.1 && init_q[ 11 ] < -0.1 ) {
                    first_run = false;
                    count     = 0;
                }
            }
            else {
                for ( int i = 0; i < 12; i++ ) {
                    if ( t < 1.0 )
                    {   motor_cmd.q_des[ i ]   = target1_q[ i % 3 ] * t + init_q[ i ] * ( 1 - t );}
                    else if ( t < 2.0 )
                    {   motor_cmd.q_des[ i ]   = target2_q[ i % 3 ] * ( t - 1 ) + target1_q[ i % 3 ] * ( 2 - t );}
                    else
                    {   state = 1;
                        stable_q[ i ] = target2_q[ i % 3 ];
                    }
                    motor_cmd.kp_des[ i ]  = 100;
                    motor_cmd.kd_des[ i ]  = 3;
                    motor_cmd.qd_des[ i ]  = 0;
                    motor_cmd.tau_des[ i ] = 0;
                }
            }
            if ((count++) % 1000 == 0) {
                printf("interval:---------%.4f-------------\n", robot_data.ctrl_topic_interval);
                printf("rpy [3]:");
                for (int i = 0; i < 3; i++)
                    printf(" %.2f", robot_data.rpy[i]);
                printf("\nacc [3]:");
                for (int i = 0; i < 3; i++)
                    printf(" %.2f", robot_data.acc[i]);
                printf("\nquat[4]:");
                for (int i = 0; i < 4; i++)
                    printf(" %.2f", robot_data.quat[i]);
                printf("\nomeg[3]:");
                for (int i = 0; i < 3; i++)
                    printf(" %.2f", robot_data.omega[i]);
                printf("\nq  [12]:");
                for (int i = 0; i < 12; i++)
                    printf(" %.2f", robot_data.q[i]);
                printf("\nqd [12]:");
                for (int i = 0; i < 12; i++)
                    printf(" %.2f", robot_data.qd[i]);
                printf("\ntau[12]:");
                for (int i = 0; i < 12; i++)
                    printf(" %.2f", robot_data.tau[i]);
                printf("\nctrl[12]:");
                for (int i = 0; i < 12; i++)
                    printf(" %.2f", motor_cmd.q_des[i]);
                printf("\n\n");
            }
        }

        if ( state == 1 ) {
            if (mode_transfer != 0) { state = 2; }  //transfer to joy_control mode
            else
                for (int i = 0; i < 12; i++)
                    motor_cmd.q_des[ i ] = stable_q[ i ];
        }

        if ( state == 2 ){
            if ( mode_transfer == 0 )
                state = 1;
            else
                for (int i = 0; i < 12; i++){
                    motor_cmd.q_des[ i ]  = target.q_des[ i ];
                    stable_q[ i ] = target.q_des[ i ];}
        }
    }
}

//Callback Function for Subscriber
void SubPub::joy_control(const sensor_msgs::msg::Joy::SharedPtr msg) {
    //identify motor
    double cmd;
    int motor_id;
    int base = 0;
    int bias = 0;
    if (msg->buttons[3] == 1) { bias = 1; }
    if (msg->buttons[4] == 1) { bias = 2; }
    motor_id = base + bias;
    //control, only angle now, not for angular velocity
    if (msg->buttons[0]) {
        mode_transfer = 1;
        cmd = io->robot_data.q[motor_id] + msg->axes[0] * 0.75;
        if (cmd > -0.75 && cmd < 0.75) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 0.75; }
    }
    if (msg->buttons[3]) {
        mode_transfer = 1;
        cmd = io->robot_data.q[motor_id] + msg->axes[0] * 2.81;
        if (cmd > -1.257 && cmd < 4.363) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 2.81 - 1.257; }
    }
    if (msg->buttons[4]) {
        mode_transfer = 1;
        cmd = io->robot_data.q[motor_id] + msg->axes[0] * 0.986;
        if (cmd > -2.478 && cmd < -0.506) { target.q_des[motor_id] = cmd; }
        else { target.q_des[motor_id] = cmd / abs(cmd) * 0.986 - 2.478; }
    }
}

void SubPub::pub_JointState()
{
    auto motor_data = sensor_msgs::msg::JointState();
    motor_data.name.resize(12);
    motor_data.position.resize(12);
    motor_data.velocity.resize(12);
    motor_data.effort.resize(12);
    for ( int i = 0; i < 12; i++ ) {
        motor_data.position[ i ] = io->robot_data.q[ i ];
        motor_data.velocity[ i ] = io->robot_data.qd[ i ];
        motor_data.effort[ i ] = io->robot_data.tau[ i ];
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: JointState");
    publisher_JointState->publish(state);
    }

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    signal( SIGINT, signal_callback_handler );
    io = std::make_shared< CustomInterface >( 500 );
    rclcpp::spin(std::make_shared< SubPub >(500));
    rclcpp::shutdown();
    return 0;
}
