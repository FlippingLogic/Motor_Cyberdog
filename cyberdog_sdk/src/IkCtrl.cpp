//
// Created by ysy on 7/11/22.
//

#include <chrono>
#include <memory>
#include <CustomInterface.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <stdio.h>
#include <math.h>

using namespace std::chrono_literals;

const int reduction_ratio = 1;                                       // Reduction ratio of the actuator
const float calf = 170, thigh = 170, shoulder = 85;      // Segment length of the leg parts, shoulder is the offset between the shoulder motor and the axis of rotation
float center_position[12] = {-0.0003,-0.0002,-1.693,
                             -8.727e-05,-0.0002,-1.693,
                             -0.0003,-0.0002,-1.693,
                             -8.727e-05,-0.0002,-1.693};


 class IK_node: public rclcpp::Node
 {
 public:
    IK_node();
 private:
    float target_X(Leg *leg, float height, float distX);
    float target_Y(Leg *leg, float height, float posY);
    void  target_Z(Leg *leg, float height);
    void  ik_compute(Leg *leg, float pos_z, float pos_x, float pos_y);
    void  ik_manager();
    void  manipulate_ctrl();
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
 };

IK_node::IK_node(): Node("IK_node") {
    publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    timer = this->create_wall_timer(2ms,std::bind(&IK_node::ik_manager, this));
}



// Calculates the angle of the calf actuator as well as part of the angle of the hip actuator
// Takes a pointer to the struct that cointains the angles for the leg
void IK_node::target_Z(Leg *leg, float height){
  float thetaZ = ( PI/2 - acos( calf*calf + height*height - thigh*thigh) / ( 2*calf*height ) ) * reduction_ratio;
  float phiZ = ( acos( calf*calf + thigh*thigh - height*height) / ( 2*calf*thigh ) )  * reduction_ratio;
  float *theta = &(*leg).theta;
  *theta -= thetaZ;
  float *phi = &(*leg).phi;
  *phi = -phiZ;
}
// Calculates part of the angle of the hip actuator as well as the "Leg length" considering the offset cause by inputing a non 0 target_X coordinate
float IK_node::target_X(Leg *leg, float height, float distX){
  float extraTheta = ( atan( distX / height ) );
  float thetaX = extraTheta * reduction_ratio;
  float newLegLength = ( height / (cos(extraTheta)) );
  //newLegLength = heightRestriction(abs(newLegLength));
  float *theta = &(*leg).theta;
  *theta = thetaX;
  return newLegLength;
}
// Calculates the hip angle
float IK_node::target_Y(Leg *leg, float height, float posY){
  float distY = shoulder + posY;
  float gammaP = atan( distY / height );
  if (isnan(gammaP)) gammaP = PI/2;
  float hipHyp = distY / sin( gammaP );
  float lambda = asin (shoulder / hipHyp );
  float gammaY = ( (  - lambda ) + gammaP  ) * reduction_ratio;
  float newNewLegLength = shoulder/tan(lambda);
  //newNewLegLength = heightRestriction(abs(newNewLegLength));
  float *gamma = &(*leg).gamma;
  *gamma = gammaY;
  return newNewLegLength;
}
void IK_node::ik_compute(Leg *leg, float pos_z, float pos_x, float pos_y){
  target_Z(leg, target_X(leg, target_Y(leg, pos_z, pos_y), pos_x));
}



void IK_node::ik_manager(){
  Leg legs[] = { {.id = 0},
                 {.id = 1},
                 {.id = 2},
                 {.id = 3} };
  int leg_amount = sizeof(legs)/sizeof(legs[0]);
  ik_compute(&legs[0], 10, 10, 5);
  ik_compute(&legs[1], 10, 20, 5);
  ik_compute(&legs[2], 10, 2, 15);
  ik_compute(&legs[3], 10, 0, 0);
  // for (int i = 0; i < leg_amount; i++){
  //   printf("ID: %d, Theta: %f, Phi: %f, Gamma: %f \n", legs[i].id, legs[i].theta, legs[i].phi, legs[i].gamma);
  // }
  auto ik_joint = sensor_msgs::msg::JointState();
  ik_joint.name.resize(12);
  ik_joint.position.resize(12);
  ik_joint.header.stamp = now();
  for (int i = 0; i < 12; i++) {
      ik_joint.name[ i ] = builtin_name[ i ];
      if ( i % 3 == 0) { ik_joint.position[ i ] = legs[ i / 4 ].theta; }
      if ( i % 3 == 1) { ik_joint.position[ i ] = legs[ i / 4 ].phi; }
      if ( i % 3 == 2) { ik_joint.position[ i ] = legs[ i / 4 ].gamma; }
  }
  publisher->publish(ik_joint);
}



void IK_node::manipulate_ctrl() {
  auto ik_joint = sensor_msgs::msg::JointState();
  ik_joint.name.resize(12);
  ik_joint.position.resize(12);
  ik_joint.header.stamp = now();
  for (int i = 0; i < 12; i++){
    ik_joint.name[ i ] = builtin_name[ i ];
    ik_joint.position[ i ] = center_position[ i ];
  }
  publisher->publish(ik_joint);
}



int main(int argc, char * argv[]){
  rclcpp::init( argc, argv );
  rclcpp::spin(std::make_shared< IK_node >());
  std::cout << "shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}