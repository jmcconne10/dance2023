// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  r_driver.SetSquareScale(true);

  r_left_back.Follow(r_left_front);
  r_right_back.Follow(r_right_front);
  r_left_front.SetInverted(true);
  r_right_front.SetInverted(false);
  r_extension.SetInverted(true);

  r_left_pid.SetP(0.1);
  r_left_pid.SetI(0);
  r_left_pid.SetD(0);
  r_left_pid.SetFF(0);

  r_right_pid.SetP(0.1);
  r_right_pid.SetI(0);
  r_right_pid.SetD(0);
  r_right_pid.SetFF(0);

  r_arm_pid.SetP(0.07);
  r_arm_pid.SetI(0);
  r_arm_pid.SetD(0);
  r_arm_pid.SetFF(0);

  r_extension_pid.SetP(0.05);
  r_extension_pid.SetI(0);
  r_extension_pid.SetD(0);
  r_extension_pid.SetFF(0);

  r_compressor.EnableAnalog(60_psi, 120_psi);

  r_gyro.SetYawAxis(ADIS16470_IMU::IMUAxis::kY);

  CameraServer::StartAutomaticCapture();

  r_auto_mode.AddOption("Charge Station", "Charge Station");
  r_auto_mode.SetDefaultOption("No Charge Station", "No Charge Station");
  SmartDashboard::PutData("Auto Mode", &r_auto_mode);
}

void Robot::RobotPeriodic() {
  SmartDashboard::PutNumber("Left", r_left_encoder.GetPosition());
  SmartDashboard::PutNumber("Right", r_right_encoder.GetPosition());
  SmartDashboard::PutNumber("Arm", r_arm_encoder.GetPosition());
  SmartDashboard::PutNumber("Extension", r_extension_encoder.GetPosition());
  SmartDashboard::PutNumber("Gyro Angle", double(r_gyro.GetAngle()));
  SmartDashboard::PutNumber("Gyro Rate", double(r_gyro.GetRate()));
  frc::SmartDashboard::PutBoolean("bB : ", bB);
}

//This code only runs once at the beginning of autonomous
void Robot::AutonomousInit() {
  //Begin by resetting and rezeroing variables and encoders
  stop_all();
  auto_state = 0; //This variable here controls what step in the auto code to run. It starts at 0.
  left_hold = 0;
  r_left_encoder.SetPosition(0);
  r_right_encoder.SetPosition(0);
  r_gyro.Reset();
  timer1.Stop();
  timer1.Reset();
}

//This code runs in an infinite loop all throughout autonomous
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  stop_all();
  arm_state = 0;
  extension_state = 0;
  arm_hold = 0;
  extension_hold = 0;
  r_left_front.SetInverted(true);
  r_right_front.SetInverted(false);
}

void Robot::TeleopPeriodic() {

  drivetrain();
  stepForward();
  stepBackward();
  counterClockwise();
  clockwise();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  stop_all();
  while(r_extension_limit_back.Get() == 0) {
    r_extension.Set(0.2);
  }
  r_extension_encoder.SetPosition(0);
  while(r_extension_encoder.GetPosition() > -10) {
    r_extension.Set(-0.1);
  }
  while(r_extension_limit_back.Get() == 0) {
    r_extension.Set(0.05);
  }
  r_extension.Set(0);
  r_extension_encoder.SetPosition(0);
  while(r_arm_limit_low.Get() == 0) {
    r_arm.Set(0.2);
  }
  r_arm_encoder.SetPosition(0);
  while(r_arm_encoder.GetPosition() > -5) {
    r_arm.Set(-0.1);
  }
  while(r_arm_limit_low.Get() == 0) {
    r_arm.Set(0.02);
  }
  r_arm.Set(0);
  r_arm_encoder.SetPosition(0);
  r_arm_pid.SetReference(-2, ControlType::kPosition);
  r_extension_pid.SetReference(-45, ControlType::kPosition);
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
