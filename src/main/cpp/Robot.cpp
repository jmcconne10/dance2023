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
void Robot::AutonomousPeriodic() {
  /*
  This section is just dedicated to initializing some variables for how the auto code will run.
  The benefit of putting the settings up here is that they are easy and fast to tweak.
  */
  double arm_place;
  double extension_place;
  double initial_back;
  if(r_auto_mode.GetSelected() == "Charge Station") {
    arm_place = -75;
    extension_place = -280;
    initial_back = 0;
  }
  else {
    arm_place = -80;
    extension_place = -300;
    initial_back = 1;
  }
  double drive_speed = 0.4;
  double full_back = 75;
  double charge_back = 38;
  double balance_speed = 0.1;
  units::degree_t balance_zone = 3_deg;
  double balance_rate = 5;

  /*
  This switch statement is where the actual auto runs.
  Depending on what auto_state is, it will only run 1 of the steps.
  Each step is under a "case" statement.
  So only one of the sections under a case statement will be running at a time.

  Step descriptions:
  0 - Pull up the extension and drive back a little. If grabbing a cone, grab with high pressure.
  1 - Raise the arm
  2 - Move the extension out
  3 - If placing a cone, move the arm down a little bit.
  4 - Open the grabber
  5 - Drive back a set amount (either onto the charge station or out of the community).
      While this is happening, pull back the extension and lower the arm.
  6 - If going on the charge station, run the balancing routine.
  */
  switch(auto_state) {
    case 0:
      /*
      1 of the 2 ways that a motor can be run is through its PID controller.
      This is useful for setting a motor to go to an exact position.
      In the statement below, you can see that I have the extension motor set to go to the 0 position by using its PID controller.
      The SetReference() command will run the motor to a specified number of MOTOR rotations from the 0 point.
      */
      r_extension_pid.SetReference(0, ControlType::kPosition);

      /*
      We had 2 auto routines set for competition.
      There was a charge station auto and a non charge station auto.
      The way I told the robot which auto to run was by putting a selector in Shuffleboard.
      r_auto_mode is the selector and I can get which option is selected by using the GetSelected() command.
      */
      if(r_auto_mode.GetSelected() == "No Charge Station") {
        grabber_close_high();
      }

      /*
      The second way that a motor can be run is by directly setting it to a power with the Set() command.
      The command takes a power from -1 to 1.
      */
      r_left_front.Set(drive_speed);
      r_right_front.Set(drive_speed);
      if(r_left_encoder.GetPosition() >= initial_back) {
        r_left_front.Set(0);
        r_right_front.Set(0);

        /*
        At some point in the auto step is a trigger to go to the next step.
        After auto_state is incremented, this case statement will no longer run with each loop and instead the next one will run.*/
        auto_state++;
      }
      break;
    
    case 1:
      r_arm_pid.SetReference(arm_place, ControlType::kPosition);

      /*If I need to get the position that a motor is currently in, I can do so by using its built-in encoder.
      The GetPosition() command returns how many rotations the MOTOR has rotated from the 0 point.*/
      if(abs(r_arm_encoder.GetPosition() - arm_place) < 2) {
        auto_state++;
      }
      break;

    case 2:
      r_extension_pid.SetReference(extension_place, ControlType::kPosition);
      if(abs(r_extension_encoder.GetPosition() - extension_place) < 1) {
        auto_state++;
      }
      break;

    case 3:
      if(r_auto_mode.GetSelected() == "Charge Station") {
        auto_state++;
      }
      r_arm_pid.SetReference(-72, ControlType::kPosition);
      if(abs(r_arm_encoder.GetPosition() + 72) < 1) {
        auto_state++;
      }
      break;

    case 4:
      grabber_open();
      if(timer1.Get() == 0_s) {
        timer1.Start();
      }
      if(timer1.Get() > 0.5_s) {
        auto_state++;
      }
      break;
    
    case 5:
      r_extension_pid.SetReference(0, ControlType::kPosition);
      if(r_extension_encoder.GetPosition() > -20) {
        r_arm_pid.SetReference(0, ControlType::kPosition);
      }

      if((r_left_encoder.GetPosition() < charge_back && r_auto_mode.GetSelected() == "Charge Station")
      || (r_left_encoder.GetPosition() < full_back && r_auto_mode.GetSelected() == "No Charge Station")) {
        r_left_front.Set(drive_speed);
        r_right_front.Set(drive_speed);
      }
      else {
        r_left_front.Set(0);
        r_right_front.Set(0);
        if(r_auto_mode.GetSelected() == "Charge Station") {
          auto_state++;
        }
      }
      break;
    
    case 6:
      if(r_extension_encoder.GetPosition() > -20) {
        r_arm_pid.SetReference(0, ControlType::kPosition);
      }

      if(r_gyro.GetAngle() < -balance_zone && abs(double(r_gyro.GetRate())) < balance_rate) {
        r_left_front.Set(balance_speed);
        r_right_front.Set(balance_speed);
        left_hold = 0;
      }
      else if(r_gyro.GetAngle() > balance_zone && abs(double(r_gyro.GetRate())) < balance_rate) {
        r_left_front.Set(-balance_speed);
        r_right_front.Set(-balance_speed);
        left_hold = 0;
      }
      else {
        if(left_hold == 0) {
          left_hold = r_left_encoder.GetPosition();
          right_hold = r_right_encoder.GetPosition();
        }
        r_left_pid.SetReference(left_hold, ControlType::kPosition);
        r_right_pid.SetReference(right_hold, ControlType::kPosition);
      }
      break;
  }
}

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
  arm();
  extension();
  box();
  grabber();
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
