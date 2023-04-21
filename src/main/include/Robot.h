// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"

#include "Ports.h"
#include "CustomController.h"

using namespace frc;
using namespace rev;

class Robot : public frc::TimedRobot {
 public:
  CustomController r_driver;
  CustomController r_operator;
  CANSparkMax r_left_front;
  CANSparkMax r_left_back;
  SparkMaxRelativeEncoder r_left_encoder = r_left_front.GetEncoder();
  SparkMaxPIDController r_left_pid = r_left_front.GetPIDController();
  CANSparkMax r_right_front;
  CANSparkMax r_right_back;
  SparkMaxRelativeEncoder r_right_encoder = r_right_front.GetEncoder();
  SparkMaxPIDController r_right_pid = r_right_front.GetPIDController();
  CANSparkMax r_arm;
  SparkMaxRelativeEncoder r_arm_encoder = r_arm.GetEncoder();
  SparkMaxPIDController r_arm_pid = r_arm.GetPIDController();
  CANSparkMax r_extension;
  SparkMaxRelativeEncoder r_extension_encoder = r_extension.GetEncoder();
  SparkMaxPIDController r_extension_pid = r_extension.GetPIDController();
  ADIS16470_IMU r_gyro;
  Compressor r_compressor;
  Solenoid r_box;
  Solenoid r_low_grabber;
  Solenoid r_high_grabber;
  DigitalInput r_arm_limit_low;
  DigitalInput r_arm_limit_high;
  DigitalInput r_extension_limit_back;
  DigitalInput r_extension_limit_front;
  Timer timer1;
  Timer timer2;
  SendableChooser<std::string> r_auto_mode;

  Robot() :
  r_driver(Ports::driver),
  r_operator(Ports::r_operator),
  r_left_front(Ports::left_front, CANSparkMax::MotorType::kBrushless),
  r_left_back(Ports::left_back, CANSparkMax::MotorType::kBrushless),
  r_right_front(Ports::right_front, CANSparkMax::MotorType::kBrushless),
  r_right_back(Ports::right_back, CANSparkMax::MotorType::kBrushless),
  r_arm(Ports::arm, CANSparkMax::MotorType::kBrushless),
  r_extension(Ports::extension, CANSparkMax::MotorType::kBrushless),
  r_compressor(Ports::PH, PneumaticsModuleType::REVPH),
  r_box(Ports::PH, PneumaticsModuleType::REVPH, Ports::box),
  r_low_grabber(Ports::PH, PneumaticsModuleType::REVPH, Ports::low_grabber),
  r_high_grabber(Ports::PH, PneumaticsModuleType::REVPH, Ports::high_grabber),
  r_arm_limit_low(Ports::arm_limit_low),
  r_arm_limit_high(Ports::arm_limit_high),
  r_extension_limit_back(Ports::extension_limit_back),
  r_extension_limit_front(Ports::extension_limit_front)
  {}

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  
  //Custom functions and trackers
  void stop_all();
  void drivetrain();
  void arm();
  void extension();
  void box();
  void grabber();
  void grabber_open();
  void grabber_close_low();
  void grabber_close_high();
  int auto_state;
  double left_hold;
  double right_hold;
  double arm_hold;
  double extension_hold;
  int arm_state;
  int extension_state;

  private:
};
