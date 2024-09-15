#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <utilities/SwerveSteerIO.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <memory>
#include <numbers>

class SwerveModule {
 public:
  SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID, double cancoderMagOffset); 
  void SetDesiredState(const frc::SwerveModuleState& state);
  void SyncSensors();
  void SendSensorsToDash();
  void SetDesiredAngle(units::degree_t angle);
  void SetDesiredVelocity(units::meters_per_second_t velocity);
  void DriveStraightVolts(units::volt_t volts);
  void StopMotors();
  void UpdateSim(units::second_t deltaTime);
  void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode);
  void ConfigSteerMotor();
  frc::SwerveModulePosition GetPosition();
  frc::Rotation2d GetAngle();
  frc::Rotation2d GetCanCoderAngle();
  units::meters_per_second_t GetSpeed();
  units::volt_t GetDriveVoltage();
  frc::SwerveModuleState GetState();
  units::radian_t GetDrivenRotations();

  static constexpr double STEER_GEAR_RATIO = 150.0/7.0;
  static constexpr double DRIVE_GEAR_RATIO = 5.9; // L2 - Fast kit
  static constexpr units::meter_t WHEEL_RADIUS = 0.0481098886_m;
  static constexpr units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;


  static constexpr double DRIVE_P = 1.6998; 
  static constexpr double DRIVE_I = 0.0;
  static constexpr double DRIVE_D = 0.0;
  static constexpr double DRIVE_F = 0;
  static constexpr double DRIVE_S = 0.3017;  // Units is V
  static constexpr double DRIVE_V = 0.65;  // Units is V/1m/s      //MAKE SURE TO TUNE ABOVE 12.5 VOLTS
  static constexpr double DRIVE_A = 0.34324; // Units is V/1m/s^2
 
 private:
  ctre::phoenix6::hardware::TalonFX _canDriveMotor;
  std::unique_ptr<SwerveSteerIO> _steerMotorIO;
  ctre::phoenix6::hardware::CANcoder _cancoder;

  ctre::phoenix6::configs::TalonFXConfiguration _configCanDriveMotor{};
  ctre::phoenix6::configs::CANcoderConfiguration _configCancoder{};

  frc::sim::DCMotorSim _driveMotorSim{frc::DCMotor::Falcon500(), DRIVE_GEAR_RATIO, 0.05_kg_sq_m};
};
