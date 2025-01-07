// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <iostream>
#include "utilities/RobotLogs.h"

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID,
                           units::turn_t cancoderMagOffset)
    : _canDriveMotor(canDriveMotorID),
      _canTurnMotor(canTurnMotorID, 40_A),
      _canTurnEncoder(canTurnEncoderID) {
  using namespace ctre::phoenix6::signals;
  using namespace ctre::phoenix6::configs;

  // Config CANCoder
  _configTurnEncoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;
  _configTurnEncoder.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
  _configTurnEncoder.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _canTurnEncoder.GetConfigurator().Apply(_configTurnEncoder);
  Logger::Log("swerve/cancoder "+std::to_string(canTurnEncoderID) + " mag offset", cancoderMagOffset);

  //Config Turn Motor
  ConfigTurnMotor();
  SyncSensors();
  frc::SmartDashboard::PutData("swerve/turn motor "+std::to_string(canTurnMotorID), &_canTurnMotor);

  // Config Driving Motor
  _canDriveMotor.GetConfigurator().Apply(TalonFXConfiguration{});
  _configCanDriveMotor.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RotorSensor;
  _configCanDriveMotor.ClosedLoopGeneral.ContinuousWrap = false;
  _configCanDriveMotor.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
  _configCanDriveMotor.Slot0.kP = DRIVE_P;
  _configCanDriveMotor.Slot0.kI = DRIVE_I;
  _configCanDriveMotor.Slot0.kD = DRIVE_D;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
  _configCanDriveMotor.CurrentLimits.StatorCurrentLimitEnable = true;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLimit = 60.0_A;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLowerLimit = 40.0_A;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLowerTime = 1_s;
  _configCanDriveMotor.CurrentLimits.StatorCurrentLimit = 70.0_A;
  _configCanDriveMotor.Slot0.kS = DRIVE_S;
  _configCanDriveMotor.Slot0.kV = DRIVE_V;
  _configCanDriveMotor.Slot0.kA = DRIVE_A;
  _configCanDriveMotor.MotorOutput.NeutralMode = NeutralModeValue::Brake;
  _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::ConfigTurnMotor(){
  // Config Turning Motor
  rev::spark::SparkBaseConfig config;
  config.encoder
    .PositionConversionFactor(1.0 / TURNING_GEAR_RATIO)
    .VelocityConversionFactor(TURNING_GEAR_RATIO / 60.0);
  config.closedLoop
    .P(TURN_P)
    .I(TURN_I)
    .D(TURN_D)
    .PositionWrappingEnabled(true)
    .PositionWrappingMinInput(0)
    .PositionWrappingMaxInput(1);
  config
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  
  _canTurnMotor.AdjustConfig(config);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState referenceState, 
                                    units::newton_t xForceFF,
                                    units::newton_t yForceFF
                                    ) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto currentAngle = GetAngle();
  referenceState.Optimize(currentAngle);

  // Find component of force feedforward that matches the direction of our wheel
  namespace m = units::math;
  units::turn_t forceFFAngle = m::atan(yForceFF / xForceFF);
  units::turn_t angleError = forceFFAngle - currentAngle.Radians();
  units::newton_t forceFFNorm = m::sqrt(m::pow<2>(xForceFF) + m::pow<2>(yForceFF));
  units::newton_t scaledForceFF = forceFFNorm * m::cos(angleError);

  // Make sure force feedforward has the same sign as the velocity
  scaledForceFF = m::copysign(scaledForceFF, referenceState.speed);

  // Slow down drive speed when not pointing the right way. This results in smoother driving.
  referenceState.CosineScale(currentAngle);

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(referenceState.angle.Degrees());
  SetDesiredVelocity(referenceState.speed, scaledForceFF);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  auto wheelRot = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      _canDriveMotor.GetPosition(), _canDriveMotor.GetVelocity());
  units::meter_t distance = (WHEEL_CIRCUMFERENCE.value() * wheelRot.value()) * 1_m;
  return {distance, GetAngle()};
}

void SwerveModule::SendSensorsToDash() {
  // clang-format off
  std::string driveMotorName = "swerve/drive motor " + std::to_string(_canDriveMotor.GetDeviceID());
  std::string turnMotorName = "swerve/turn motor " + std::to_string(_canTurnMotor.GetDeviceId());
  std::string turnEncoderName = "swerve/turn encoder " + std::to_string(_canTurnEncoder.GetDeviceID());

  Logger::LogFalcon(driveMotorName, _canDriveMotor);
  Logger::Log(turnEncoderName+ " Abs position", _canTurnEncoder.GetAbsolutePosition());
  // clang-format on
}

frc::Rotation2d SwerveModule::GetAngle() {
  units::radian_t turnAngle = _canTurnMotor.GetPosition();
  return turnAngle;
}

frc::Rotation2d SwerveModule::GetCanCoderAngle() {
  units::radian_t tAngle = _canTurnEncoder.GetAbsolutePosition().GetValue();
  return tAngle;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  return (_canDriveMotor.GetVelocity().GetValue().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

frc::SwerveModuleState SwerveModule::GetCANCoderState() {
  return {GetSpeed(), GetCanCoderAngle()};
}

units::volt_t SwerveModule::GetDriveVoltage() {
  return _canDriveMotor.GetMotorVoltage().GetValue();
}

units::radian_t SwerveModule::GetDrivenRotations() {
  return _canDriveMotor.GetPosition().GetValue();
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _canTurnMotor.SetPositionTarget(angle);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity,
                                      units::newton_t forceFF) {
  units::newton_meter_t torque = forceFF * WHEEL_RADIUS;
  units::ampere_t torqueCurrentFF = _driveMotorModel.Current(torque);
  Logger::Log("swerve/drive " + std::to_string(_canDriveMotor.GetDeviceID()) + " torqueCurrent", torqueCurrentFF);
  torqueCurrentFF *= Logger::Tune("swerve/currentFF enabled", true);
  units::turns_per_second_t wheelVelocity{velocity.value() / WHEEL_CIRCUMFERENCE.value()};
  _canDriveMotor.SetControl(
      ctre::phoenix6::controls::VelocityTorqueCurrentFOC{wheelVelocity}.WithFeedForward(
          torqueCurrentFF));
}

void SwerveModule::DriveStraightVolts(units::volt_t volts) {
  SetDesiredAngle(0_deg);
  _canDriveMotor.SetControl(ctre::phoenix6::controls::VoltageOut{volts});
}

void SwerveModule::StopMotors() {
  _canDriveMotor.Set(0);
  _canTurnMotor.Set(0);
}

void SwerveModule::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  //   _configCanDriveMotor.MotorOutput.NeutralMode = mode;
  //   _canTurnMotor.
  //   _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SyncSensors() {
  _canTurnMotor.SetCANTimeout(500);
  units::turn_t truePos = _canTurnEncoder.GetAbsolutePosition().GetValue();
  int maxAttempts = 15;
  int currentAttempts = 0;
  units::turn_t tolerance = 0.01_tr;

  while (units::math::abs(_canTurnMotor.GetPosition() - truePos) > tolerance &&
         currentAttempts < maxAttempts) {
    _canTurnMotor.SetPosition(truePos);
    currentAttempts++;
  }

  currentAttempts = 0;
  _canTurnMotor.SetCANTimeout(10);
}

void SwerveModule::UpdateSim(units::second_t deltaTime) {
  // Drive Motor
  auto& driveState = _canDriveMotor.GetSimState();
  _driveMotorSim.SetInputVoltage(driveState.GetMotorVoltage());
  _driveMotorSim.Update(deltaTime);
  driveState.SetRawRotorPosition(_driveMotorSim.GetAngularPosition() * DRIVE_GEAR_RATIO);
  driveState.SetRotorVelocity(_driveMotorSim.GetAngularVelocity() * DRIVE_GEAR_RATIO);
  driveState.SetRotorAcceleration(_driveMotorSim.GetAngularAcceleration() * DRIVE_GEAR_RATIO);
  // Turn Motor
  auto turnVolts = _canTurnMotor.CalcSimVoltage();
  _turnMotorSim.SetInputVoltage(turnVolts);
  _turnMotorSim.Update(deltaTime);
  _canTurnMotor.IterateSim(_turnMotorSim.GetAngularVelocity());

  // CANcoders are attached directly to the mechanism, so don't account for the steer gearing
  auto& cancoderState = _canTurnEncoder.GetSimState();
  cancoderState.SetRawPosition(_turnMotorSim.GetAngularPosition());
  cancoderState.SetVelocity(_turnMotorSim.GetAngularVelocity());
}
