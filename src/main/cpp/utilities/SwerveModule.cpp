#include "utilities/SwerveModule.h"
#include "utilities/BotVars.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <iostream>

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID,
                           double cancoderMagOffset)
    : _canDriveMotor(canDriveMotorID),
      _cancoder(canTurnEncoderID) {
  using namespace ctre::phoenix6::signals;
  using namespace ctre::phoenix6::configs;
  

  // Config CANCoder
  _configCancoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Unsigned_0To1;
  _configCancoder.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
  _configCancoder.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _cancoder.GetConfigurator().Apply(_configCancoder);
  frc::SmartDashboard::PutNumber("swerve/cancoder "+std::to_string(canTurnEncoderID) + " mag offset", cancoderMagOffset);

  //Config Turn Motor
  if (BotVars::activeRobot == BotVars::PRACTICE) {
    _steerMotorIO = std::make_unique<SwerveSteerSparkMaxIO>(canTurnMotorID);
  } else {
    _steerMotorIO = std::make_unique<SwerveSteerTalonFXIO>(canTurnMotorID);
  }
  ConfigSteerMotor();
  SyncSensors();

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
  _configCanDriveMotor.CurrentLimits.SupplyCurrentLimit = 40.0;
  _configCanDriveMotor.CurrentLimits.SupplyCurrentThreshold = 60.0;
  _configCanDriveMotor.CurrentLimits.SupplyTimeThreshold = 0.1;
  _configCanDriveMotor.CurrentLimits.StatorCurrentLimit = 70.0; //Untested
  _configCanDriveMotor.Slot0.kS = DRIVE_S;
  _configCanDriveMotor.Slot0.kV = DRIVE_V;
  _configCanDriveMotor.Slot0.kA = DRIVE_A;
  _configCanDriveMotor.MotorOutput.NeutralMode = NeutralModeValue::Brake;
  _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::ConfigSteerMotor(){
  _steerMotorIO->Config();
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetState.angle.Degrees());
  SetDesiredVelocity(targetState.speed);
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
  std::string steerMotorName = "swerve/steer motor " + _steerMotorIO->GetDeviceID();
  std::string steerEncoderName = "swerve/steer encoder " + std::to_string(_cancoder.GetDeviceID());

  frc::SmartDashboard::PutNumber(driveMotorName + " Target velocity", _canDriveMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(driveMotorName + " velocity", _canDriveMotor.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber(steerMotorName  + " position", GetAngle().Degrees().value()/360.0);
  frc::SmartDashboard::PutNumber(steerMotorName  + " target", _steerMotorIO->GetTarget().value());
  frc::SmartDashboard::PutNumber(steerMotorName  + " error", _steerMotorIO->GetError().value());
  frc::SmartDashboard::PutNumber(steerEncoderName+ " Abs position", _cancoder.GetAbsolutePosition().GetValue().value());
  // clang-format on
}

frc::Rotation2d SwerveModule::GetAngle() {
  units::radian_t angle = _steerMotorIO->GetAngle();
  return angle;
}

frc::Rotation2d SwerveModule::GetCanCoderAngle() {
  units::radian_t angle = _cancoder.GetAbsolutePosition().GetValue();
  return angle;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  return (_canDriveMotor.GetVelocity().GetValue().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

units::volt_t SwerveModule::GetDriveVoltage() {
  return _canDriveMotor.GetMotorVoltage().GetValue();
}

units::radian_t SwerveModule::GetDrivenRotations() {
  return _canDriveMotor.GetPosition().GetValue();
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _steerMotorIO->SetTarget(angle);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  units::turns_per_second_t TurnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value()) * 1_tps;

  _canDriveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(TurnsPerSec)});
}

void SwerveModule::DriveStraightVolts(units::volt_t volts) {
  SetDesiredAngle(0_deg);
  _canDriveMotor.SetControl(ctre::phoenix6::controls::VoltageOut{volts});
}

void SwerveModule::StopMotors() {
  _canDriveMotor.Set(0);
  _steerMotorIO->Stop();
}

void SwerveModule::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  if (mode == ctre::phoenix6::signals::NeutralModeValue::Brake){
    _steerMotorIO->SetBreakMode();
  } else {
    _steerMotorIO->SetCoastMode();
  }
  //   _configCanDriveMotor.MotorOutput.NeutralMode = mode;
  //   _canDriveMotor.GetConfigurator().Apply(_configCanDriveMotor);
}

void SwerveModule::SyncSensors() {
  _steerMotorIO->SetAngle(_cancoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::UpdateSim(units::second_t deltaTime) {
  // Drive Motor
  auto& driveState = _canDriveMotor.GetSimState();
  _driveMotorSim.SetInputVoltage(driveState.GetMotorVoltage());
  _driveMotorSim.Update(deltaTime);
  driveState.SetRawRotorPosition(_driveMotorSim.GetAngularPosition() * DRIVE_GEAR_RATIO);
  driveState.SetRotorVelocity(_driveMotorSim.GetAngularVelocity() * DRIVE_GEAR_RATIO);

  // Steer Motor
  _steerMotorIO->Simulate(deltaTime);

  // CANcoders are attached directly to the mechanism, so don't account for the steer gearing
  auto& cancoderState = _cancoder.GetSimState();
  cancoderState.SetRawPosition(_steerMotorIO->GetAngle());
  cancoderState.SetVelocity(_steerMotorIO->GetVelocity());
}
