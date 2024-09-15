#include "utilities/SwerveSteerIO.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "utilities/SwerveModule.h"

SwerveSteerIO::SwerveSteerIO()
    : _motorSim(frc::DCMotor::KrakenX60FOC(), SwerveModule::STEER_GEAR_RATIO,
                0.000000001_kg_sq_m) {}

SwerveSteerTalonFXIO::SwerveSteerTalonFXIO(int canID) : _talon(canID) {}

void SwerveSteerTalonFXIO::Config() {}

void SwerveSteerTalonFXIO::SetAngle(units::turn_t target) {}

void SwerveSteerTalonFXIO::SetTarget(units::turn_t target) {}

void SwerveSteerTalonFXIO::Stop() {}

void SwerveSteerTalonFXIO::SetBreakMode() {}

void SwerveSteerTalonFXIO::SetCoastMode() {}

std::string SwerveSteerTalonFXIO::GetDeviceID() { return 0; }

units::turn_t SwerveSteerTalonFXIO::GetTarget() { return units::turn_t(0); }

units::turn_t SwerveSteerTalonFXIO::GetError() { return units::turn_t(0); }

units::turn_t SwerveSteerTalonFXIO::GetAngle() { return units::turn_t(0); }

units::turns_per_second_t SwerveSteerTalonFXIO::GetVelocity() { return 0_tps; }

void SwerveSteerTalonFXIO::Simulate(units::second_t deltaTime) {}

SwerveSteerSparkMaxIO::SwerveSteerSparkMaxIO(int canID) : _spark(canID, 40_A) {
  frc::SmartDashboard::PutData("swerve/turn motor " + std::to_string(canID),
                               (wpi::Sendable*)&_spark);
}

void SwerveSteerSparkMaxIO::Config() {
  _spark.SetConversionFactor(1.0 / SwerveModule::STEER_GEAR_RATIO);
  _spark.EnableClosedLoopWrapping(0_tr, 1_tr);
  _spark.SetPIDFF(SwerveModule::STEER_P, SwerveModule::STEER_I,
                  SwerveModule::STEER_D);
  _spark.SetInverted(true);
  _spark.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void SwerveSteerSparkMaxIO::SetAngle(units::turn_t angle) {
  _spark.SetCANTimeout(500);
  int attempt = 0;
  units::turn_t tolerance = 0.01_tr;

  while (units::math::abs(_spark.GetPosition() - angle) > tolerance &&
         attempt < 30) {
    _spark.SetPosition(angle);
    attempt++;
  }
  if (units::math::abs(_spark.GetPosition() - angle) > tolerance) {
    frc::SmartDashboard::PutString("swerve/errors/steer motor " + GetDeviceID(),
                                   "failed to set angle");
  }

  _spark.SetCANTimeout(10);
}

void SwerveSteerSparkMaxIO::SetTarget(units::turn_t target) {
  _spark.SetPositionTarget(target);
}

void SwerveSteerSparkMaxIO::Stop() { _spark.StopMotor(); }

void SwerveSteerSparkMaxIO::SetBreakMode() {
  _spark.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

void SwerveSteerSparkMaxIO::SetCoastMode() {
  _spark.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
}

std::string SwerveSteerSparkMaxIO::GetDeviceID() {
  return std::to_string(_spark.GetDeviceId());
}

units::turn_t SwerveSteerSparkMaxIO::GetTarget() {
  return _spark.GetPositionTarget();
}

units::turn_t SwerveSteerSparkMaxIO::GetError() { return _spark.GetPosError(); }

units::turn_t SwerveSteerSparkMaxIO::GetAngle() { return _spark.GetPosition(); }

units::turns_per_second_t SwerveSteerSparkMaxIO::GetVelocity() {
  return _spark.GetVelocity();
}

void SwerveSteerSparkMaxIO::Simulate(units::second_t deltaTime) {
  auto turnVolts = _spark.GetSimVoltage();
  _motorSim.SetInputVoltage(turnVolts);
  _motorSim.Update(deltaTime);
  auto turnAngle = _motorSim.GetAngularPosition();
  auto turnVelocity = _motorSim.GetAngularVelocity();
  _spark.UpdateSimEncoder(turnAngle, turnVelocity);
}