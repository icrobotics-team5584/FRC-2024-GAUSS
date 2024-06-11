#include "utilities/ICSparkBase.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>

ICSparkBase::ICSparkBase(rev::CANSparkBase* spark, units::ampere_t currentLimit)
    : _spark(spark) {
  _spark->RestoreFactoryDefaults();
  _spark->SetSmartCurrentLimit(currentLimit.value());
  SetConversionFactor(1);  // Makes the internal encoder use revs per sec not revs per min

  _pidController.SetSmartMotionMinOutputVelocity(0);
  SetClosedLoopOutputRange(-1, 1);
}

void ICSparkBase::InitSendable(wpi::SendableBuilder& builder) {
  // clang-format off
  builder.AddDoubleProperty("Position", [&] { return GetPosition().value(); }, nullptr);  // setter is null, cannot set position directly
  builder.AddDoubleProperty("Velocity", [&] { return GetVelocity().value(); }, nullptr);
  builder.AddDoubleProperty("Position Target", [&] { return GetPositionTarget().value(); }, [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target", [&] { return GetVelocityTarget().value(); }, [&](double targ) { SetVelocityTarget(targ*1_tps); });

  builder.AddDoubleProperty("Voltage", [&] { 
        return (frc::RobotBase::IsSimulation()) 
          ? GetSimVoltage().value() 
          : _spark->GetAppliedOutput() * 12;
      }, nullptr);

  builder.AddDoubleProperty("P Gain", [&] { return _simController.GetP(); }, [&](double P) { SetP(P); });
  builder.AddDoubleProperty("I Gain", [&] { return _simController.GetI(); }, [&](double I) { SetI(I); });
  builder.AddDoubleProperty("D Gain", [&] { return _simController.GetD(); }, [&](double D) { SetD(D); });
  builder.AddDoubleProperty("FF Gain", [&] { return _simFF; }, [&](double FF) { SetFF(FF); });
  // clang-format on
}

void ICSparkBase::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
  // auto err = GetLastError();
}

void ICSparkBase::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kPosition);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkBase::SetSmartMotionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kSmartMotion);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkBase::SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(Mode::kVelocity);

  _pidController.SetReference(target.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkBase::SetDutyCycle(double speed) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(Mode::kDutyCycle);
  if (frc::RobotBase::IsSimulation()) {
    _pidController.SetReference(speed, Mode::kDutyCycle);
  }
  _spark->Set(speed);
}

void ICSparkBase::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  SetInternalControlType(Mode::kVoltage);

  _pidController.SetReference(output.value(), GetControlType(), 0, _arbFeedForward.value());
}

void ICSparkBase::Stop() {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(Mode::kDutyCycle);
  _spark->StopMotor();
}

void ICSparkBase::SetInternalControlType(Mode controlType) {
  _controlType = controlType;
}

void ICSparkBase::ConfigSmartMotion(units::turns_per_second_t maxVelocity,
                                   units::turns_per_second_squared_t maxAcceleration,
                                   units::turn_t tolerance) {
  _pidController.SetSmartMotionMaxAccel(maxAcceleration.value());
  _pidController.SetSmartMotionMaxVelocity(maxVelocity.value());
  _pidController.SetSmartMotionAllowedClosedLoopError(tolerance.value());

  _motionProfile = frc::TrapezoidProfile<units::turns>{{maxVelocity, maxAcceleration}};
}

void ICSparkBase::SetConversionFactor(double rotationsToDesired) {
  _encoder.SetConversionFactor(rotationsToDesired);
}

void ICSparkBase::UseAbsoluteEncoder(units::turn_t zeroOffset) {
  auto encoder = _spark->GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
  encoder.SetAverageDepth(128); 
  _spark->SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 10);
  _spark->SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 10);
  encoder.SetZeroOffset(zeroOffset.value());
  _encoder.UseAbsolute(std::move(encoder));
  _pidController.SetFeedbackDevice(_encoder.GetAbsolute());
}

void ICSparkBase::EnableClosedLoopWrapping(units::turn_t min, units::turn_t max) {
  _pidController.SetPositionPIDWrappingMinInput(min.value());
  _pidController.SetPositionPIDWrappingMaxInput(max.value());
  _pidController.SetPositionPIDWrappingEnabled(true);
  _simController.EnableContinuousInput(min.value(), max.value());
}

void ICSparkBase::SetPIDFF(double P, double I, double D, double FF) {
  SetP(P);
  SetI(I);
  SetD(D);
  SetFF(FF);
}

void ICSparkBase::SetP(double P) {
  _pidController.SetP(P);
  _simController.SetP(P);
}

void ICSparkBase::SetI(double I) {
  _pidController.SetI(I);
  _simController.SetI(I);
}

void ICSparkBase::SetD(double D) {
  _pidController.SetD(D);
  _simController.SetD(D);
}

void ICSparkBase::SetFF(double FF) {
  _pidController.SetFF(FF);
  _simFF = FF;
}

void ICSparkBase::SetClosedLoopOutputRange(double minOutputPercent, double maxOutputPercent) {
  _minPidOutput = minOutputPercent;
  _maxPidOutput = maxOutputPercent;
  _pidController.SetOutputRange(minOutputPercent, maxOutputPercent);
}

units::turns_per_second_t ICSparkBase::GetVelocity() {
  if (frc::RobotBase::IsSimulation()) {
    return _simVelocity;
  } else {
    return units::turns_per_second_t{_encoder.GetVelocity()};
  }
}

units::volt_t ICSparkBase::GetSimVoltage() {
  units::volt_t output = 0_V;

  switch (_controlType) {
    case Mode::kDutyCycle:
      output = units::volt_t{_spark->Get() * 12};
      break;

    case Mode::kVelocity:
      // Spark internal PID uses native units (motor shaft RPM)
      // so divide by conversion factor to use that
      output =
          units::volt_t{_simController.Calculate(GetVelocity().value(), _velocityTarget.value()) /
                            _encoder.GetVelocityConversionFactor() +
                        _simFF * _velocityTarget.value()};
      break;

    case Mode::kPosition:
      // Spark internal PID uses native units (motor shaft rotations)
      // so divide by conversion factor to use that
      output = units::volt_t{
          _simController.Calculate(GetPosition().value(),
                                   _positionTarget.value()) /
              _encoder.GetPositionConversionFactor() +
          _simFF * _positionTarget.value()};
      break;

    case Mode::kVoltage:
      output = _voltageTarget;
      break;

    case Mode::kSmartMotion:
      output = units::volt_t{
          _simController.Calculate(GetVelocity().value(), EstimateSMVelocity().value()) +
          _simFF * EstimateSMVelocity().value()};
      break;

    case Mode::kCurrent:
      break;

    case Mode::kSmartVelocity:
      break;
  }
  output += _arbFeedForward;
  return std::clamp(output, _minPidOutput * 12_V, _maxPidOutput * 12_V);
}

void ICSparkBase::UpdateSimEncoder(units::turn_t position, units::turns_per_second_t velocity) {
  _encoder.SetPosition(position.value());
  _simVelocity = velocity;
}

units::turns_per_second_t ICSparkBase::EstimateSMVelocity() {
  if (_controlType != Mode::kSmartMotion) {
    return units::turns_per_second_t{0};
  }

  units::turn_t error = units::math::abs(_positionTarget - GetPosition());
  units::turn_t tolerance = _pidController.GetSmartMotionAllowedClosedLoopError() * 1_tr;
  if (error < tolerance) {
    return units::turns_per_second_t{0};
  }

  return _motionProfile
      .Calculate(20_ms, {GetPosition(), GetVelocity()},
                 {_positionTarget, units::turns_per_second_t{0}})
      .velocity;
}