#include "utilities/ICSparkBase.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>

ICSpark::ICSpark(rev::CANSparkBase* spark,
                         rev::SparkRelativeEncoder&& inbuiltEncoder,
                         units::ampere_t currentLimit)
    : _spark(spark), _encoder{std::move(inbuiltEncoder)} {
  _spark->RestoreFactoryDefaults();
  _spark->SetSmartCurrentLimit(currentLimit.value());
  SetConversionFactor(1);  // Makes the internal encoder use revs per sec not revs per min

  _pidController.SetSmartMotionMinOutputVelocity(0);
  SetClosedLoopOutputRange(-1, 1);
}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
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

void ICSpark::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
  // auto err = GetLastError();
}

void ICSpark::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(ControlType::kPosition);

  _pidController.SetReference(target.value(),
                              rev::CANSparkLowLevel::ControlType::kPosition, 0,
                              _arbFeedForward.value());
}

void ICSpark::SetSmartMotionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(ControlType::kSmartMotion);

  _pidController.SetReference(target.value(),
                              rev::CANSparkLowLevel::ControlType::kSmartMotion,
                              0, _arbFeedForward.value());
}

void ICSpark::SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(ControlType::kMotionProfile);

  _pidController.SetReference(CalcMotionTarget().position.value(),
                              rev::CANSparkLowLevel::ControlType::kPosition, 0,
                              _arbFeedForward.value());
}

void ICSpark::SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(ControlType::kVelocity);

  _pidController.SetReference(target.value(),
                              rev::CANSparkLowLevel::ControlType::kVelocity, 0,
                              _arbFeedForward.value());
}

void ICSpark::SetDutyCycle(double speed) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(ControlType::kDutyCycle);
  _spark->Set(speed);
}

void ICSpark::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  SetInternalControlType(ControlType::kVoltage);

  _pidController.SetReference(output.value(),
                              rev::CANSparkLowLevel::ControlType::kVoltage, 0,
                              _arbFeedForward.value());
}

void ICSpark::UpdateControls(units::second_t loopTime) {
  switch (GetControlType()) {
    case ControlType::kMotionProfile:
      _arbFeedForward = CalculateFeedForward();
      _pidController.SetReference(CalcMotionTarget().position.value(),
                                  rev::CANSparkLowLevel::ControlType::kPosition,
                                  0, _arbFeedForward.value());
      break;
    case ControlType::kSmartMotion:
      SetSmartMotionTarget(_positionTarget, CalculateFeedForward());
      break;
    case ControlType::kPosition:
      SetPositionTarget(_positionTarget, CalculateFeedForward());
      break;
    case ControlType::kVelocity:
      SetVelocityTarget(_velocityTarget, CalculateFeedForward());
      break;
    default:
      break;
  }
}

void ICSpark::Stop() {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  SetInternalControlType(ControlType::kDutyCycle);
  _spark->StopMotor();
}

void ICSpark::SetInternalControlType(ControlType controlType) {
  _controlType = controlType;
}

rev::CANSparkLowLevel::ControlType ICSpark::GetREVControlType() {
  auto controlType = GetControlType();
  if (controlType == ControlType::kMotionProfile) {
    return rev::CANSparkLowLevel::ControlType::kPosition;
  } else {
    return (rev::CANSparkLowLevel::ControlType)controlType;
  }
}

void ICSpark::ConfigMotion(units::turns_per_second_t maxVelocity,
                                   units::turns_per_second_squared_t maxAcceleration,
                                   units::turn_t tolerance) {
  _pidController.SetSmartMotionMaxAccel(maxAcceleration.value());
  _pidController.SetSmartMotionMaxVelocity(maxVelocity.value());
  _pidController.SetSmartMotionAllowedClosedLoopError(tolerance.value());

  _motionProfile = frc::TrapezoidProfile<units::turns>{{maxVelocity, maxAcceleration}};
}

void ICSpark::SetConversionFactor(double rotationsToDesired) {
  _encoder.SetConversionFactor(rotationsToDesired);
}

void ICSpark::UseAbsoluteEncoder(units::turn_t zeroOffset) {
  auto absEncoder = _spark->GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
  absEncoder.SetAverageDepth(128); 
  _spark->SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 10);
  _spark->SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 10);
  absEncoder.SetZeroOffset(zeroOffset.value());
  _encoder.UseAbsolute(std::move(absEncoder));
  _pidController.SetFeedbackDevice(_encoder.GetPIDFeedbackDevice());
}

void ICSpark::EnableClosedLoopWrapping(units::turn_t min, units::turn_t max) {
  _pidController.SetPositionPIDWrappingMinInput(min.value());
  _pidController.SetPositionPIDWrappingMaxInput(max.value());
  _pidController.SetPositionPIDWrappingEnabled(true);
  _simController.EnableContinuousInput(min.value(), max.value());
}

void ICSpark::SetPIDFF(double P, double I, double D, double FF) {
  SetP(P);
  SetI(I);
  SetD(D);
  SetFF(FF);
}

void ICSpark::SetP(double P) {
  _pidController.SetP(P);
  _simController.SetP(P);
}

void ICSpark::SetI(double I) {
  _pidController.SetI(I);
  _simController.SetI(I);
}

void ICSpark::SetD(double D) {
  _pidController.SetD(D);
  _simController.SetD(D);
}

void ICSpark::SetFF(double FF) {
  _pidController.SetFF(FF);
  _simFF = FF;
}

void ICSpark::SetClosedLoopOutputRange(double minOutputPercent, double maxOutputPercent) {
  _minPidOutput = minOutputPercent;
  _maxPidOutput = maxOutputPercent;
  _pidController.SetOutputRange(minOutputPercent, maxOutputPercent);
}

units::turns_per_second_t ICSpark::GetVelocity() {
  if (frc::RobotBase::IsSimulation()) {
    return _simVelocity;
  } else {
    return units::turns_per_second_t{_encoder.GetVelocity()};
  }
}

units::volt_t ICSpark::GetSimVoltage() {
  units::volt_t output = 0_V;

  switch (_controlType) {
    case ControlType::kDutyCycle:
      output = units::volt_t{_spark->Get() * 12};
      break;

    case ControlType::kVelocity:
      // Spark internal PID uses native units (motor shaft RPM)
      // so divide by conversion factor to use that
      output = units::volt_t{_simController.Calculate(GetVelocity().value(),
                                                      _velocityTarget.value()) /
                                 _encoder.GetVelocityConversionFactor() +
                             _simFF * _velocityTarget.value()};
      break;

    case ControlType::kPosition:
      // Spark internal PID uses native units (motor shaft rotations)
      // so divide by conversion factor to use that
      output = units::volt_t{_simController.Calculate(GetPosition().value(),
                                                      _positionTarget.value()) /
                                 _encoder.GetPositionConversionFactor() +
                             _simFF * _positionTarget.value()};
      break;

    case ControlType::kVoltage:
      output = _voltageTarget;
      break;

    case ControlType::kSmartMotion:
      output = units::volt_t{
          _simController.Calculate(GetVelocity().value(),
                                   CalcMotionTarget().velocity.value()) /
              _encoder.GetVelocityConversionFactor() +
          _simFF * CalcMotionTarget().velocity.value()};
      break;

    case ControlType::kMotionProfile:
      output = units::volt_t{
          _simController.Calculate(GetPosition().value(),
                                   CalcMotionTarget().position.value()) /
              _encoder.GetPositionConversionFactor() +
          _simFF * _positionTarget.value()};
      break;

    case ControlType::kCurrent:
      break;
  }
  output += _arbFeedForward;
  return std::clamp(output, _minPidOutput * 12_V, _maxPidOutput * 12_V);
}

void ICSpark::UpdateSimEncoder(units::turn_t position, units::turns_per_second_t velocity) {
  _encoder.SetPosition(position.value());
  _simVelocity = velocity;
}

bool ICSpark::InMotionMode() {
  return GetControlType() == ControlType::kMotionProfile ||
         GetControlType() == ControlType::kSmartMotion;
}

ICSpark::MPState ICSpark::CalcMotionTarget(units::second_t lookahead) {
  units::turn_t error = units::math::abs(_positionTarget - GetPosition());
  units::turn_t tolerance = _pidController.GetSmartMotionAllowedClosedLoopError() * 1_tr;
  if (error < tolerance) {
    return MPState{GetPosition(), 0_tps};
  }

  return _motionProfile.Calculate(
      lookahead, {GetPosition(), GetVelocity()},
      {_positionTarget, units::turns_per_second_t{0}});
}