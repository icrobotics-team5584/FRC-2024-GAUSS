#include "utilities/ICSparkMax.h"

ICSparkMax::ICSparkMax(int deviceID, units::ampere_t currentLimit)
    : rev::CANSparkMax(deviceID, rev::CANSparkLowLevel::MotorType::kBrushless),
      ICSparkBase(this, currentLimit) {}

void ICSparkMax::Set(double speed) { ICSparkBase::SetDutyCycle(speed); }

void ICSparkMax::SetVoltage(units::volt_t output) {
  ICSparkBase::SetVoltage(output);
}

void ICSparkMax::StopMotor() { ICSparkBase::Stop(); }