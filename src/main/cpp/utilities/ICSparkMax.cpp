#include "utilities/ICSparkMax.h"

ICSparkMax::ICSparkMax(int deviceID, units::ampere_t currentLimit)
    : ICSparkBase(this, currentLimit),
      rev::CANSparkMax(deviceID, rev::CANSparkLowLevel::MotorType::kBrushless) {
}

void ICSparkMax::Set(double speed) { ICSparkBase::SetDutyCycle(speed); }

void ICSparkMax::SetVoltage(units::volt_t output) {
  ICSparkBase::SetVoltage(output);
}

void ICSparkMax::StopMotor() { ICSparkBase::Stop(); }