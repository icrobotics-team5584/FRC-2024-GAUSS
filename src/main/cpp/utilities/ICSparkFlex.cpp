#include "utilities/ICSparkFlex.h"

ICSparkFlex::ICSparkFlex(int deviceID, units::ampere_t currentLimit)
    : rev::CANSparkFlex(deviceID, rev::CANSparkLowLevel::MotorType::kBrushless),
      ICSparkBase(this, currentLimit) {}

void ICSparkFlex::Set(double speed) { ICSparkBase::SetDutyCycle(speed); }

void ICSparkFlex::SetVoltage(units::volt_t output) {
  ICSparkBase::SetVoltage(output);
}

void ICSparkFlex::StopMotor() { ICSparkBase::Stop(); }