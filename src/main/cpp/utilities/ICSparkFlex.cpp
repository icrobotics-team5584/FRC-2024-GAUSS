#include "utilities/ICSparkFlex.h"

ICSparkFlex::ICSparkFlex(int deviceID, units::ampere_t currentLimit)
    : rev::CANSparkFlex(deviceID, rev::CANSparkLowLevel::MotorType::kBrushless),
      ICSpark(std::shared_ptr<rev::CANSparkBase>(this), CANSparkFlex::GetEncoder(), currentLimit) {}

void ICSparkFlex::Set(double speed) {
  ICSpark::SetDutyCycle(speed);
}

void ICSparkFlex::SetVoltage(units::volt_t output) {
  ICSpark::SetVoltage(output);
}

double ICSparkFlex::Get() const {
  return ICSpark::GetDutyCycle();
}

void ICSparkFlex::StopMotor() {
  ICSpark::StopMotor();
}

void ICSparkFlex::UseExternalEncoder(int countsPerRev) {
  ICSpark::UseRelativeEncoder(CANSparkFlex::GetExternalEncoder(countsPerRev));
}