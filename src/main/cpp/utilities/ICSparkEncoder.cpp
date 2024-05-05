#include <Utilities/ICSparkEncoder.h>
#include <frc/RobotBase.h>
#include <utility>
#include <frc/smartdashboard/SmartDashboard.h>

ICSparkEncoder::ICSparkEncoder(rev::SparkRelativeEncoder&& inbuilt)
    : _inbuilt(std::move(inbuilt)) {}

double ICSparkEncoder::GetPosition() {
  switch (_selected) {
    case ABSOLUTE:
      return frc::RobotBase::IsSimulation() ? _absoluteSimPos : _absolute->GetPosition();
    case ALTERNATE:
      return _alternate->GetPosition();
    case INBUILT:
    default:
      return _inbuilt.GetPosition();
  }
}

double ICSparkEncoder::GetVelocity() {
  switch (_selected) {
    case ABSOLUTE:
    frc::SmartDashboard::PutNumber("arm/Encoder velocity", _absolute->GetVelocity());
      return _absolute->GetVelocity();
    case ALTERNATE:
      return _alternate->GetVelocity();
    case INBUILT:
    default:
      return _inbuilt.GetVelocity();
  }
}

void ICSparkEncoder::SetPosition(double pos) {
  if(_alternate){
    _alternate->SetPosition(pos);
  }
  _inbuilt.SetPosition(pos);
  _absoluteSimPos = pos;
}

void ICSparkEncoder::SetConversionFactor(double rotationsToDesired) {

  if(_absolute){
     _absolute->SetPositionConversionFactor(rotationsToDesired);
     _absolute->SetVelocityConversionFactor(rotationsToDesired);
  }
  // Need to divide vel by 60 because Spark Max uses Revs per minute not Revs per second
  if(_alternate){
    _alternate->SetPositionConversionFactor(rotationsToDesired);
    _alternate->SetVelocityConversionFactor(rotationsToDesired / 60);
  }

  _inbuilt.SetPositionConversionFactor(rotationsToDesired);
  _inbuilt.SetVelocityConversionFactor(rotationsToDesired / 60);
}

void ICSparkEncoder::UseAbsolute(rev::SparkAbsoluteEncoder&& encoder) {
  _selected = ABSOLUTE;
  _absolute = std::make_unique<rev::SparkAbsoluteEncoder>(encoder);
  
  SetConversionFactor(_inbuilt.GetPositionConversionFactor());
}

void ICSparkEncoder::UseAlternate(rev::SparkMaxAlternateEncoder&& encoder) {
  _selected = ALTERNATE;
  _alternate = std::make_unique<rev::SparkMaxAlternateEncoder>(encoder);
  SetConversionFactor(_inbuilt.GetPositionConversionFactor());
}

rev::SparkRelativeEncoder& ICSparkEncoder::GetInbuilt() {
  return _inbuilt;
}
rev::SparkAbsoluteEncoder& ICSparkEncoder::GetAbsolute() {
  return *_absolute;
}
rev::SparkMaxAlternateEncoder& ICSparkEncoder::GetAlternate() {
  return *_alternate;
}