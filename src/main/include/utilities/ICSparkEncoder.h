#pragma once

#include <rev/CANSparkMax.h>

class ICSparkEncoder {
 public:
  ICSparkEncoder(rev::SparkRelativeEncoder&& inbuilt);
  enum EncoderType { INBUILT, ABSOLUTE, ALTERNATE };

  double GetPosition();
  double GetVelocity();
  void SetPosition(double pos);
  void SetConversionFactor(double rotationsToDesired);
  void UseAlternate(rev::SparkMaxAlternateEncoder&& encoder);
  void UseAbsolute(rev::SparkAbsoluteEncoder&& encoder);
  rev::SparkRelativeEncoder& GetInbuilt();
  rev::SparkAbsoluteEncoder& GetAbsolute();
  rev::SparkMaxAlternateEncoder& GetAlternate();

 private:
  rev::SparkRelativeEncoder _inbuilt;
  std::unique_ptr<rev::SparkAbsoluteEncoder> _absolute;
  std::unique_ptr<rev::SparkMaxAlternateEncoder> _alternate;
  double _absoluteSimPos = 0;
  EncoderType _selected = INBUILT;
};