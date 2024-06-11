#pragma once

#include <rev/CANSparkMax.h>

#include "utilities/ICSparkBase.h"

/**
 * Wrapper around the Rev CANSparkMax class with some convenience features.
 * - Brushless motor type assumed
 * - Required Current Limit Setting
 * - Better simulation support (see ICSparkBase::GetSimVoltage() and ICSparkBase::UpdateSimEncoder())
 * - Uses C++ units
 * - Encoder and pid functions are built into this class
 */
class ICSparkMax : public rev::CANSparkMax, public ICSparkBase {
 public:
  ICSparkMax(int deviceID, units::ampere_t currentLimit);

  /**
   * Stop the motor until Set is called again or closed loop control is started.
   */
  void StopMotor() override;

  /**
   * Sets the duty cycle of a speed controller.
   *
   * @param speed The duty cycle to set. Value should be between -1.0 and 1.0.
   */
  void Set(double speed) override;

  /**
   * Sets the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output) override;
};