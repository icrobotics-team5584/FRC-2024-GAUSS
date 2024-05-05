#pragma once

#include <frc/Notifier.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <hal/simulation/SimDeviceData.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/current.h>
#include <units/velocity.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <utilities/ICSparkEncoder.h>

/**
 * Wrapper around the Rev CANSparkMax class with some convenience features.
 * - Brushless motor type assumed
 * - 20 Amp default current limit
 * - Better simulation support (see GetSimVoltage() and UpdateSimEncoder())
 * - Uses C++ units
 * - Encoder and pid functions are built into this class
 */
class ICSparkMax : public rev::CANSparkMax, wpi::Sendable {
 public:
  /**
   * Create a new object to control a SPARK MAX motor controller, with added convenience features.
   *
   * @param deviceID The device CAN id
   * @param currentLimit Value used for spark max smart current limiting
   */
  ICSparkMax(int deviceID, units::ampere_t currentLimit = 20_A);

  /**
   * Sets position of motor
   *
   * @param position What to set the position to
   */
  void SetPosition(units::turn_t position);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   */
  void SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to using
   * the Spark Max's Smart Motion control mode.
   * This generates a profiled movement that accelerates and decelerates in a controlled way. This
   * can reduce ware on components and is often much easier to tune. In this mode, you are actually
   * controlling the velocity of the motor to follow a trapezoid (speeding up, staying constant,
   * then slowing down) and as such the PID values should be tuned to follow a velocity target.
   * Controlling velocity also allows us to use the WPILib feedforward classes.
   * Also consider feeding CalcMotionProfileTarget() into SetPositionTarget() to use a profile that
   * performs feedback control based on position.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   */
  void SetSmartMotionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Sets the closed loop target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   */
  void SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Gets the current closed loop position target if there is one. Zero otherwise.
   */
  units::turn_t GetPositionTarget() { return _positionTarget; };

  /**
   * Gets the current closed loop velocity target if there is one. Zero otherwise
   */
  units::turns_per_second_t GetVelocityTarget() {
    return GetControlType() == Mode::kSmartMotion ? EstimateSMVelocity() : _velocityTarget;
  };

  /**
   * Get the closed loop position error (current position - target position) if there is one.
   * Zero otherwise.
   */
  units::turn_t GetPosError() { return GetPosition() - _positionTarget; }

  /**
   * Get the closed loop velocity error (current velocity - target velocity) if there is one.
   * Zero otherwise.
   */
  units::turns_per_second_t GetVelError() { return GetVelocity() - _velocityTarget; }

  /**
   * Calculates how much voltage the spark max would be giving to the attached motor given its
   * current control type and PID configuration. Use this in conjunction with one of WPILib's
   * physics simulation classes.
   * (https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)
   */
  units::volt_t GetSimVoltage();

  /**
   * It is the user's responsibility to update the encoder position and
   * velocity when in simulation. To do this, use WPILib's physics simulation
   * classes at
   * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
   * to get the position and velocity of the mechanism attached to this motor.
   */
  void UpdateSimEncoder(units::turn_t position, units::turns_per_second_t velocity);

  /**
   * Gets the current closed loop control type.
   */
  rev::CANSparkMax::ControlType GetControlType() { return _controlType; };

  /**
   * Get the velocity of the motor.
   */
  units::turns_per_second_t GetVelocity();

  /**
   * Get the position of the motor.
   */
  units::turn_t GetPosition() { return units::turn_t{_encoder.GetPosition()}; };

  /**
   * Common interface to stop the motor until Set is called again or closed loop control is started.
   */
  void StopMotor() override;

  /**
   * Common interface for setting the speed of a speed controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  void Set(double speed) override;

  /**
   * Common interface for setting the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output) override;

  /**
   * Configure the constrains for the SparkMax's Smart Motion control mode. Maximum velocity,
   * maximum acceleration, and position tolerance must be set.
   *
   * @param maxVelocity The maxmimum velocity for the motion profile.
   *
   * @param maxAcceleration The maxmimum acceleration for the motion profile.
   *
   * @param tolerance When the position of the motor is within tolerance, the control mode will stop
   * applying power (arbitary feedforward can still apply power).
   */
  void ConfigSmartMotion(units::turns_per_second_t maxVelocity,
                         units::turns_per_second_squared_t maxAcceleration,
                         units::turn_t tolerance);

  /**
   * Set the conversion factor for position, velocity and acceleration of the encoder. The native
   * position units of rotations will be multipled by this number before being used or returned.
   * Velocity and acceleration conversion factors will be derived from this (as position units per
   * second for velocity and velocity units per second for acceleration)
   */
  void SetConversionFactor(double rotationsToDesired);

  /**
   * Set the Proportional, Integral, Derivative and static FeedForward gain constants of the PIDF
   * controller on the SPARK MAX. This uses the Set Parameter API and should be used infrequently.
   * The parameters do not presist unless burnFlash() is called.
   *
   * @param P The proportional gain value, must be positive
   * @param I The Integral gain value, must be positive
   * @param D The Derivative gain value, must be positive
   * @param FF The Feed Forward gain value, must be positive. This is multiplied
   * by the target before being added to the final output power.
   */
  void SetPIDFF(double P, double I, double D, double FF = 0.0);
  void SetP(double P);
  void SetI(double I);
  void SetD(double D);
  void SetFF(double FF);

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently. The parameter does not presist
   * unless burnFlash() is called.
   *
   * @param minOutputPercent Power minimum allowed (-1.0 to 1.0)
   *
   * @param maxOutputPercent Power maximum allowed (-1.0 to 1.0)
   */
  void SetClosedLoopOutputRange(double minOutputPercent, double maxOutputPercent);

  /**
   * Switch to using an external encoder connected to the alternate encoder data port on the SPARK
   * MAX. The pins on this port are defined as:
   *    > Pin 4 (Forward Limit Switch): Index
   *    > Pin 6 (Multi-function): Encoder A
   *    > Pin 8 (Reverse Limit Switch): Encoder B
   *
   * This call will disable support for the limit switch inputs.
   */
  void UseAlternateEncoder();

  /**
   * Switch to using an external absolute encoder connected to the data port on the SPARK MAX.
   *
   * @param zeroOffset the position that is reported as zero. It is influenced by the absolute
   * encoder's position conversion factor, and whether it is inverted. So set those parameters
   * before calling this.
   */
  void UseAbsoluteEncoder(units::turn_t zeroOffset = 0_tr);

  /**
   * Set the minimum and maximum input value for PID Wrapping with position closed loop
   * control.
   *
   * @param max The maximum input value
   * @param min The minimum input value
   */
  void EnableClosedLoopWrapping(units::turn_t min, units::turn_t max);

  /**
   * Check whether the motor is on its position target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnPosTarget(units::turn_t tolerance) { return units::math::abs(GetPosError()) < tolerance; }

  /**
   * Check whether the motor is on its velocity target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnVelTarget(units::turns_per_second_t tolerance) {
    return units::math::abs(GetVelError()) < tolerance;
  }

  // Sendable setup, called automatically when this is passed into smartDashbaord::PutData()
  void InitSendable(wpi::SendableBuilder& builder) override;
  

 private:
  using Mode = rev::CANSparkMax::ControlType;

  // Related REVLib objects
  rev::SparkPIDController _pidController{CANSparkMax::GetPIDController()};
  ICSparkEncoder _encoder{GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)};

  // PID simulation configuration
  bool _updatingTargetFromSendable = false;
  units::turn_t _positionTarget{0};
  units::turns_per_second_t _velocityTarget{0};
  units::volt_t _voltageTarget{0};
  units::volt_t _arbFeedForward = 0.0_V;
  frc::PIDController _simController{0, 0, 0};
  double _simFF{0};
  frc::TrapezoidProfile<units::turns> _motionProfile{
      {units::turns_per_second_t{0},
       units::turns_per_second_squared_t{0}}  // constraints updated by Smart motion config
  };
  frc::Timer _smartMotionProfileTimer;
  Mode _controlType = Mode::kDutyCycle;
  void SetInternalControlType(Mode controlType);
  units::turns_per_second_t EstimateSMVelocity();
  units::turns_per_second_t _simVelocity = units::turns_per_second_t{0};

  // Sim device values (stuff that shows under Other Devices on Glass)
  frc::sim::SimDeviceSim _simDeviceSim{"SPARK MAX ", GetDeviceId()};
  hal::SimInt _simControlMode = _simDeviceSim.GetInt("Control Mode");

  double _minPidOutput = -1;
  double _maxPidOutput = 1;
};
