#pragma once

#include "utilities/ICSparkMax.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/angle.h>
#include <frc/simulation/DCMotorSim.h>

class SwerveSteerIO {
 public:
  SwerveSteerIO();
  virtual void Config() = 0;
  virtual void SetAngle(units::turn_t target) = 0;
  virtual void SetTarget(units::turn_t target) = 0;
  virtual void Stop() = 0;
  virtual void SetBreakMode() = 0;
  virtual void SetCoastMode() = 0;
  virtual std::string GetDeviceID() = 0;
  virtual units::turn_t GetTarget() = 0;
  virtual units::turn_t GetError() = 0;
  virtual units::turn_t GetAngle() = 0;
  virtual units::turns_per_second_t GetVelocity() = 0;
  virtual void Simulate(units::second_t deltaTime) = 0;
  virtual ~SwerveSteerIO() = default;
  frc::sim::DCMotorSim _motorSim;
};

class SwerveSteerTalonFXIO : public SwerveSteerIO {
 public:
  SwerveSteerTalonFXIO(int canID);
  void Config() override;
  void SetAngle(units::turn_t target) override;
  void SetTarget(units::turn_t target) override;
  void Stop() override;
  void SetBreakMode() override;
  void SetCoastMode() override;
  std::string GetDeviceID() override;
  units::turn_t GetTarget() override;
  units::turn_t GetError() override;
  units::turn_t GetAngle() override;
  units::turns_per_second_t GetVelocity() override;
  void Simulate(units::second_t deltaTime) override;

 private:
  ctre::phoenix6::hardware::TalonFX _talon;
};

class SwerveSteerSparkMaxIO : public SwerveSteerIO {
 public:
  SwerveSteerSparkMaxIO(int canID);
  void Config() override;
  void SetAngle(units::turn_t angle) override;
  void SetTarget(units::turn_t target) override;
  void Stop() override;
  void SetBreakMode() override;
  void SetCoastMode() override;
  std::string GetDeviceID() override;
  units::turn_t GetTarget() override;
  units::turn_t GetError() override;
  units::turn_t GetAngle() override;
  units::turns_per_second_t GetVelocity() override;
  void Simulate(units::second_t deltaTime) override;

 private:
  ICSparkMax _spark;
};