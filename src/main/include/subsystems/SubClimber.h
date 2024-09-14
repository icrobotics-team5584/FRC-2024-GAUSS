// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>

#include <numbers>

#include "Utilities/ICSparkMax.h"

#include <frc2/command/button/CommandXboxController.h>

#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>

#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include <grpl/LaserCan.h>

#include <units/angle.h>

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include <frc2/command/commands.h>

class SubClimber : public frc2::SubsystemBase {
 public:
  SubClimber();

  static SubClimber& GetInstance() {
    static SubClimber inst;
    return inst;
  }

  // Periodic
  void Periodic() override;
  void SimulationPeriodic() override;

   // Robot info
  static constexpr units::meter_t BASE_HEIGHT = 0.0_m;
  static constexpr units::meter_t TOP_HEIGHT = 0.5_m;
  static constexpr units::length::meter_t  STOW_HEIGHT = 0.25_m;

  //Units translation
  units::turn_t DistanceToTurn(units::meter_t distance);
  units::radians_per_second_t DistanceToTurn(units::meters_per_second_t distance);
  units::meter_t TurnToDistance(units::turn_t turn);

  // Primary actions
  void Start(double power);
  void Stop();

  void ZeroClimber();

  double GetLeftCurrent();
  double GetRightCurrent();
  units::meter_t GetLeftHeight();
  units::meter_t GetRightHeight();

  void DriveToDistance(units::meter_t distance);

  void EnableSoftLimit(bool enabled);

  //Command actions
  frc2::CommandPtr ClimberJoystickDrive(frc2::CommandXboxController& _controller);
  frc2::CommandPtr ClimberJoystickDriveLeft(frc2::CommandXboxController& _controller);
  frc2::CommandPtr ClimberJoystickDriveRight(frc2::CommandXboxController& _controller);

  frc2::CommandPtr ClimberPosition(units::meter_t distance);
  frc2::CommandPtr ClimberManualDrive(double power);
  frc2::CommandPtr ClimberStop();
  frc2::CommandPtr ClimberResetZero();
  frc2::CommandPtr ClimberResetTop();
  frc2::CommandPtr ClimberAutoReset();
  frc2::CommandPtr ClimberResetCheck();
  units::meter_t CheckLeftClimberPos();
  units::meter_t CheckRightClimberPos();
 private:
  units::meter_t TargetDistance;

  // Motor
  ICSparkMax _lClimbMotor{canid::ClimberMain, 60_A};
  ICSparkMax _rClimbMotor{canid::ClimberSecond, 60_A};

  // Motor Setup
  static constexpr double gearRatio = 45;
  static constexpr double lP = 0.001, lI = 0.0, lD = 0.0, lF = 0,
  
                          rP = 0.001, rI = 0.0, rD = 0.0, rF = 0;

  static constexpr double currentLimit = 10;

  // Unit translation
  static constexpr units::meter_t WheelCir = 0.157_m;

  static constexpr double recalcV = 35.1;

  //reset
  bool Reseting = false;
  bool Reseted = false;

  bool ResetLeft = false; bool ResetRight = false;

  // Sim

  frc::sim::ElevatorSim lElvSim{frc::DCMotor::NEO(), gearRatio, 26_kg, (WheelCir/std::numbers::pi)/2, BASE_HEIGHT, 5_m, false, BASE_HEIGHT};
  frc::sim::ElevatorSim rElvSim{frc::DCMotor::NEO(), gearRatio, 26_kg, (WheelCir/std::numbers::pi)/2, BASE_HEIGHT, 5_m, false, BASE_HEIGHT};

  frc::Mechanism2d mech{4, 4};
  frc::MechanismRoot2d* mechRootL = mech.GetRoot("ClimberL", 1, 1);
  frc::MechanismRoot2d* mechRootR = mech.GetRoot("ClimberR", 3, 1);
  frc::MechanismRoot2d* mechRootT = mech.GetRoot("ClimberT", 2, 1);
  frc::MechanismLigament2d* mechLeftElevator =
      mechRootL->Append<frc::MechanismLigament2d>("Left elevator", 1, 90_deg);
  frc::MechanismLigament2d* mechRightElevator =
      mechRootR->Append<frc::MechanismLigament2d>("Right elevator", 3, 90_deg);
  frc::MechanismLigament2d* mechTar =
      mechRootT->Append<frc::MechanismLigament2d>("Target", 2, 90_deg);
};
