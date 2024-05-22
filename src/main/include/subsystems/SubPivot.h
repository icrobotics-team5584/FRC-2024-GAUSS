// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/controller/ArmFeedforward.h>

class SubPivot : public frc2::SubsystemBase {
public:
  SubPivot();
  static SubPivot &GetInstance() {
    static SubPivot inst;
    return inst;
  }
  frc2::CommandPtr CmdSetPivotAngle(units::degree_t targetAngle);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ICSparkMax _pivotMotor{canid::ShooterPivotMotor, 20_A};

  static constexpr double _pivotP = 100;
  static constexpr double _pivotI = 0;
  static constexpr double _pivotD = 0;

  static constexpr auto PIVOT_G = 0.47_V;
  static constexpr auto PIVOT_S = 0_V;
  static constexpr auto PIVOT_V = 0_V/1_tps;
  static constexpr auto PIVOT_A = 0_V/1_tr_per_s_sq;
  
  static constexpr double PIVOT_GEAR_RATIO = 175;
  static constexpr units::meter_t SHOOTER_LENGTH = 0.523_m;
  static constexpr units::kilogram_t SHOOTER_MASS = 6.9_kg;
  static constexpr units::degree_t SHOOTER_MIN_ANGLE = 10_deg;
  static constexpr units::degree_t SHOOTER_MAX_ANGLE = 100_deg;

  frc::ArmFeedforward _pivotFF{PIVOT_S, PIVOT_G, PIVOT_V, PIVOT_A};

  //simulating pivot in smartdashboard
  frc::sim::SingleJointedArmSim _pivotSim{
    frc::DCMotor::NEO(1),
    PIVOT_GEAR_RATIO,
    frc::sim::SingleJointedArmSim::EstimateMOI(SHOOTER_LENGTH, SHOOTER_MASS),
    SHOOTER_LENGTH,
    SHOOTER_MIN_ANGLE,
    SHOOTER_MAX_ANGLE,
    true,
    SHOOTER_MIN_ANGLE
  };
};

// 6.9kg
// 523mm