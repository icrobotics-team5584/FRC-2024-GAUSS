// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "utilities/ICSparkMax.h"
#include "utilities/ICSparkEncoder.h"
#include <rev/CANSparkMax.h>
#include <frc2/command/commands.h>
#include "frc/Encoder.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/velocity.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/simulation/DCMotorSim.h>

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();

  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  frc2::CommandPtr CmdSetShooterSpeaker();
  frc2::CommandPtr CmdSetShooterAmp();
  frc2::CommandPtr CmdSetShooterPassing();
  frc2::CommandPtr CmdSetShooterOff();
  bool IsOnTarget();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX _ShooterFlywheelMotorLeft {canid::ShooterFlywheelMotorLeft};
  ctre::phoenix6::hardware::TalonFX _ShooterFlywheelMotorRight {canid::ShooterFlywheelMotorRight};
  ctre::phoenix6::controls::VelocityVoltage _flywheelTargetVelocity{0_tps, 0_tr_per_s_sq, true, 0_V, 0, false};
  
  units::turns_per_second_t ShooterOff = 0_tps;
  units::turns_per_second_t SpeakerSpeed = 60_tps;
  units::turns_per_second_t PassingSpeed = 55_tps;
  units::turns_per_second_t AmpSpeed = 5_tps;

  static constexpr double _flywheelP = 0.1;
  static constexpr double _flywheelI = 0;
  static constexpr double _flywheelD = 0;
  static constexpr double _flywheelV = 0.115;

  //Simulation stuff
  frc::sim::DCMotorSim _leftSim{frc::DCMotor::Falcon500(), 1, 0.005_kg_sq_m};
  frc::sim::DCMotorSim _rightSim{frc::DCMotor::Falcon500(), 1, 0.005_kg_sq_m};
};