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

class SubShooter : public frc2::SubsystemBase {
 public:
  SubShooter();

  static SubShooter& GetInstance() {
    static SubShooter inst;
    return inst;
  }

  void UpdatePIDFF();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkMax _shooterPivotMotor{canid::ShooterPivotMotor, 50_A};
  ICSparkMax _shooterFlywheelMotorLeft{canid::ShooterFlywheelMotorLeft, 30_A};
  ICSparkMax _shooterFlywheelMotorRight{canid::_ShooterFlywheelMotorRight, 30_A};

  static constexpr double ShooterP = 0;
  static constexpr double ShooterI = 0;
  static constexpr double ShooterD = 0;

  frc::Encoder _leftEncoder{dio::ShooterFlywheelEncoderLeftChannelA, dio::ShooterFlywheelEncoderLeftChannelB, false, frc::Encoder::EncodingType::k1X};
  frc::Encoder _rightEncoder{dio::ShooterFlywheelEncoderRightChannelA, dio::ShooterFlywheelEncoderRightChannelB, false, frc::Encoder::EncodingType::k1X};
  frc::PIDController _leftShooterPID{ShooterP, ShooterI, ShooterD};
  frc::PIDController _rightShooterPID{ShooterP, ShooterI, ShooterD};

  double _leftEncoderPositionPrev = 0;
  double _rightEncoderPositionPrev = 0;
  double _leftEncoderDif = 0;
  double _rightEncoderDif = 0;
  double _leftPastVelocityAverage = 0;
  double _rightPastVelocityAverage = 0;

  std::array <double, 3> _leftPastVelocityMeasurements{0, 0, 0};
  std::array <double, 3> _rightPastVelocityMeasurements{0, 0, 0};

  frc::SimpleMotorFeedforward <units::turns> _shooterFeedForward {kS, kV, kA};

  static constexpr units::volt_t kS = 0_V;
  static constexpr decltype(1_V / 1_tps) kV = 0 / 1_tps;
  static constexpr decltype(1_V / 1_tr_per_s_sq) kA = 0_V / 1tr_per_s_sq;

  units::turns_per_second_t CurrentShooterTarget = 0_tps;
  };
