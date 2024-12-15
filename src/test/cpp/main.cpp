#include <gtest/gtest.h>
#include <hal/HALBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/CommandPtr.h>
#include <units/time.h>

#include "subsystems/SubFeeder.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubClimber.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

void SimCmdScheduler(units::second_t duration = 20_ms) {
  int loops = (duration.value()*1000) / 20;
  for (int i=0; i<=loops; i++){
    frc2::CommandScheduler::GetInstance().Run();
  }
}

// FEEDER TESTS

TEST(feeder, feedToIntake) {
  auto cmd = SubFeeder::GetInstance().FeedToIntake();
  cmd.Schedule();
  SimCmdScheduler();
  EXPECT_GT(SubFeeder::GetInstance().GetDutyCycle(), 0.1);
}

TEST(feeder, feedToShooter) {
  auto cmd = SubFeeder::GetInstance().FeedToShooter();
  cmd.Schedule();
  SimCmdScheduler();
  EXPECT_LT(SubFeeder::GetInstance().GetDutyCycle(), -0.1);
}

TEST(feeder, cancelFeedToIntake) {
  auto cmd = SubFeeder::GetInstance().FeedToIntake();
  cmd.Schedule();
  SimCmdScheduler();
  cmd.Cancel();
  SimCmdScheduler();
  EXPECT_EQ(SubFeeder::GetInstance().GetDutyCycle(), 0.0);
}

// PIVOT TESTS

TEST(pivot, setAngle) {
  auto cmd = SubPivot::GetInstance().CmdSetPivotAngle(0.2_tr);
  cmd.Schedule();
  SimCmdScheduler(3_s);
  EXPECT_NEAR(SubPivot::GetInstance().GetAngle().value(), 0.2, 0.01);
}

TEST(pivot, visionAlign) {
  auto fakeVisionProcess = []{return 0_deg;};
  units::turn_t expectedAngle = 27_deg;
  auto cmd = SubPivot::GetInstance().CmdPivotFromVision(fakeVisionProcess);
  cmd.Schedule();
  SimCmdScheduler(2_s);
  EXPECT_NEAR(SubPivot::GetInstance().GetAngle().value(), expectedAngle.value(), 0.01);
}

TEST(pivot, lowSoftLimit) {
  units::turn_t target = 10_deg;
  auto cmd = SubPivot::GetInstance().CmdSetPivotAngle(target);
  cmd.Schedule();
  SimCmdScheduler(2_s);
  // Hits lower limit of 14 deg and goes to zero volts. 
  // Might slowly drop due to gravity so just check volts not position.
  EXPECT_GE(SubPivot::GetInstance().GetVoltage().value(), 0);
}

TEST(pivot, highSoftLimit) {
  units::turn_t target = 180_deg;
  units::turn_t highLimit = 90_deg;
  auto cmd = SubPivot::GetInstance().CmdSetPivotAngle(target);
  cmd.Schedule();
  SimCmdScheduler(2_s);
  // Hits upper limit of 90 deg and goes to zero volts. 
  // Mechanism is capable of going further so it might swing past, so just check volts not position.
  EXPECT_LE(SubPivot::GetInstance().GetVoltage().value(), 0);
}

// CLIMBER TESTS

TEST(climber, goUp) {
  units::meter_t target = 0.4_m;
  auto cmd = SubClimber::GetInstance().ClimberPosition(target);
  cmd.Schedule();
  SimCmdScheduler(3_s);
  EXPECT_NEAR(SubClimber::GetInstance().GetRightHeight().value(), target.value(), 0.01);
  EXPECT_NEAR(SubClimber::GetInstance().GetLeftHeight().value(), target.value(), 0.01);
}