#include <gtest/gtest.h>
#include <hal/HALBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/SubFeeder.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

TEST(feeder, feederTest) {
  // Setup
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();

  // Test
  auto cmd = SubFeeder::GetInstance().FeedToIntake();
  cmd.Schedule();
  frc2::CommandScheduler::GetInstance().Run();
  EXPECT_NEAR(SubFeeder::GetInstance().GetDutyCycle(), 0.7, 0.05);
}