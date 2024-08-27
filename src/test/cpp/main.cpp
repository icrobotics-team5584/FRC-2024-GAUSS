#include <iostream>
#include <thread>

#include <gtest/gtest.h>
#include <hal/HALBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/simulation/DriverStationData.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>
#include <units/time.h>
#include "Robot.h"
#include <frc2/command/CommandScheduler.h>

#include "subsystems/SubFeeder.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

TEST(feeder, feederTest) {
  // Setup
  // Robot m_robot;
  // std::optional<std::thread> m_thread;
  // frc::sim::PauseTiming();
  // std::cout << "finished PauseTiming()\n";
  // joystickWarning = frc::DriverStation::IsJoystickConnectionWarningSilenced();
  // frc::DriverStation::SilenceJoystickConnectionWarning(true);
  // m_thread = std::thread([&] { m_robot.StartCompetition(); });
  // frc::sim::StepTiming(0.0_ms);  // Wait for Notifiers
  frc::sim::DriverStationSim::SetAutonomous(false);
  std::cout << "finished SetAutonomous()\n";
  frc::sim::DriverStationSim::SetEnabled(true);
  std::cout << "finished SetEnabled()\n";
  frc::sim::DriverStationSim::NotifyNewData();
  std::cout << "finished NotifyNewData()\n";
  // frc::sim::DriverStationSim::RefreshData();
  // std::cout << "finished RefreshData()\n";
  // HALSIM_NotifyDriverStationNewData();
  // std::cout << "finished HALSIM_NotifyDriverStationNewData()\n";

  // Test
  auto cmd = SubFeeder::GetInstance().FeedToIntake();
  std::cout << "finished FeedToIntake()\n";
  
  cmd.Schedule();
  std::cout << "finished Schedule()\n";
  
  // frc::sim::StepTiming(2_s);
  // std::cout << "finished StepTiming()\n";

  frc2::CommandScheduler::GetInstance().Run();
  std::cout << "finished Run()\n";
  
  EXPECT_NEAR(SubFeeder::GetInstance().GetDutyCycle(), 0.7, 0.05);
  std::cout << "finished EXPECT_NEAR()\n";

  // Tear Down
  // m_robot.EndCompetition();
  // m_thread->join();
  // frc::sim::DriverStationSim::ResetData();
  // frc::DriverStation::SilenceJoystickConnectionWarning(joystickWarning);
}