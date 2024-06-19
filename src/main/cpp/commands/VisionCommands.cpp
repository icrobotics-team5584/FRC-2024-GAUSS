#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include "commands/VisionCommands.h"
#include <frc/DriverStation.h>
#include "RobotContainer.h"
#include <frc/DriverStation.h>

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr AddVisionMeasurement() {
  return Run(
      [] {
        auto estimatedPose = SubVision::GetInstance().GetPose();
        frc::SmartDashboard::PutBoolean("Vision/is teleop", frc::DriverStation::IsTeleop());
        frc::SmartDashboard::PutBoolean("Vision/has value", estimatedPose.has_value());
        if (estimatedPose.has_value() && frc::DriverStation::IsTeleop()) {
          auto estimatedPoseValue = estimatedPose.value();
          SubDrivebase::GetInstance().AddVisionMeasurement(
              estimatedPoseValue.estimatedPose.ToPose2d(), 0,
              estimatedPoseValue.timestamp);
          SubDrivebase::GetInstance().DisplayPose(
              "EstimatedPose", estimatedPoseValue.estimatedPose.ToPose2d());
        } else {
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose", {});
        }
      },
      {&SubVision::GetInstance()});
}
}

