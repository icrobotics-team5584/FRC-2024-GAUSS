#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <units/time.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include "utilities/RobotLogs.h"

SubDrivebase::SubDrivebase() {
  frc::SmartDashboard::PutData("Drivebase/Vision/Rotation Controller: ", &_teleopRotationController);

  _teleopRotationController.EnableContinuousInput(0_deg, 360_deg);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);

  using namespace pathplanner;
  AutoBuilder::configure(
      // Robot pose supplier
      [this]() { return GetPose(); },

      // Method to reset odometry (will be called if your auto has a starting pose)
      [this](frc::Pose2d pose) { SetPose(pose); },

      // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this]() { return GetRobotRelativeSpeeds(); },

      // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
      // outputs individual module feedforwards
      [this](auto speeds, auto feedforwards) { Drive(speeds.vx, speeds.vy, speeds.omega, false); },

      // PID Feedback controller for translation and rotation
      _pathplannerController,

      // robot mass, MOT, wheel locations, etc
      RobotConfig::fromGUISettings(),

      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      []() {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          frc::SmartDashboard::PutString(
              "Drivebase/Alliance",
              alliance.value() == frc::DriverStation::Alliance::kBlue ? "Blue" : "Red");
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        frc::SmartDashboard::PutString("Drivebase/Alliance",
                                       "Failed to detect alliance, assuming blue");
        return false;
      },

      // Reference to this subsystem to set requirements
      this);
}

void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();
  // Dashboard Displays:
  frc::SmartDashboard::PutNumber("Drivebase/heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("Drivebase/velocity", GetVelocity().value());

  frc::SmartDashboard::PutNumberArray("Drivebase/true swerve states",
                                      std::array{
                                          _frontLeft.GetAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });

  frc::SmartDashboard::PutNumberArray("Drivebase/can coders swerve states",
                                      std::array{
                                          _frontLeft.GetCanCoderAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetCanCoderAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetCanCoderAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetCanCoderAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  UpdateOdometry();
  frc::SmartDashboard::PutNumber("Drivebase/loop time (sec)", (frc::GetTime() - loopStart).value());
}

void SubDrivebase::SimulationPeriodic() {
  _frontLeft.UpdateSim(20_ms);
  _frontRight.UpdateSim(20_ms);
  _backLeft.UpdateSim(20_ms);
  _backRight.UpdateSim(20_ms);
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  double deadband = 0.08;
  auto velocity = Logger::Tune("Drivebase/Config/MaxVelocity", MAX_VELOCITY);
  auto angularVelocity = Logger::Tune("Drivebase/Config/MaxAngularVelocity", MAX_ANGULAR_VELOCITY);
  static frc::SlewRateLimiter<units::scalar> _xspeedLimiter{MAX_JOYSTICK_ACCEL / 1_s};
  static frc::SlewRateLimiter<units::scalar> _yspeedLimiter{MAX_JOYSTICK_ACCEL / 1_s};
  static frc::SlewRateLimiter<units::scalar> _rotLimiter{MAX_ANGULAR_JOYSTICK_ACCEL / 1_s};
  auto forwardSpeed =
      _yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftY(), deadband)) * velocity;
  auto rotationSpeed =
      _rotLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(), deadband)) * angularVelocity;
  auto sidewaysSpeed =
      _xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(), deadband)) * velocity;

  return frc::ChassisSpeeds{forwardSpeed, sidewaysSpeed, rotationSpeed};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller) {
  return Drive([this, &controller] { return CalcJoystickSpeeds(controller); });
}

frc2::CommandPtr SubDrivebase::Drive(std::function<frc::ChassisSpeeds()> speeds) {
  return Run([this, speeds] {
           auto speedVals = speeds();
           Drive(speedVals.vx, speedVals.vy, speedVals.omega, false);
         })
      .FinallyDo([this] { Drive(0_mps, 0_mps, 0_deg_per_s, false); });
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                         units::turns_per_second_t rot, bool fieldRelative) {
  // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                    ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  constexpr bool inSim = frc::RobotBase::IsSimulation();
  speeds = frc::ChassisSpeeds::Discretize(speeds, inSim ? 200_ms : -200_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(
      &states,
      frc::SmartDashboard::GetNumber("Drivebase/Config/MaxVelocity", MAX_VELOCITY.value()) * 1_mps);

  // Setting modules from aquired states
  auto [fl, fr, bl, br] = states;

  frc::SmartDashboard::PutNumberArray("Drivebase/desired swerve states",
                                      std::array{
                                          fl.angle.Degrees().value(),
                                          fl.speed.value(),
                                          fr.angle.Degrees().value(),
                                          fr.speed.value(),
                                          bl.angle.Degrees().value(),
                                          bl.speed.value(),
                                          br.angle.Degrees().value(),
                                          br.speed.value(),
                                      });

  _frontLeft.SetDesiredState(fl);
  _frontRight.SetDesiredState(fr);
  _backLeft.SetDesiredState(bl);
  _backRight.SetDesiredState(br);

  // Check if robot is in simulation.
  // Manualy adjusting gyro by calculating rotation in simulator as gyro is not enabled in
  // simulation
  if (frc::RobotBase::IsSimulation()) {
    units::radian_t radPer20ms = rot * 20_ms;
    units::degree_t newHeading = GetHeading().RotateBy(radPer20ms).Degrees();
    _gyro.SetAngleAdjustment(-newHeading.value());  // negative to switch to CW from CCW
  }
}

frc::ChassisSpeeds SubDrivebase::GetRobotRelativeSpeeds() {
  auto fl = _frontLeft.GetState();
  auto fr = _frontRight.GetState();
  auto bl = _backLeft.GetState();
  auto br = _backRight.GetState();
  return _kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();

  // config turn motors so it can run in auto init also. Had issues with parameters not being set on startup
  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

frc2::CommandPtr SubDrivebase::SyncSensorBut() {
  return RunOnce([this] { SyncSensors(); });
}

frc::Rotation2d SubDrivebase::GetHeading() {
  return _gyro.GetRotation2d();
}

units::meters_per_second_t SubDrivebase::GetVelocity() {
  // Use pythag to find velocity from x and y components
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                            _backLeft.GetState(), _backRight.GetState());
  namespace m = units::math;
  return m::sqrt(m::pow<2>(speeds.vx) + m::pow<2>(speeds.vy));
}

frc::SwerveDriveKinematics<4> SubDrivebase::GetKinematics() {
  return _kinematics;
}

// calculates the relative field location
void SubDrivebase::UpdateOdometry() {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _poseEstimator.Update(GetHeading(), {fl, fr, bl, br});
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

frc::ChassisSpeeds SubDrivebase::CalcDriveToPoseSpeeds(frc::Pose2d targetPose) {
  // Find current and target values
  DisplayPose("targetPose", targetPose);
  double targetXMeters = targetPose.X().value();
  double targetYMeters = targetPose.Y().value();
  units::turn_t targetRotation = targetPose.Rotation().Radians();
  frc::Pose2d currentPosition = GetPose();
  double currentXMeters = currentPosition.X().value();
  double currentYMeters = currentPosition.Y().value();
  units::turn_t currentRotation = currentPosition.Rotation().Radians();

  // Use PID controllers to calculate speeds
  auto xSpeed = _teleopTranslationcontroller.Calculate(currentXMeters, targetXMeters) * 1_mps;
  auto ySpeed = _teleopTranslationcontroller.Calculate(currentYMeters, targetYMeters) * 1_mps;
  auto rSpeed = CalcRotateSpeed(targetRotation - currentRotation);

  // Clamp to max velocity
  xSpeed = units::math::min(xSpeed, MAX_VELOCITY);
  xSpeed = units::math::max(xSpeed, -MAX_VELOCITY);
  ySpeed = units::math::min(ySpeed, MAX_VELOCITY);
  ySpeed = units::math::max(ySpeed, -MAX_VELOCITY);

  return frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed};
}

units::turns_per_second_t SubDrivebase::CalcRotateSpeed(units::turn_t rotationError) { 
  auto omega = _teleopRotationController.Calculate(rotationError, 0_deg) * 1_rad_per_s;
  omega = units::math::min(omega, MAX_ANGULAR_VELOCITY);
  omega = units::math::max(omega, -MAX_ANGULAR_VELOCITY);
  return omega;
}

bool SubDrivebase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = _poseEstimator.GetEstimatedPosition();
  auto rotError = currentPose.Rotation() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());

  if (units::math::abs(rotError.Degrees()) < 1_deg && posError < 1_cm) {
    return true;
  } else {
    return false;
  }
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.Reset();
  _gyro.SetAngleAdjustment(startingAngle.value());
}

frc2::CommandPtr SubDrivebase::ResetGyroCmd() {
  return RunOnce([this] { ResetGyroHeading(); });
}

frc::Pose2d SubDrivebase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void SubDrivebase::SetPose(frc::Pose2d pose) {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();
  _poseEstimator.ResetPosition(GetHeading(), {fl, fr, bl, br}, pose);
  ResetGyroHeading(pose.Rotation().Degrees());
}

void SubDrivebase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}

void SubDrivebase::UpdatePosition(frc::Pose2d robotPosition) {
  _poseEstimator.AddVisionMeasurement(robotPosition, 2_ms);
}

void SubDrivebase::DisplayTrajectory(std::string name, frc::Trajectory trajectory) {
  _fieldDisplay.GetObject(name)->SetTrajectory(trajectory);
}

void SubDrivebase::AddVisionMeasurement(frc::Pose2d pose, double ambiguity,
                                        units::second_t timeStamp) {
  frc::SmartDashboard::PutNumber("Timestamp", timeStamp.value());
  _poseEstimator.AddVisionMeasurement(pose, timeStamp);
}

void SubDrivebase::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  _frontLeft.SetNeutralMode(mode);
  _frontRight.SetNeutralMode(mode);
  _backLeft.SetNeutralMode(mode);
  _backRight.SetNeutralMode(mode);
}

units::degree_t SubDrivebase::GetPitch() {
  return _gyro.GetPitch() * 1_deg;
}

frc2::CommandPtr SubDrivebase::WheelCharecterisationCmd() {
  static units::radian_t initialGyroHeading = 0_rad;
  static units::radian_t initialWheelDistance = 0_rad;

  return RunOnce([this] {
           initialGyroHeading = GetHeading().Radians();
           // initialWheelDistance =
           //     (_frontRight.GetPosition().distance + _frontLeft.GetPosition().distance +
           //      _backRight.GetPosition().distance + _backLeft.GetPosition().distance) /
           //     4;
           initialWheelDistance = _frontRight.GetDrivenRotations();
         })
      .AndThen(Drive([] { return frc::ChassisSpeeds{0_mps, 0_mps, 15_deg_per_s}; }))
      .FinallyDo([this] {
        units::meter_t drivebaseRadius = _frontLeftLocation.Norm();
        units::radian_t finalGyroHeading = GetHeading().Radians();
        // auto finalWheelDistance =
        //     (_frontRight.GetPosition().distance + _frontLeft.GetPosition().distance +
        //      _backRight.GetPosition().distance + _backLeft.GetPosition().distance) /
        //     4;
        units::radian_t finalWheelDistance = _frontRight.GetDrivenRotations();

        units::radian_t gyroDelta = finalGyroHeading - initialGyroHeading;
        units::radian_t wheelDistanceDelta = finalWheelDistance - initialWheelDistance;

        frc::SmartDashboard::PutNumber(
            "Drivebase/WheelCharacterisation/CalcedWheelRadius",
            ((gyroDelta * drivebaseRadius) / wheelDistanceDelta).value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/Gyro", gyroDelta.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/DrivebaseRadius",
                                       drivebaseRadius.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/WheelDistance",
                                       wheelDistanceDelta.value());
      });
}
