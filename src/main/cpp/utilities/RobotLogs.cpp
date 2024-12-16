#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/RobotLogs.h"

namespace Logger {
void logFalcon(ctre::phoenix6::hardware::TalonFX& talonFX, std::string name) {
  std::string logPath = "FalconLogs/" + name;
  Log(logPath + "/Voltage", talonFX.GetMotorVoltage());
  Log(logPath + "/StatorCurrent", talonFX.GetStatorCurrent());
  Log(logPath + "/SupplyCurrent", talonFX.GetSupplyCurrent());
  Log(logPath + "/Position", talonFX.GetPosition());
  Log(logPath + "/Temperature", talonFX.GetDeviceTemp());
  Log(logPath + "/TrueVelocity", talonFX.GetVelocity());
  Log(logPath + "/Target", talonFX.GetClosedLoopReference());
}

void Log(std::string_view keyName, wpi::Sendable* data) {
  frc::SmartDashboard::PutData(keyName, data);
}

void Log(std::string_view keyName, double value) {
  frc::SmartDashboard::PutNumber(keyName, value);
}

void Log(std::string_view keyName, ctre::phoenix6::StatusSignal<double>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string_view keyName, bool value) {
  frc::SmartDashboard::PutBoolean(keyName, value);
}

void Log(std::string_view keyName, std::string_view value) {
  frc::SmartDashboard::PutString(keyName, value);
}

void Log(std::string keyName, units::turn_t value) {
  Log(keyName + " (tr)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turn_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::degree_t value) {
  Log(keyName + " (deg)", value.value());
}

void Log(std::string keyName, units::turns_per_second_t value) {
  Log(keyName + " (tps)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::turns_per_second_squared_t value) {
  Log(keyName + " (tps_sq)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::meter_t value) {
  Log(keyName + " (m)", value.value());
}

void Log(std::string keyName, units::meters_per_second_t value) {
  Log(keyName + " (mps)", value.value());
}

void Log(std::string keyName, units::meters_per_second_squared_t value) {
  Log(keyName + " (mps_sq)", value.value());
}

void Log(std::string keyName, units::volt_t value) {
  Log(keyName + " (V)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::volt_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::ampere_t value) {
  Log(keyName + " (A)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::ampere_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::second_t value) {
  Log(keyName + " (s)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::second_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::kilogram_t value) {
  Log(keyName + " (kg)", value.value());
}

void Log(std::string keyName, units::celsius_t value) {
  Log(keyName + " (C)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::celsius_t>& signal) {
  Log(keyName, signal.GetValue());
}


void Log(std::string keyName, frc::Rotation2d value) {
  Log(keyName, value.Degrees());
}


}  // namespace Logger
