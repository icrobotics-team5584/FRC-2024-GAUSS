#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/mass.h>
#include <units/temperature.h>
#include <frc/geometry/Rotation2d.h>

namespace Logger {
void logFalcon(ctre::phoenix6::hardware::TalonFX& talonFX, std::string name);

void Log(std::string_view keyName, wpi::Sendable* data);
void Log(std::string_view keyName, double value);
void Log(std::string_view keyName, ctre::phoenix6::StatusSignal<double>& signal);
void Log(std::string_view keyName, bool value);
void Log(std::string_view keyName, std::string_view value);
void Log(std::string keyName, units::turn_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turn_t>& signal);
void Log(std::string keyName, units::degree_t value);
void Log(std::string keyName, units::turns_per_second_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_t>& signal);
void Log(std::string keyName, units::turns_per_second_squared_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>& signal);
void Log(std::string keyName, units::meter_t value);
void Log(std::string keyName, units::meters_per_second_t value);
void Log(std::string keyName, units::meters_per_second_squared_t value);
void Log(std::string keyName, units::volt_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::volt_t>& signal);
void Log(std::string keyName, units::ampere_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::ampere_t>& signal);
void Log(std::string keyName, units::second_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::second_t>& signal);
void Log(std::string keyName, units::kilogram_t value);
void Log(std::string keyName, units::celsius_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::celsius_t>& signal);
void Log(std::string keyName, frc::Rotation2d value);

};  // namespace Logger