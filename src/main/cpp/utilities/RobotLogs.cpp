#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/RobotLogs.h"
#include <frc/RobotController.h>
#include <frc/PowerDistribution.h>

namespace Logger {
    void logFalcon(ctre::phoenix6::hardware::TalonFX& talonFX, std::string name) {
        std::string logPath = "FalconLogs/" + name;
        frc::SmartDashboard::PutNumber(logPath + "/Voltage", talonFX.GetMotorVoltage().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/StatorCurrent", talonFX.GetStatorCurrent().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/SupplyCurrent", talonFX.GetSupplyCurrent().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/Position", talonFX.GetPosition().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/Temperature", talonFX.GetDeviceTemp().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/TrueVelocity", talonFX.GetVelocity().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/TargetVelocity", talonFX.GetClosedLoopReference().GetValue());
    }

    void logRio(std::string name) {
        std::string logPath = "RioLogs/" + name;
        frc::SmartDashboard::PutBoolean(logPath + "/IsBrownedOut", frc::RobotController::IsBrownedOut());
        frc::SmartDashboard::PutNumber(logPath + "/InputVoltage", frc::RobotController::GetInputVoltage());
        frc::SmartDashboard::PutNumber(logPath + "/InputCurrent", frc::RobotController::GetInputCurrent());
        // frc::SmartDashboard::PutData(logPath + "/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
        // frc::SmartDashboard::PutData(logPath + "/CanStatus", frc::RobotController::GetCANStatus());
    }

    void logPDP(frc::PowerDistribution& pdp, std::string name){
        std::string logPath = "PDPLogs/" + name;
        frc::SmartDashboard::PutNumber(logPath + "/PdpVoltage", pdp.GetVoltage());
        frc::SmartDashboard::PutNumber(logPath + "/PdpTotalCurrent", pdp.GetTotalCurrent());
    }
}
