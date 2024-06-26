#pragma once

#include <filesystem>
#include <fstream>
#include <frc/smartdashboard/smartdashboard.h>
#include <string>

namespace BotVars {

enum Robot { COMP, PRACTICE };
const inline std::string COMP_BOT_MAC_ADDRESS = "00:80:2f:33:d2:cb";
const inline std::string PRACTICE_BOT_MAC_ADDRESS = "00:80:2f:34:07:fe";

Robot DetermineRobot();

const inline Robot activeRobot = DetermineRobot();

template <typename T>
T Choose(T compBotValue, T practiceBotValue) {
  return activeRobot == COMP ? compBotValue : practiceBotValue;
}

}  // namespace BotVars