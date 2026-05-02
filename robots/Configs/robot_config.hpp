#ifndef __ROBOT_CONFIG_HPP
#define __ROBOT_CONFIG_HPP

#include <type_traits>

namespace robot_config {
    enum class ConfigType { Hero, Infantry, Sentry, AutoAim, Test, None };
    enum class ChassisType { Mecanum, Omni, Swerve };
    enum class GimbalType { DJI };

#ifdef INFANTRY
    constexpr ConfigType config_type = ConfigType::Infantry;
#endif

#ifdef HERO
    constexpr ConfigType config_type = ConfigType::Hero;
#endif

#ifdef SENTRY
    constexpr ConfigType config_type = ConfigType::Sentry;
#endif

#ifdef AUTO_AIM_RIG
    constexpr ConfigType config_type = ConfigType::AutoAim;
#endif

#ifdef GTEST
    constexpr ConfigType config_type = ConfigType::Test;
#endif

#ifdef ENGINEER
    constexpr ConfigType config_type = ConfigType::None;
#endif

#ifdef OMNI_INFANTRY
#include "omni_infantry_config.hpp"
    constexpr ChassisType chassis_type = ChassisType::Omni;
    constexpr GimbalType gimbal_type = GimbalType::DJI;
#endif

#ifdef HERO
#include "hero_config.hpp"
    constexpr ChassisType chassis_type = ChassisType::Mecanum;
    constexpr GimbalType gimbal_type = GimbalType::DJI;
#endif

#ifdef SENTRY
#include "sentry_config.hpp"
    constexpr ChassisType chassis_type = ChassisType::Swerve;
    constexpr GimbalType gimbal_type = GimbalType::DJI;
#endif

#ifdef AUTO_AIM_RIG
#include "auto_aim_rig_config.hpp"
#endif

#ifdef KIDDIE_MODE
#include "kiddie_mode_config.hpp"
    constexpr ChassisType chassis_type = ChassisType::Omni;
    constexpr GimbalType gimbal_type = GimbalType::DJI;
#endif

#ifdef ENGINEER
#include "engineer_config.hpp"
#endif

#ifdef GTEST
#include "test_config.hpp"
#else
#include "shared_config.hpp"
#endif

}  // namespace robot_config

#endif