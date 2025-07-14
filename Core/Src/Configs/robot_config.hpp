#ifndef __ROBOT_CONFIG_HPP
#define __ROBOT_CONFIG_HPP

namespace robot_config {
#ifdef OMNI_INFANTRY
#include "omni_infantry_config.hpp"
#endif

#ifdef MECANUM_INFANTRY
#include "mecanum_infantry_config.hpp"
#endif

#ifdef HERO
#include "hero_config.hpp"
#endif

#ifdef SENTRY
#include "sentry_config.hpp"
#endif

#ifdef GTEST
#include "test_config.hpp"
#else
#include "shared_config.hpp"
#endif

}  // namespace robot_config

#endif