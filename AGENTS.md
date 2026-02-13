# AGENTS.md File

## Context
This is a codebase for multi-nodal embedded system to control different systems on a multiple slightly different robots.
Each robot will have atleast 2 microcontroller development boards (STM32F4 MCUs) each with their own responsibilties.
The boards communicate with each other over a CAN2.0B bus and are attached to various sensors and BLDC motors.

## Library Usage and Conventions
 - Only use a restricted subset of the c++ standard library should be used in development. Unless explicitly told to do so,
   pleas only use `<cstdlib>`, `<string>`, `<cstring>`, `<array>`, `<span>`, `<byte>`, `<concepts>`, and `<type_traits>`.
 - Always prefer static memory allocation over dynamic allocation.
 - Prefer modern C++20 conventions and best practices.
 - Follow the format outlined in .clang-format (based on Google C++ style guidelines).