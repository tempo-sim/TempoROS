// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

// Unreal defines a "check" macro, which conflicts with a method in rclcpp
#pragma push_macro("check")
#undef check
#include "rclcpp/rclcpp.hpp"
#pragma pop_macro("check")
