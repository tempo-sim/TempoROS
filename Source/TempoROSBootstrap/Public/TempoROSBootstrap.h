// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

// ROS uses runtime dynamic linking of dlls. It won't be able to find those dlls at
// runtime unless their containing directory is on the user's PATH variable. In the
// packaged game, adding the TempoROS/Source/ThirdParty/rclcpp/Binaries/Windows directory
// to the user's PATH is the only solution. However in the Editor, where modules are
// loaded as individual dlls, we can use this special "bootsttrap" module to effectively
// add that directory to the PATH for the user *before* TempoROS module is loaded.
class FTempoROSBootstrapModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
