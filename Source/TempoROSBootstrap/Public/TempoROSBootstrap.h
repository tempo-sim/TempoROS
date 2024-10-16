// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

// ROS uses runtime dynamic linking of .dlls. On Windows, since those .dlls live in the Binaries directory, as opposed
// to the dylibs on Mac and .sos on Linux, which live in the Libraries directory, it won't be able to find them at
// runtime unless the Binaries directory is on the user's PATH variable. In the packaged game, adding the
// TempoROS/Source/ThirdParty/rclcpp/Binaries/Windows directory to the user's PATH is the only solution. However in the
// Editor, where modules are themselves loaded as individual dlls, we can use this special "bootstrap" module to
// effectively add that directory to the PATH as long as it is initialized *before* the TempoROS module.
class FTempoROSBootstrapModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
