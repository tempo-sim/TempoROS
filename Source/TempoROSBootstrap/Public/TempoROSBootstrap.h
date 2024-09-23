// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

// Since Windows dynamic linking involves linking symbols via a .lib interface but then loading them at runtime
// via a .dll, we must explicitly add TempoROS's third party Dll directory before loading the TempoROS module.
class FTempoROSBootstrapModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
