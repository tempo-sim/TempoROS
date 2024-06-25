// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

TEMPOROS_API DECLARE_LOG_CATEGORY_EXTERN(LogTempoROS, Log, All);

class TEMPOROS_API FTempoROSModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
