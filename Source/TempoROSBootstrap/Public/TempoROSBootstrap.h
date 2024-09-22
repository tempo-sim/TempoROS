#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FTempoROSBootstrapModule : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
#if PLATFORM_WINDOWS
    void LoadROSDlls();

    void UnloadROSDlls();
	
    TArray<void*> ROSDLLHandles;
#endif
};
