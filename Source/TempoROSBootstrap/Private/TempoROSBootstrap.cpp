// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSBootstrap.h"

#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FTempoROSBootstrapModule"

FString GetTempoROSDllDirectory()
{
	const FString TempoROSPluginPath = IPluginManager::Get().FindPlugin(TEXT("TempoROS"))->GetBaseDir();
	return FPaths::Combine(TempoROSPluginPath, TEXT("Source"), TEXT("ThirdParty"), TEXT("rclcpp"), TEXT("Binaries"), TEXT("Windows"));
}

void FTempoROSBootstrapModule::StartupModule()
{
#if PLATFORM_WINDOWS
	FPlatformProcess::PushDllDirectory(*GetTempoROSDllDirectory());
#endif
}

void FTempoROSBootstrapModule::ShutdownModule()
{
#if PLATFORM_WINDOWS
	FPlatformProcess::PopDllDirectory(*GetTempoROSDllDirectory());
#endif
}

#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FTempoROSBootstrapModule, TempoROSBootstrap)
