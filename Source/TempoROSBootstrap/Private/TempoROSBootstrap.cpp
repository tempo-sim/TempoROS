#include "TempoROSBootstrap.h"

#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FTempoROSBootstrapModule"

FString GetTempoROSPluginPath()
{
	return IPluginManager::Get().FindPlugin(TEXT("TempoROS"))->GetBaseDir();
}

void FTempoROSBootstrapModule::LoadROSDlls()
{
	const FString TempoROSPluginPath = GetTempoROSPluginPath();
	UE_LOG(LogTemp, Warning, TEXT("TempoROSPluginPath: %s"), *TempoROSPluginPath);
	const FString rclcppPath = FPaths::Combine(TempoROSPluginPath, TEXT("Source"), TEXT("ThirdParty"), TEXT("rclcpp"), TEXT("Binaries"), TEXT("Windows"));
	// TArray<FString> rclcppDLLPaths;
	// IFileManager::Get().FindFilesRecursive(rclcppDLLPaths, *TempoROSPluginPath, TEXT("*.dll"), true, false);
	FPlatformProcess::PushDllDirectory(*rclcppPath);
	// for (const FString& rclcppDLLPath : rclcppDLLPaths)
	// {
	// 	UE_LOG(LogTemp, Warning, TEXT("Loading %s"), *rclcppDLLPath);
	// 	ROSDLLHandles.Add(FPlatformProcess::GetDllHandle(*rclcppDLLPath));
	// }
}

void FTempoROSBootstrapModule::UnloadROSDlls()
{
	for (void* rclcppDllHandle : ROSDLLHandles)
	{
		FPlatformProcess::FreeDllHandle(rclcppDllHandle);
	}
	ROSDLLHandles.Empty();
}

void FTempoROSBootstrapModule::StartupModule()
{
    LoadROSDlls();
}

void FTempoROSBootstrapModule::ShutdownModule()
{
    UnloadROSDlls();
}

#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FTempoROSBootstrapModule, TempoROSBootstrap)
