// Copyright Tempo Simulation, LLC. All Rights Reserved.

#include "TempoROSEnv.h"

#include "Misc/FileHelper.h"

#define LOCTEXT_NAMESPACE "FTempoROSEnvModule"
//
// void SetAmentPrefixPath()
// {
// 	// Find the rclcpp module directory. The module is not loaded yet, so we can't use FModuleManager.
// 	const FString ProjectPath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*FPaths::ProjectDir());
// 	TArray<FString> PossibleTargets;
// #if WITH_EDITOR
// 	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp.Build.cs"), true, false);
// 	for (FString& PossibleTarget : PossibleTargets)
// 	{
// 		PossibleTarget = FPaths::GetPath(PossibleTarget);
// 	}
// #else
// 	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp"), false, true);
// #endif
// 	checkf(PossibleTargets.Num() == 1, TEXT("Expected to find exactly one rclcpp module"));
// 	const FString rclcppDir = PossibleTargets[0];
//
// 	// Find the Binaries and Libraries directories within rclcpp 
// #if PLATFORM_MAC
// 	const FString PlatformDir(TEXT("Mac"));
// #elif PLATFORM_WINDOWS
// 	const FString PlatformDir(TEXT("Windows"));
// #elif PLATFORM_LINUX
// 	const FString PlatformDir(TEXT("Linux"));
// #else
// 	checkf(false, TEXT("Unsupported platform"));
// #endif
// 	FString LibDir = FPaths::Combine(rclcppDir, "Libraries", PlatformDir);
// 	FPaths::CollapseRelativeDirectories(LibDir);
// 	checkf(FPaths::DirectoryExists(*LibDir), TEXT("rclcpp library directory %s did not exist"), *LibDir);
// 	FPlatformMisc::SetEnvironmentVar(TEXT("AMENT_PREFIX_PATH"), *LibDir);
// 	FFileHelper::SaveStringToFile(FString::Printf(TEXT("Logging to %s. NO_LOGGING: %d"), *FPaths::ProjectLogDir(), NO_LOGGING), TEXT("/Users/pete/Desktop/loginfo.txt"));
// 	UE_LOG(LogTemp, Warning, TEXT("Logging to %s. NO_LOGGING: %d"), *FPaths::ProjectLogDir(), NO_LOGGING);
// }
//
// void SetRMWImplementation()
// {
// 	FPlatformMisc::SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_cyclonedds_cpp"));
// }

void FTempoROSEnvModule::StartupModule()
{
	// SetAmentPrefixPath();
	// SetRMWImplementation();
}

void FTempoROSEnvModule::ShutdownModule()
{
    
}

#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FTempoROSEnvModule, TempoROSEnv)
