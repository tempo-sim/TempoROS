// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROS.h"

#if WITH_EDITOR
#include "IHotReload.h"
#endif

#define LOCTEXT_NAMESPACE "FTempoROSModule"

#include "rclcpp/utilities.hpp"
#include "rclcpp/logger.hpp"

DEFINE_LOG_CATEGORY(LogTempoROS);

void SetEnvironmentVars()
{
	// AMENT_PREFIX_PATH
	// Find the rclcpp module directory. The module is not loaded yet, so we can't use FModuleManager.
	const FString ProjectPath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*FPaths::ProjectDir());
	TArray<FString> PossibleTargets;
#if WITH_EDITOR
	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp.Build.cs"), true, false);
	for (FString& PossibleTarget : PossibleTargets)
	{
		PossibleTarget = FPaths::GetPath(PossibleTarget);
	}
#else
	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp"), false, true);
#endif
	checkf(PossibleTargets.Num() == 1, TEXT("Expected to find exactly one rclcpp module"));
	const FString rclcppDir = PossibleTargets[0];

	// Find the Binaries and Libraries directories within rclcpp 
#if PLATFORM_MAC
	const FString PlatformDir(TEXT("Mac"));
#elif PLATFORM_WINDOWS
	const FString PlatformDir(TEXT("Windows"));
#elif PLATFORM_LINUX
	const FString PlatformDir(TEXT("Linux"));
#else
	checkf(false, TEXT("Unsupported platform"));
#endif
	FString LibDir = FPaths::Combine(rclcppDir, "Libraries", PlatformDir);
	FPaths::CollapseRelativeDirectories(LibDir);
	checkf(FPaths::DirectoryExists(*LibDir), TEXT("rclcpp library directory %s did not exist"), *LibDir);
	FPlatformMisc::SetEnvironmentVar(TEXT("AMENT_PREFIX_PATH"), *LibDir);

	// RMW_IMPLEMENTATION
	FPlatformMisc::SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_fastrtps_cpp"));
}

void FTempoROSModule::StartupModule()
{
	SetEnvironmentVars();
	
	rcutils_logging_set_output_handler(rcutils_logging_console_output_handler);
	rclcpp::init(0, nullptr);
	
#if WITH_EDITOR
	IHotReloadModule::Get().OnModuleCompilerStarted().AddLambda([](bool bIsAsyncCompile)
	{
		rclcpp::shutdown();
	});

	IHotReloadModule::Get().OnModuleCompilerFinished().AddLambda([](const FString&, ECompilationResult::Type, bool)
    {
		rclcpp::init(0, nullptr);
    });
#endif
}

void FTempoROSModule::ShutdownModule()
{
	rclcpp::shutdown();
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FTempoROSModule, TempoROS)
