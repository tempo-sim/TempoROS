// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROS.h"

#include "TempoROSSettings.h"

#if WITH_EDITOR
#include "IHotReload.h"
#endif

#define LOCTEXT_NAMESPACE "FTempoROSModule"

#include "rclcpp/utilities.hpp"
#include "rclcpp/logger.hpp"

DEFINE_LOG_CATEGORY(LogTempoROS);

void SetAmentPrefixPath()
{
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
}

void FTempoROSModule::StartupModule()
{
	SetAmentPrefixPath();
	
	InitROS();
	
#if WITH_EDITOR
	IHotReloadModule::Get().OnModuleCompilerStarted().AddLambda([this](bool bIsAsyncCompile)
	{
		ShutdownROS();
	});

	IHotReloadModule::Get().OnModuleCompilerFinished().AddLambda([this](const FString&, ECompilationResult::Type, bool)
	{
		InitROS();
	});
#endif

	GetMutableDefault<UTempoROSSettings>()->TempoROSSettingsChangedEvent.AddRaw(this, &FTempoROSModule::InitROS);
}

void FTempoROSModule::ShutdownModule()
{
	ShutdownROS();
}

void FTempoROSModule::InitROS()
{
	if (bROSInitialized)
	{
		ShutdownROS();
	}
	
	// RMW_IMPLEMENTATION
	const UTempoROSSettings* TempoROSSettings = GetDefault<UTempoROSSettings>();
	switch (const ERMWImplementation RMWImplementation = TempoROSSettings->GetRMWImplementation())
	{
	case ERMWImplementation::CycloneDDS:
		{
			FPlatformMisc::SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_cyclonedds_cpp"));
			break;
		}
	case ERMWImplementation::FastRTPS:
		{
			FPlatformMisc::SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_fastrtps_cpp"));
			break;
		}
	}

	// ROS_DOMAIN_ID
	FPlatformMisc::SetEnvironmentVar(TEXT("ROS_DOMAIN_ID"), *FString::FromInt(TempoROSSettings->GetROSDomainID()));

	rclcpp::init(0, nullptr);
	
	bROSInitialized = true;
}

void FTempoROSModule::ShutdownROS()
{
	if (bROSInitialized)
	{
		rclcpp::shutdown();
	
		bROSInitialized = false;
	}
	else
	{
		UE_LOG(LogTempoROS, Warning, TEXT("ShutdownROS called when ROS was not initialized"));
	}
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FTempoROSModule, TempoROS)
