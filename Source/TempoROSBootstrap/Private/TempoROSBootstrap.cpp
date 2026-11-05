// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSBootstrap.h"

#include "TempoROSEnvironment.h"

#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FTempoROSBootstrapModule"

FString GetTempoROSDllDirectory()
{
	const FString TempoROSPluginPath = IPluginManager::Get().FindPlugin(TEXT("TempoROS"))->GetBaseDir();
	return FPaths::Combine(TempoROSPluginPath, TEXT("Source"), TEXT("ThirdParty"), TEXT("rclcpp"), TEXT("Binaries"), TEXT("Windows"));
}

// Point AMENT_PREFIX_PATH at the vendored rclcpp install root.
//
// This must happen before anything maps the ROS libraries, which is why it lives in this module
// (LoadingPhase EarliestPossible) rather than in TempoROS. libimage_transport builds a
// pluginlib::ClassLoader in a static initializer, and pluginlib reaches
// ament_index_cpp::get_search_paths(), which throws std::runtime_error when AMENT_PREFIX_PATH is
// unset. Static initializers run as the dynamic loader maps the library, i.e. while
// libUnrealEditor-TempoROS.so is being loaded -- strictly before FTempoROSModule::StartupModule
// could run. The throw escapes module loading and aborts the process during engine startup.
static void SetAmentPrefixPath()
{
	// Find the rclcpp module directory. The module is not loaded yet, so we can't use FModuleManager.
	const FString ProjectPath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*FPaths::ProjectDir());
	TArray<FString> PossibleTargets;
#if WITH_EDITOR
	// With the Editor we simply look for the Build.cs file
	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp.Build.cs"), true, false);
	for (FString& PossibleTarget : PossibleTargets)
	{
		PossibleTarget = FPaths::GetPath(PossibleTarget);
	}
#else
	// In the packaged game we search for a directory called "rclcpp" within a directory called "ThirdParty"
	IFileManager::Get().FindFilesRecursive(PossibleTargets, *ProjectPath, TEXT("rclcpp"), false, true);
	for (auto PossibleTargetIt = PossibleTargets.CreateIterator(); PossibleTargetIt; ++PossibleTargetIt)
	{
		if (!FPaths::GetPath(*PossibleTargetIt).EndsWith(TEXT("ThirdParty")))
		{
			PossibleTargetIt.RemoveCurrent();
		}
	}
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
#if PLATFORM_WINDOWS
	FString LibDir = FPaths::Combine(rclcppDir, "Binaries", PlatformDir);
#else
	FString LibDir = FPaths::Combine(rclcppDir, "Libraries", PlatformDir);
#endif
	FPaths::CollapseRelativeDirectories(LibDir);
	checkf(FPaths::DirectoryExists(*LibDir), TEXT("rclcpp library directory %s did not exist"), *LibDir);

	SetEnvironmentVar(TEXT("AMENT_PREFIX_PATH"), *LibDir);
}

void FTempoROSBootstrapModule::StartupModule()
{
	// Before anything maps the ROS libraries. See SetAmentPrefixPath.
	SetAmentPrefixPath();

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
