// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROS.h"

#include "TempoROSAllocator.h"
#include "TempoROSEnvironment.h"
#include "TempoROSSettings.h"

#if WITH_EDITOR
#include "IHotReload.h"
#endif

#define LOCTEXT_NAMESPACE "FTempoROSModule"

#include "rclcpp/utilities.hpp"

DEFINE_LOG_CATEGORY(LogTempoROS);

void FTempoROSModule::StartupModule()
{
	// Route rclcpp's default std::pmr allocations through Unreal's allocator before anything in rclcpp runs.
	SetUnrealDefaultMemoryResource();

	// AMENT_PREFIX_PATH is set by TempoROSBootstrap (LoadingPhase EarliestPossible). It cannot be
	// set here: the ROS libraries this module links are already mapped by the time StartupModule
	// runs, and libimage_transport's static initializers read it. See SetAmentPrefixPath.
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

bool FTempoROSModule::IsROSInitialized()
{
	// rclcpp::ok() reports whether the global default context is valid: false before rclcpp::init succeeds,
	// after rclcpp::shutdown, and after rclcpp's own signal handler shuts the context down.
	return rclcpp::ok();
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
			SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_cyclonedds_cpp"));
			break;
		}
	case ERMWImplementation::FastRTPS:
		{
			SetEnvironmentVar(TEXT("RMW_IMPLEMENTATION"), TEXT("rmw_fastrtps_cpp"));
			break;
		}
	}

#if PLATFORM_LINUX
	// CYCLONEDDS_URI
	const FString CycloneDDS_URI = TempoROSSettings->GetCycloneDDS_URI();
	if (!CycloneDDS_URI.IsEmpty())
	{
		if (FPaths::FileExists(CycloneDDS_URI))
		{
			SetEnvironmentVar(TEXT("CYCLONEDDS_URI"), *FString::Printf(TEXT("file://%s"), *TempoROSSettings->GetCycloneDDS_URI()));
		}
		else
		{
			UE_LOG(LogTempoROS, Error, TEXT("Configured CycloneDDS URI file not found: %s"), *CycloneDDS_URI);
		}
	}
#endif

	// ROS_DOMAIN_ID
	SetEnvironmentVar(TEXT("ROS_DOMAIN_ID"), *FString::FromInt(TempoROSSettings->GetROSDomainID()));

	try
	{
		rclcpp::init(0, nullptr);
	}
	catch (const std::exception& E)
	{
		// Leave bROSInitialized false. Otherwise the failure is silent here and only resurfaces later,
		// as a confusing error from every UTempoROSNode::Create call against an uninitialized context.
		UE_LOG(LogTempoROS, Error, TEXT("Failed to initialize rclcpp with error: %s. TempoROS will be unavailable."), UTF8_TO_TCHAR(E.what()));
		return;
	}

	bROSInitialized = true;
}

void FTempoROSModule::ShutdownROS()
{
	if (bROSInitialized)
	{
		try
		{
			rclcpp::shutdown();
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to shut down rclcpp with error: %s"), UTF8_TO_TCHAR(E.what()));
		}

		bROSInitialized = false;
	}
	else
	{
		UE_LOG(LogTempoROS, Warning, TEXT("ShutdownROS called when ROS was not initialized"));
	}
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FTempoROSModule, TempoROS)
