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

	// Whether there is a valid ROS context. When false no ROS object may be created or spun. Callers
	// must check this (or the result of UTempoROSNode::Create) rather than relying on rclcpp to throw,
	// since an escaping rclcpp exception takes down the process.
	static bool IsROSInitialized();

private:
	void InitROS();
	void ShutdownROS();

	bool bROSInitialized = false;
};
