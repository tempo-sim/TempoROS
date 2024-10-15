// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSTypes.h"

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "TempoROSSettings.generated.h"

DECLARE_MULTICAST_DELEGATE(FTempoROSSettingsChanged);

UCLASS(Config=Game, DisplayName="Tempo ROS")
class TEMPOROS_API UTempoROSSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UTempoROSSettings()
#if PLATFORM_MAC
		: RMWImplementation(ERMWImplementation::CycloneDDS)
#else
		: RMWImplementation(ERMWImplementation::FastRTPS)
#endif
	{}

	const FString& GetFixedFrameName() const { return FixedFrameName; }
	ERMWImplementation GetRMWImplementation() const { return RMWImplementation; }
	FString GetCycloneDDS_URI() const { return CycloneDDS_URI.FilePath; }
	int32 GetROSDomainID() const { return ROSDomainID; }
	FTempoROSSettingsChanged TempoROSSettingsChangedEvent;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	
private:
	// The name of the special "fixed" coordinate name to which all ROS TF transforms are relative.
	UPROPERTY(EditAnywhere, Config)
	FString FixedFrameName = TEXT("map");

	// You can have multiple ROS domains on a single local network by providing a unique ID for each.
	UPROPERTY(EditAnywhere, Config, meta=(ClampMin=0, ClampMax=101, UIMin=0, UIMax=101))
	int32 ROSDomainID = 0;

	// The middleware implementation to use. Note that on Mac only CycloneDDS is supported.
	UPROPERTY(EditAnywhere, Config)
	ERMWImplementation RMWImplementation;

	// Additional configuration, including shared memory support, for CycloneDDS middleware.
	UPROPERTY(EditAnywhere, Config)
	FFilePath CycloneDDS_URI;
};
