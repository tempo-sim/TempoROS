// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSTypes.h"

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "TempoROSSettings.generated.h"

DECLARE_MULTICAST_DELEGATE(FTempoROSSettingsChanged);

UCLASS(Config=Game)
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
	int32 GetROSDomainID() const { return ROSDomainID; }
	FTempoROSSettingsChanged TempoROSSettingsChangedEvent;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	
private:
	UPROPERTY(EditAnywhere, Config)
	FString FixedFrameName = TEXT("map");
	
	UPROPERTY(EditAnywhere, Config, AdvancedDisplay, meta=(ClampMin=0, ClampMax=101, UIMin=0, UIMax=101))
	int32 ROSDomainID = 0;
	
	UPROPERTY(EditAnywhere, Config, AdvancedDisplay)
	ERMWImplementation RMWImplementation;
};
