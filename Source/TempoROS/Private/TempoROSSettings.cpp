// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSSettings.h"

#include "TempoROS.h"

#if WITH_EDITOR
void UTempoROSSettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.MemberProperty->GetName() == GET_MEMBER_NAME_CHECKED(UTempoROSSettings, RMWImplementation))
	{
#if PLATFORM_MAC
		RMWImplementation = ERMWImplementation::CycloneDDS;
		UE_LOG(LogTempoROS, Error, TEXT("CycloneDDS is the only supported RMW implementation on Mac"));
#else
		TempoROSSettingsChangedEvent.Broadcast();
#endif
	}
	else if (PropertyChangedEvent.MemberProperty->GetName() == GET_MEMBER_NAME_CHECKED(UTempoROSSettings, CycloneDDS_URI) &&
		RMWImplementation == ERMWImplementation::CycloneDDS)
	{
		TempoROSSettingsChangedEvent.Broadcast();
	}
	else if (PropertyChangedEvent.MemberProperty->GetName() == GET_MEMBER_NAME_CHECKED(UTempoROSSettings, ROSDomainID))
	{
		TempoROSSettingsChangedEvent.Broadcast();
	}
}
#endif
