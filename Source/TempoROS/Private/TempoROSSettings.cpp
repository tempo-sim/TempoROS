// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSSettings.h"

#if WITH_EDITOR
#include "ISettingsEditorModule.h"
#endif

#include "TempoROS.h"

UTempoROSSettings::UTempoROSSettings()
#if PLATFORM_MAC
		: RMWImplementation(ERMWImplementation::CycloneDDS)
#else
		: RMWImplementation(ERMWImplementation::FastRTPS)
#endif
{
	CategoryName = TEXT("Plugins");
	SectionName = TEXT("Tempo ROS");
}

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
#if WITH_EDITOR
		FModuleManager::GetModuleChecked<ISettingsEditorModule>("SettingsEditor").OnApplicationRestartRequired();
#endif
#endif
	}
	else if (PropertyChangedEvent.MemberProperty->GetName() == GET_MEMBER_NAME_CHECKED(UTempoROSSettings, CycloneDDS_URI) &&
		RMWImplementation == ERMWImplementation::CycloneDDS)
	{
		TempoROSSettingsChangedEvent.Broadcast();
#if WITH_EDITOR
		FModuleManager::GetModuleChecked<ISettingsEditorModule>("SettingsEditor").OnApplicationRestartRequired();
#endif
	}
	else if (PropertyChangedEvent.MemberProperty->GetName() == GET_MEMBER_NAME_CHECKED(UTempoROSSettings, ROSDomainID))
	{
		TempoROSSettingsChangedEvent.Broadcast();
#if WITH_EDITOR
		FModuleManager::GetModuleChecked<ISettingsEditorModule>("SettingsEditor").OnApplicationRestartRequired();
#endif
	}
}
#endif
