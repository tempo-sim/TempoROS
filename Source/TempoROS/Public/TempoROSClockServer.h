// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSNode.h"

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "TempoROSClockServer.generated.h"

UCLASS()
class TEMPOROS_API UTempoROSClockServer : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	virtual void OnWorldPreActorTick(UWorld* World, ELevelTick TickType, float DeltaTime);

private:
	UPROPERTY()
	UTempoROSNode* ROSNode;
};
