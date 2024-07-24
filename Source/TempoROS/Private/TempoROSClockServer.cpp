// Copyright Tempo Simulation, LLC. All Rights Reserved

#include "TempoROSClockServer.h"

#include "TempoROSCommonConverters.h"

#include "rosgraph_msgs/msg/clock.hpp"

template <>
struct TImplicitToROSConverter<double> : TToROSConverter<rosgraph_msgs::msg::Clock, double>
{
	static ToType Convert(const FromType& FromValue)
	{
		ToType ROSType;
		ROSType.clock = TToROSConverter<builtin_interfaces::msg::Time, double>::Convert(FromValue);
		return ROSType;
	}
};

bool UTempoROSClockServer::ShouldCreateSubsystem(UObject* Outer) const
{
	const EWorldType::Type WorldType = Outer->GetWorld()->WorldType;
	if (WorldType == EWorldType::Game || WorldType == EWorldType::PIE)
	{
		return Super::ShouldCreateSubsystem(Outer);
	}

	return false;
}

void UTempoROSClockServer::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	ROSNode = UTempoROSNode::Create("TempoROSClockServer", this);
	ROSNode->AddPublisher<double>("clock", FROSQOSProfile().Reliable().TransientLocal(), false);

	// This will be called:
	// - Before the ROS nodes spin
	// - Before Actor Ticks
	// - Not when paused
	FWorldDelegates::OnWorldPreActorTick.AddUObject(this, &UTempoROSClockServer::OnWorldPreActorTick);
}

void UTempoROSClockServer::OnWorldPreActorTick(UWorld* World, ELevelTick TickType, float DeltaTime)
{
	const EWorldType::Type WorldType = World->WorldType;
	if (WorldType == EWorldType::Game || WorldType == EWorldType::PIE)
	{
		ROSNode->Publish("Clock", World->GetTimeSeconds());
	}
}
