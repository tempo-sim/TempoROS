// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROS.h"
#include "TempoROSPublisher.h"
#include "TempoROSService.h"
#include "TempoROSSubscription.h"

#include "rclcpp.h"

#include "TempoROSNode.generated.h"

UCLASS(BlueprintType)
class TEMPOROS_API UTempoROSNode: public UObject
{
	GENERATED_BODY()
public:
	UTempoROSNode();
	
	template <typename MessageType>
	bool AddPublisher(const FString& Topic)
	{
		if (Publishers.Contains(Topic))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has publisher for topic %s"), *Topic);
			return false;
		}
		Publishers.Add(Topic, TTempoROSPublisher<MessageType>(Node, Topic));
		return true;
	}

	template <typename MessageType>
	bool AddSubscription(const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback)
	{
		Subscriptions.FindOrAdd(Topic).Add(TTempoROSSubscription<MessageType>(Node, Topic, Callback));
		return true;
	}

	template <typename ServiceType>
	bool AddService(const FString& Name, const TROSServiceDelegate<ServiceType>& Callback)
	{
		if (Services.Contains(Name))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node %s already has service with name %s"), *Name);
			return false;
		}
		Services.Add(Name, TTempoROSService<ServiceType>(Node, Name, Callback));
		return true;
	}

	template <typename MessageType>
	bool Publish(const FString& Topic, const MessageType& Message)
	{
		FTempoROSPublisher& Publisher = Publishers.FindOrAdd(Topic, TTempoROSPublisher<MessageType>(Node, Topic));
		if (const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(&Publisher))
		{
			TypedPublisher->Publish(Message);
			return true;
		}
		UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
		return false;
	}

	void Tick() const;

private:
	TMap<FString, FTempoROSPublisher> Publishers;
	TMap<FString, TArray<FTempoROSSubscription>> Subscriptions;
	TMap<FString, FTempoROSService> Services;
	
	std::shared_ptr<rclcpp::Node> Node = nullptr;
};
