// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROS.h"
#include "TempoROSPublisher.h"
#include "TempoROSSubscription.h"
#include "TempoROSService.h"

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
			UE_LOG(LogTempoROS, Error, TEXT("Node %s already has publisher for topic"), *Topic);
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
	TTempoROSService<ServiceType> AddService(const FString& Topic, const TROSServiceDelegate<ServiceType>& Callback)
	{
		return TTempoROSService<ServiceType>(Node, Topic, Callback);
	}

	template <typename MessageType>
	bool Publish(const FString& Topic, const MessageType& Message)
	{
		if (const auto Publisher = Publishers.Find(Topic))
		{
			if (const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(Publisher))
			{
				TypedPublisher->Publish(Message);
				return true;
			}
			UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
			return false;
		}
		UE_LOG(LogTempoROS, Error, TEXT("No publisher found for topic %s"), *Topic);
		return false;
	}

	// bool Publish(const FString& Topic, const FTempoROSMessage& Message)
	// {
	// 	if (const auto Publisher = Publishers.Find(Topic))
	// 	{
	// 		if (Publisher->GetMessageType() == Message.GetType())
	// 		{
	// 			Publisher->Publish((void*)&Message);
	// 		}
	// 		// if (const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(Publisher))
	// 		// {
	// 		// 	TypedPublisher->Publish(Message);
	// 		// 	return true;
	// 		// }
	// 		UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
	// 		return false;
	// 	}
	// 	UE_LOG(LogTempoROS, Error, TEXT("No publisher found for topic %s"), *Topic);
	// 	return false;
	// }

	void Tick() const;

private:
	TMap<FString, FTempoROSPublisher> Publishers;
	TMap<FString, TArray<FTempoROSSubscription>> Subscriptions;
	
	std::shared_ptr<rclcpp::Node> Node = nullptr;
};
