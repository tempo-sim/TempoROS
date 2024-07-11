// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROS.h"
#include "TempoROSPublisher.h"
#include "TempoROSService.h"
#include "TempoROSSubscription.h"

#include "rclcpp.h"
#include "image_transport/image_transport.hpp"

#include "TempoROSNode.generated.h"

UCLASS(BlueprintType)
class TEMPOROS_API UTempoROSNodeBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	UFUNCTION(BlueprintCallable, meta=(WorldContext="Owner"))
	static UTempoROSNode* CreateTempoROSNode(const FString& NodeName, UObject* Owner, bool bAutoTick=true);
};

UCLASS(BlueprintType)
class TEMPOROS_API UTempoROSNode: public UObject, public IPublisherSupportInterface
{
	GENERATED_BODY()
public:
	UTempoROSNode() = default;

	static UTempoROSNode* Create(const FString& NodeName,
								 UObject* Outer=GetTransientPackage(),
							     bool bAutoTick=true,
							     const rclcpp::NodeOptions& NodeOptions=rclcpp::NodeOptions());

	const TMap<FString, TUniquePtr<FTempoROSPublisher>>& GetPublishers() const { return Publishers; }

	UFUNCTION(BlueprintCallable)
	TSet<FString> GetPublishedTopics() const;
	
	template <typename MessageType>
	void AddPublisher(const FString& Topic)
	{
		if (Publishers.Contains(Topic))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has publisher for topic %s"), *Topic);
			return;
		}
		Publishers.Emplace(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic));
	}

	UFUNCTION(BlueprintCallable)
	void RemovePublisher(const FString& Topic);

	template <typename MessageType>
	void Publish(const FString& Topic, const MessageType& Message)
	{
		const TUniquePtr<FTempoROSPublisher>& Publisher = Publishers.FindOrAdd(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic));
		PublishInternal<MessageType>(Topic, Message);
	}
	
	template <typename MessageType>
	void AddSubscription(const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback)
	{
		Subscriptions.FindOrAdd(Topic).Emplace(MakeUnique<TTempoROSSubscription<MessageType>>(Node, Topic, Callback));
	}

	UFUNCTION(BlueprintCallable)
	void RemoveSubscriptions(const FString& Topic);

	template <typename ServiceType>
	void AddService(const FString& Name, const TROSServiceDelegate<ServiceType>& Callback)
	{
		if (Services.Contains(Name))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has service with name %s"), *Name);
			return;
		}
		Services.Emplace(Name, MakeUnique<TTempoROSService<ServiceType>>(Node, Name, Callback));
	}

	UFUNCTION(BlueprintCallable)
	void Tick(float DeltaTime) const;

	virtual const std::shared_ptr<rclcpp::Node>& GetNode() const override { return Node; }
	virtual const std::unique_ptr<image_transport::ImageTransport>& GetImageTransport() const override { return ImageTransport; }

private:
	void Init(const FString& NodeName, const rclcpp::NodeOptions& NodeOptions, UWorld* TickWithWorld);

	template <typename MessageType>
	void PublishInternal(const FString& Topic, const MessageType& Message)
	{
		if (const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(Publishers[Topic].Get()))
		{
			TypedPublisher->Publish(Message);
			return;
		}
		UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
	}

	TMap<FString, TUniquePtr<FTempoROSPublisher>> Publishers;
	TMap<FString, image_transport::Publisher> ImagePublishers;
	TMap<FString, TArray<TUniquePtr<FTempoROSSubscription>>> Subscriptions;
	TMap<FString, TUniquePtr<FTempoROSService>> Services;

	std::shared_ptr<rclcpp::Node> Node = nullptr;
	std::unique_ptr<image_transport::ImageTransport> ImageTransport;
};
