// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROS.h"
#include "TempoROSPublisher.h"
#include "TempoROSService.h"
#include "TempoROSSubscription.h"
#include "TempoTF.h"

#include "rclcpp.h"
#include "TempoROSSettings.h"
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
	void AddPublisher(const FString& Topic, const FROSQOSProfile& QOSProfile=FROSQOSProfile(), bool bPrependNodeName=true)
	{
		if (TMessageTypeTraits<MessageType>::MessageTypeDescriptor == NAME_None)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Attempted to create a publisher for a type with missing type traits. Use DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS to define."));
			return;
		}
		if (Publishers.Contains(Topic))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has publisher for topic %s"), *Topic);
			return;
		}
		Publishers.Emplace(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic, QOSProfile, bPrependNodeName));
	}

	UFUNCTION(BlueprintCallable)
	void RemovePublisher(const FString& Topic);

	template <typename MessageType>
	void Publish(const FString& Topic, const MessageType& Message)
	{
		TUniquePtr<FTempoROSPublisher>* PublisherPtr = Publishers.Find(Topic);
		if (!PublisherPtr)
		{
			PublisherPtr = &Publishers.Emplace(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic, FROSQOSProfile(), true));
		}
		if (const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(PublisherPtr->Get()))
		{
			TypedPublisher->Publish(Message);
			return;
		}
		UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
	}
	
	template <typename MessageType>
	void AddSubscription(const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback)
	{
		Subscriptions.FindOrAdd(Topic).Emplace(MakeUnique<TTempoROSSubscription<MessageType>>(Node, Topic, Callback));
	}

	// Remove all subscriptions for a topic. TODO: Support removing individual subscriptions. 
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

	// Publish the "static" transform, which will be latched and provided to all new listeners, between the To and From
	// frames at the specified time. Timestamp=0.0 (the default) means "now".
	UFUNCTION(BlueprintCallable, BlueprintPure=false, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	void PublishStaticTransform(const FTransform& Transform, const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
	{
		FString FromFrameResolved = FromFrame;
		if (FromFrameResolved.IsEmpty())
		{
			FromFrameResolved = GetDefault<UTempoROSSettings>()->GetFixedFrameName();
		}
		double TimestampResolved = Timestamp;
		if (TimestampResolved == 0.0)
		{
			TimestampResolved = GetWorld()->GetTimeSeconds();
		}
		StaticTFPublisher->PublishTransform(FStampedTransform(TimestampResolved, FromFrameResolved, ToFrame, Transform));
	}

	// Publish the "dynamic" transform, which will not be latched, between the To and From
	// frames at the specified time. Timestamp=0.0 (the default) means "now".
	UFUNCTION(BlueprintCallable, BlueprintPure=false, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	void PublishDynamicTransform(const FTransform& Transform, const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
	{
		FString FromFrameResolved = FromFrame;
		if (FromFrameResolved.IsEmpty())
		{
			FromFrameResolved = GetDefault<UTempoROSSettings>()->GetFixedFrameName();
		}
		double TimestampResolved = Timestamp;
		if (TimestampResolved == 0.0)
		{
			TimestampResolved = GetWorld()->GetTimeSeconds();
		}
		DynamicTFPublisher->PublishTransform(FStampedTransform(TimestampResolved, FromFrameResolved, ToFrame, Transform));
	}

	// Get the transform between the To and From frames at the specified time.
	// Timestamp=0.0 (the default) gets the latest transform.
	UFUNCTION(BlueprintCallable, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	FTransform GetTransform(const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
	{
		FString FromFrameResolved = FromFrame;
		if (FromFrameResolved.IsEmpty())
		{
			FromFrameResolved = GetDefault<UTempoROSSettings>()->GetFixedFrameName();
		}
		return TFListener->GetTransform(FromFrameResolved, ToFrame, Timestamp);
	}

	UFUNCTION(BlueprintCallable)
	void Tick(float DeltaTime) const;

	virtual const std::shared_ptr<rclcpp::Node>& GetNode() const override { return Node; }
	virtual const std::unique_ptr<image_transport::ImageTransport>& GetImageTransport() const override { return ImageTransport; }

private:
	void Init(const FString& NodeName, const rclcpp::NodeOptions& NodeOptions, UWorld* TickWithWorld);

	TMap<FString, TUniquePtr<FTempoROSPublisher>> Publishers;
	TMap<FString, TArray<TUniquePtr<FTempoROSSubscription>>> Subscriptions;
	TMap<FString, TUniquePtr<FTempoROSService>> Services;

	std::shared_ptr<rclcpp::Node> Node = nullptr;
	std::unique_ptr<image_transport::ImageTransport> ImageTransport;

	TUniquePtr<FTempoStaticTFPublisher> StaticTFPublisher;
	TUniquePtr<FTempoDynamicTFPublisher> DynamicTFPublisher;
	TUniquePtr<FTempoTFListener> TFListener;
};
