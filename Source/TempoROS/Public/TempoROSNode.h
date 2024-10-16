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
							     const rclcpp::NodeOptions& NodeOptions=rclcpp::NodeOptions(GetUnrealAllocator()));

	const TMap<FString, TUniquePtr<FTempoROSPublisher>>& GetPublishers() const { return Publishers; }

	UFUNCTION(BlueprintCallable)
	TSet<FString> GetPublishedTopics() const;
	
	template <typename MessageType>
	bool AddPublisher(const FString& Topic, const FROSQOSProfile& QOSProfile=FROSQOSProfile(), bool bPrependNodeName=true)
	{
		if (TMessageTypeTraits<MessageType>::MessageTypeDescriptor == NAME_None)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Attempted to create a publisher for a type with missing type traits. Use DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS to define."));
			return false;
		}
		if (Publishers.Contains(Topic))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has publisher for topic %s"), *Topic);
			return false;
		}
		try
		{
			Publishers.Emplace(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic, QOSProfile, bPrependNodeName));
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to create publisher for topic %s with error: %s"), *Topic, UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	UFUNCTION(BlueprintCallable)
	void RemovePublisher(const FString& Topic);

	template <typename MessageType>
	bool Publish(const FString& Topic, const MessageType& Message)
	{
		TUniquePtr<FTempoROSPublisher>* PublisherPtr = Publishers.Find(Topic);
		if (!PublisherPtr)
		{
			try
			{
				PublisherPtr = &Publishers.Emplace(Topic, MakeUnique<TTempoROSPublisher<MessageType>>(this, Topic, FROSQOSProfile(), true));
			}
			catch (const std::exception& E)
			{
				UE_LOG(LogTempoROS, Error, TEXT("Failed to create publisher for topic %s with error: %s"), *Topic, UTF8_TO_TCHAR(E.what()));
				return false;
			}
		}
		const TTempoROSPublisher<MessageType>* TypedPublisher = Cast<MessageType>(PublisherPtr->Get());
		if (!TypedPublisher)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Publisher for topic %s did not have correct type"), *Topic);
			return false;
		}
		{
			try
			{
				TypedPublisher->Publish(Message);
			}
			catch (const std::exception& E)
			{
				UE_LOG(LogTempoROS, Error, TEXT("Failed to publish on topic %s with error: %s"), *Topic, UTF8_TO_TCHAR(E.what()));
				return false;
			}
		}
		return true;
	}
	
	template <typename MessageType>
	bool AddSubscription(const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback, const FROSQOSProfile& QOSProfile=FROSQOSProfile())
	{
		try
		{
			Subscriptions.FindOrAdd(Topic).Emplace(MakeUnique<TTempoROSSubscription<MessageType>>(Node, Topic, Callback, QOSProfile));
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to create subscription for topic %s with error: %s"), *Topic, UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	// Remove all subscriptions for a topic. TODO: Support removing individual subscriptions.
	UFUNCTION(BlueprintCallable)
	void RemoveSubscriptions(const FString& Topic);

	template <typename ServiceType>
	bool AddService(const FString& Name, const TROSServiceDelegate<ServiceType>& Callback)
	{
		if (Services.Contains(Name))
		{
			UE_LOG(LogTempoROS, Error, TEXT("Node already has service with name %s"), *Name);
			return false;
		}
		try
		{
			 Services.Emplace(Name, MakeUnique<TTempoROSService<ServiceType>>(Node, Name, Callback));
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to create service with error: %s"), UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	// Publish the "static" transform, which will be latched and provided to all new listeners, between the To and From
	// frames at the specified time. Timestamp=0.0 (the default) means "now".
	UFUNCTION(BlueprintCallable, BlueprintPure=false, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	bool PublishStaticTransform(const FTransform& Transform, const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
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
		try
		{
			StaticTFPublisher->PublishTransform(FStampedTransform(TimestampResolved, FromFrameResolved, ToFrame, Transform));
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to publish static transform with error: %s"), UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	// Publish the "dynamic" transform, which will not be latched, between the To and From
	// frames at the specified time. Timestamp=0.0 (the default) means "now".
	UFUNCTION(BlueprintCallable, BlueprintPure=false, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	bool PublishDynamicTransform(const FTransform& Transform, const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
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
		try
		{
			DynamicTFPublisher->PublishTransform(FStampedTransform(TimestampResolved, FromFrameResolved, ToFrame, Transform));
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to publish dynamic transform with error: %s"), UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	// Get the transform between the To and From frames at the specified time.
	// Timestamp=0.0 (the default) gets the latest transform.
	UFUNCTION(BlueprintCallable, meta=(AutoCreateRefTerm="FromFrame,Timestamp", HidePin="Timestamp"))
	bool GetTransform(FTransform& Transform, const FString& ToFrame, const FString& FromFrame="", double Timestamp=0.0) const
	{
		FString FromFrameResolved = FromFrame;
		if (FromFrameResolved.IsEmpty())
		{
			FromFrameResolved = GetDefault<UTempoROSSettings>()->GetFixedFrameName();
		}
		try
		{
			return TFListener->GetTransform(FromFrameResolved, ToFrame, Timestamp, Transform);
		}
		catch (const std::exception& E)
		{
			UE_LOG(LogTempoROS, Error, TEXT("Failed to publish dynamic transform with error: %s"), UTF8_TO_TCHAR(E.what()));
			return false;
		}
		return true;
	}

	UFUNCTION(BlueprintCallable)
	void Tick(float DeltaTime) const;

protected:
	/* IPublisherSupportInterface */
	template <typename T>
	friend struct TTempoROSPublisher;
	virtual const std::shared_ptr<rclcpp::Node>& GetNode() const override { return Node; }
	virtual const std::unique_ptr<image_transport::ImageTransport>& GetImageTransport() const override { return ImageTransport; }

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
