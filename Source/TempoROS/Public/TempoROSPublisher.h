// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "TempoROS.h"
#include "TempoROSAllocator.h"
#include "TempoROSTypes.h"

#include "rclcpp.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

#if PLATFORM_LINUX && ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION < 6
namespace std::pmr
{
	template <class _ValueT>
	using polymorphic_allocator = std::experimental::pmr::polymorphic_allocator<_ValueT>;
}
#endif

inline rclcpp::PublisherOptions TempoROSPublisherOptions(const FString& Topic)
{
	// rclcpp::PublisherOptions defaults its allocator type to std::pmr::polymorphic_allocator<void>,
	// which picks up the default memory resource set in SetUnrealDefaultMemoryResource() at startup.
	rclcpp::PublisherOptions PublisherOptions;
	// See TempoROSSubscriptionOptions: rclcpp's default callbacks do not reach the Unreal log.
	PublisherOptions.use_default_callbacks = false;
	PublisherOptions.event_callbacks.incompatible_qos_callback = [Topic](rclcpp::QOSOfferedIncompatibleQoSInfo& Info)
	{
		UE_LOG(LogTempoROS, Warning, TEXT("Discovered a subscription on topic %s whose QOS is incompatible with our publisher's (policy: %s). It will not receive any messages."),
			*Topic, QOSPolicyKindName(Info.last_policy_kind));
	};
	return PublisherOptions;
}

static FString PrependNodeName(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic)
{
	return FString::Printf(TEXT("%s/%s"), UTF8_TO_TCHAR(Node->get_name()), *Topic);
}

struct IPublisherSupportInterface
{
	virtual ~IPublisherSupportInterface() = default;
	virtual const std::shared_ptr<rclcpp::Node>& GetNode() const = 0;
	virtual const std::unique_ptr<image_transport::ImageTransport>& GetImageTransport() const = 0;
};

struct FTempoROSPublisher
{
	virtual ~FTempoROSPublisher() = default;
	virtual FName GetMessageType() const { return FName(NAME_None); }
	virtual bool HasSubscriptions() const { return false; }
};

template<typename MessageType>
concept ImageConvertible = std::is_same_v<typename TImplicitToROSConverter<MessageType>::ToType, sensor_msgs::msg::Image>;

template <typename MessageType>
struct TTempoROSPublisher : FTempoROSPublisher
{
	using ROSMessageType = typename TImplicitToROSConverter<MessageType>::ToType;

	TTempoROSPublisher(const IPublisherSupportInterface* PublisherSupport, const FString& Topic, const FROSQOSProfile& QOSProfile, bool bPrependNodeName)
		: Node(PublisherSupport->GetNode())
	{
		const FString ResolvedTopic = bPrependNodeName ? PrependNodeName(Node, Topic) : Topic;
		Publisher = Node->create_publisher<ROSMessageType>(
			TCHAR_TO_UTF8(*ResolvedTopic),
			QOSProfile.ToROS(),
			TempoROSPublisherOptions(ResolvedTopic)
		);
		bUseSharedMemory = QOSProfile.bUseSharedMemory;
#if !PLATFORM_LINUX
		if (bUseSharedMemory)
		{
			UE_LOG(LogTempoROS, Warning, TEXT("Shared memory transport is only supported on Linux"));
		}
#endif
	}

	void Publish(const MessageType& Message) const
	{
		if (bUseSharedMemory)
		{
			rclcpp::LoanedMessage<ROSMessageType> LoanedMessage = Publisher->borrow_loaned_message();
			LoanedMessage.get() = TImplicitToROSConverter<MessageType>::Convert(Message);
			Publisher->publish(std::move(LoanedMessage));
		}
		else
		{
			Publisher->publish(TImplicitToROSConverter<MessageType>::Convert(Message));
		}
	}

	virtual FName GetMessageType() const override
	{
		return TMessageTypeTraits<MessageType>::MessageTypeDescriptor;
	}

	virtual bool HasSubscriptions() const override
	{
		return Publisher->get_subscription_count() > 0;
	}

private:
	const std::shared_ptr<rclcpp::Node>& Node;
	std::shared_ptr<rclcpp::Publisher<ROSMessageType>> Publisher;
	bool bUseSharedMemory = false;
};

template <ImageConvertible MessageType>
struct TTempoROSPublisher<MessageType> : FTempoROSPublisher
{
	TTempoROSPublisher(const IPublisherSupportInterface* PublisherSupport, const FString& Topic, const FROSQOSProfile& QOSProfile, bool bPrependNodeName)
		: Node(PublisherSupport->GetNode()), Publisher(PublisherSupport->GetImageTransport()->advertise(
			bPrependNodeName ? TCHAR_TO_UTF8(*PrependNodeName(Node, Topic)) : TCHAR_TO_UTF8(*Topic),
			QOSProfile.QueueSize,
			QOSProfile.Durability == EROSQOSDurability::TransientLocal,
			TempoROSPublisherOptions(bPrependNodeName ? PrependNodeName(Node, Topic) : Topic))) {}

	void Publish(const MessageType& Message) const
	{
		Publisher.publish(TImplicitToROSConverter<MessageType>::Convert(Message));
	}

	virtual FName GetMessageType() const override
	{
		return TMessageTypeTraits<MessageType>::MessageTypeDescriptor;
	}

	virtual bool HasSubscriptions() const override
	{
		return Publisher.getNumSubscribers() > 0;
	}

private:
	const std::shared_ptr<rclcpp::Node>& Node;
	image_transport::Publisher Publisher;
};

template <typename MessageType>
static TTempoROSPublisher<MessageType>* Cast(FTempoROSPublisher* Publisher)
{
	if (TMessageTypeTraits<MessageType>::MessageTypeDescriptor == NAME_None)
	{
		UE_LOG(LogTempoROS, Error, TEXT("Attempted to publish a type with missing type traits. Use DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS to define."));
		return nullptr;
	}
	if (Publisher->GetMessageType() == TMessageTypeTraits<MessageType>::MessageTypeDescriptor)
	{
		return StaticCast<TTempoROSPublisher<MessageType>*>(Publisher);
	}
	return nullptr;
}
