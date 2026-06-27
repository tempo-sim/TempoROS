// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSAllocator.h"
#include "TempoROSConversion.h"

#include "rclcpp.h"

#if PLATFORM_LINUX && ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION < 6
namespace std::pmr
{
	template <class _ValueT>
	using polymorphic_allocator = std::experimental::pmr::polymorphic_allocator<_ValueT>;
}
#endif

inline rclcpp::SubscriptionOptions TempoROSSubscriptionOptions()
{
	// rclcpp::SubscriptionOptions defaults its allocator type to std::pmr::polymorphic_allocator<void>,
	// which picks up the default memory resource set in SetUnrealDefaultMemoryResource() at startup. The
	// default message memory strategy likewise default-constructs its allocator from that resource.
	rclcpp::SubscriptionOptions SubscriptionOptions;
	SubscriptionOptions.use_default_callbacks = false;
	return SubscriptionOptions;
}

template <class MessageType>
using TROSSubscriptionDelegate = TDelegate<void(const MessageType&)>;

struct FTempoROSSubscription
{
	virtual ~FTempoROSSubscription() = default;
};

template <typename MessageType>
struct TTempoROSSubscription : FTempoROSSubscription
{
	using ROSMessageType = typename TImplicitFromROSConverter<MessageType>::FromType;

	TTempoROSSubscription(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback, const FROSQOSProfile& QOSProfile)
	{
		Subscription = Node->create_subscription<ROSMessageType>(
			TCHAR_TO_UTF8(*Topic),
			QOSProfile.ToROS(),
			[Callback](const ROSMessageType& Message)
			{
			  Callback.ExecuteIfBound(TImplicitFromROSConverter<MessageType>::Convert(Message));
			},
			TempoROSSubscriptionOptions()
		);
	}

private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType>> Subscription;
};
