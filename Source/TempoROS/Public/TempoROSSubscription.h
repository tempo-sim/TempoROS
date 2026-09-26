// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROS.h"
#include "TempoROSAllocator.h"
#include "TempoROSConversion.h"
#include "TempoROSTypes.h"

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
	// Do not register any QOS event callbacks (and do not let rclcpp register its defaults). Doing so
	// makes rclcpp's header code, compiled into the calling Unreal module and so allocating with that
	// module's operator new (FMemory), insert into maps that ~SubscriptionBase() / ~PublisherBase() free
	// inside the prebuilt rclcpp library, which corrupts the heap when the entity is destroyed
	// (https://github.com/tempo-sim/TempoROS/issues/82).
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
