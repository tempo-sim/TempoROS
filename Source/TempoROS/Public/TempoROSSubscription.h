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

inline rclcpp::SubscriptionOptions TempoROSSubscriptionOptions(const FString& Topic)
{
	// rclcpp::SubscriptionOptions defaults its allocator type to std::pmr::polymorphic_allocator<void>,
	// which picks up the default memory resource set in SetUnrealDefaultMemoryResource() at startup. The
	// default message memory strategy likewise default-constructs its allocator from that resource.
	rclcpp::SubscriptionOptions SubscriptionOptions;
	// rclcpp's own default callbacks log through rcutils, which does not reach the Unreal log. Supply our
	// own instead: a QOS mismatch does not fail subscription creation, so without this the subscription
	// simply never receives anything, with nothing to explain why.
	SubscriptionOptions.use_default_callbacks = false;
	// Not on Windows, though: registering any event callback makes rclcpp's header code insert into a map
	// that ~SubscriptionBase() / ~PublisherBase() then frees inside the prebuilt rclcpp DLL. Each Unreal
	// module has its own operator new/delete (FMemory) on Windows while that DLL uses the CRT heap, so
	// destroying the entity corrupts the heap (https://github.com/tempo-sim/TempoROS/issues/82). Restore
	// this once the rclcpp dependency allocates those maps through std::pmr.
#if !PLATFORM_WINDOWS
	SubscriptionOptions.event_callbacks.incompatible_qos_callback = [Topic](rclcpp::QOSRequestedIncompatibleQoSInfo& Info)
	{
		UE_LOG(LogTempoROS, Warning, TEXT("Discovered a publisher on topic %s whose QOS is incompatible with our subscription's (policy: %s). No messages will be received from it."),
			*Topic, QOSPolicyKindName(Info.last_policy_kind));
	};
#endif
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
			TempoROSSubscriptionOptions(Topic)
		);
	}

private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType>> Subscription;
};
