// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSAllocator.h"
#include "TempoROSConversion.h"

#include "rclcpp.h"

#if PLATFORM_LINUX
namespace std::pmr
{
  template <class _ValueT>
  using polymorphic_allocator = std::experimental::pmr::polymorphic_allocator<_ValueT>;
}
#endif

inline rclcpp::SubscriptionOptions TempoROSSubscriptionOptions(const std::shared_ptr<std::pmr::polymorphic_allocator<void>>& Allocator)
{
	rclcpp::SubscriptionOptionsWithAllocator<std::pmr::polymorphic_allocator<void>> SubscriptionOptions;
	SubscriptionOptions.allocator = Allocator;
	SubscriptionOptions.use_default_callbacks = false;
	return SubscriptionOptions;
}

template <typename ROSMessageType>
inline std::shared_ptr<rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageType>>
TempoROSSubscriptionMemoryStrategy(const std::shared_ptr<std::pmr::polymorphic_allocator<void>>& Allocator)
{
	return std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageType>>(Allocator);
}

template <class MessageType>
using TROSSubscriptionDelegate = TDelegate<void(const MessageType&)>;

struct FTempoROSSubscription
{
	virtual ~FTempoROSSubscription() = default;
};

template <typename MessageType>
struct TEMPOROS_API TTempoROSSubscription : FTempoROSSubscription
{
	using ROSMessageType = typename TImplicitFromROSConverter<MessageType>::FromType;
	
	TTempoROSSubscription(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback)
	{
		std::shared_ptr<std::pmr::polymorphic_allocator<void>> Allocator = GetPolymorphicUnrealAllocator();
		Subscription = Node->create_subscription<ROSMessageType>(
			TCHAR_TO_UTF8(*Topic),
			0,
			[Callback](const ROSMessageType& Message)
			{
			  Callback.ExecuteIfBound(TImplicitFromROSConverter<MessageType>::Convert(Message));
			},
			TempoROSSubscriptionOptions(Allocator),
			TempoROSSubscriptionMemoryStrategy<ROSMessageType>(Allocator));
	}
	
private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType>> Subscription;
};
