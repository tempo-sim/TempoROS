// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "TempoROSAllocator.h"

#include "rclcpp.h"

inline rclcpp::SubscriptionOptions TempoROSSubscriptionOptions(const std::shared_ptr<std::pmr::polymorphic_allocator<void>>& Allocator)
{
	rclcpp::SubscriptionOptionsWithAllocator<std::pmr::polymorphic_allocator<void>> SubscriptionOptions;
	SubscriptionOptions.allocator = Allocator;
	SubscriptionOptions.use_default_callbacks = false;
	return SubscriptionOptions;
}

template <typename ROSMessageType>
inline std::shared_ptr<rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageType, std::pmr::polymorphic_allocator<void>>>
TempoROSSubscriptionMemoryStrategy(const std::shared_ptr<std::pmr::polymorphic_allocator<void>>& Allocator)
{
	return std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageType, std::pmr::polymorphic_allocator<void>>>(Allocator);
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
			10,
			[Callback](const ROSMessageType& Message)
			{
			  Callback.ExecuteIfBound(TImplicitFromROSConverter<MessageType>::Convert(Message));
			},
			TempoROSSubscriptionOptions(Allocator),
			TempoROSSubscriptionMemoryStrategy<ROSMessageType>(Allocator));
	}
	
private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType, std::pmr::polymorphic_allocator<void>>> Subscription;
};
