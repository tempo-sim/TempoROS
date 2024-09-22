// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "TempoROSAllocator.h"

#include "rclcpp.h"

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
		UnrealMemoryResource mem_resource{};
		auto alloc = std::make_shared<std::pmr::polymorphic_allocator<void>>(&mem_resource);
		rclcpp::SubscriptionOptionsWithAllocator<std::pmr::polymorphic_allocator<void>> subscription_options;
		subscription_options.allocator = alloc;
		subscription_options.use_default_callbacks = false;
		auto msg_mem_strat = std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageType, std::pmr::polymorphic_allocator<void>>>(alloc);
		Subscription = Node->create_subscription<ROSMessageType>(
			TCHAR_TO_UTF8(*Topic),
			10,
			[Callback](const ROSMessageType& Message)
			{
			  Callback.ExecuteIfBound(TImplicitFromROSConverter<MessageType>::Convert(Message));
			},
			subscription_options,
			msg_mem_strat);
	}
	
private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType, std::pmr::polymorphic_allocator<void>>> Subscription;
};
