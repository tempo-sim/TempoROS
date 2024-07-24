// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

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
		: Subscription(Node->create_subscription<ROSMessageType>(TCHAR_TO_UTF8(*Topic), 0, [Callback](const ROSMessageType& Message)
		{
			Callback.ExecuteIfBound(TImplicitFromROSConverter<MessageType>::Convert(Message));
		})) {}
	
private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType>> Subscription;
};
