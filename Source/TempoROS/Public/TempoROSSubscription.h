// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"

template <class MessageType>
using TROSSubscriptionDelegate = TDelegate<void(const MessageType&)>;

struct FTempoROSSubscription
{
	virtual ~FTempoROSSubscription() = default;
	virtual FName GetMessageType() const { return FName(NAME_None); }
};

template <typename MessageType>
struct TEMPOROS_API TTempoROSSubscription : FTempoROSSubscription
{
	using ROSMessageType = typename TFromROSConverter<MessageType>::FromType;
	
	TTempoROSSubscription(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic, const TROSSubscriptionDelegate<MessageType>& Callback)
		: Subscription(Node->create_subscription<ROSMessageType>(TCHAR_TO_UTF8(*Topic), 0, [Callback](const ROSMessageType& Message)
		{
			Callback.ExecuteIfBound(TFromROSConverter<MessageType>::Convert(Message));
		})) {}

	virtual FName GetMessageType() const override
	{
		return TMessageTypeTraits<MessageType>::MessageTypeDescriptor;
	}
	
private:
	std::shared_ptr<rclcpp::Subscription<ROSMessageType>> Subscription;
};

template <typename MessageType>
static TTempoROSSubscription<MessageType>* Cast(FTempoROSSubscription* Subscription)
{
	if (Subscription->GetMessageType() == TMessageTypeTraits<MessageType>::MessageTypeDescriptor)
	{
		return StaticCast<TTempoROSSubscription<MessageType>*>(Subscription);
	}
	return nullptr;
}
