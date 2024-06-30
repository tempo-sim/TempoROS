// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"

static FString PrependNodeName(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic)
{
	return FString::Printf(TEXT("%s/%s"), UTF8_TO_TCHAR(Node->get_name()), *Topic);
}

struct FTempoROSPublisher
{
	virtual ~FTempoROSPublisher() = default;
	virtual FName GetMessageType() const { return FName(NAME_None); }
	virtual bool HasSubscriptions() const { return false; }
};

template <typename MessageType>
struct TTempoROSPublisher : FTempoROSPublisher
{
	using ROSMessageType = typename TToROSConverter<MessageType>::ToType;
	
	TTempoROSPublisher(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic)
		: Publisher(Node->create_publisher<ROSMessageType>(TCHAR_TO_UTF8(*PrependNodeName(Node, Topic)), 0)) {}
	
	void Publish(const MessageType& Message) const
	{
		Publisher->publish(TToROSConverter<MessageType>::Convert(Message));
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
	std::shared_ptr<rclcpp::Publisher<ROSMessageType>> Publisher;
};

template <typename MessageType>
static TTempoROSPublisher<MessageType>* Cast(FTempoROSPublisher* Publisher)
{
	if (Publisher->GetMessageType() == TMessageTypeTraits<MessageType>::MessageTypeDescriptor)
	{
		return StaticCast<TTempoROSPublisher<MessageType>*>(Publisher);
	}
	return nullptr;
}
