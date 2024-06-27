// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"

struct FTempoROSPublisher
{
	virtual ~FTempoROSPublisher() = default;
	virtual FName GetMessageType() const { return FName(NAME_None); }
};

template <typename MessageType>
struct TTempoROSPublisher : FTempoROSPublisher
{
	using ROSMessageType = typename TToROSConverter<MessageType>::ToType;
	
	TTempoROSPublisher(const std::shared_ptr<rclcpp::Node>& Node, const FString& Topic)
		: Publisher(Node->create_publisher<ROSMessageType>(TCHAR_TO_UTF8(*Topic), 0)) {}
	
	void Publish(const MessageType& Message) const
	{
		Publisher->publish(TToROSConverter<MessageType>::Convert(Message));
	}
	
	virtual FName GetMessageType() const override
	{
		return TMessageTypeTraits<MessageType>::MessageTypeDescriptor;
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
