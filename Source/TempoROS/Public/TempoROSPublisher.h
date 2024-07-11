// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/Image.hpp"

struct IPublisherSupportInterface
{
	virtual ~IPublisherSupportInterface() = default;
	virtual const std::shared_ptr<rclcpp::Node>& GetNode() const = 0;
	virtual const std::unique_ptr<image_transport::ImageTransport>& GetImageTransport() const = 0;
};

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

template<typename MessageType>
concept ImageConvertible = std::is_same_v<typename TImplicitToROSConverter<MessageType>::ToType, sensor_msgs::msg::Image>;

template <typename MessageType>
struct TTempoROSPublisher : FTempoROSPublisher
{
	using ROSMessageType = typename TImplicitToROSConverter<MessageType>::ToType;
	
	TTempoROSPublisher(const IPublisherSupportInterface* PublisherSupport, const FString& Topic)
		: Publisher(PublisherSupport->GetNode()->create_publisher<ROSMessageType>(TCHAR_TO_UTF8(*PrependNodeName(PublisherSupport->GetNode(), Topic)), 0)) {}
	
	void Publish(const MessageType& Message) const
	{
		Publisher->publish(TImplicitToROSConverter<MessageType>::Convert(Message));
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

template <ImageConvertible MessageType>
struct TTempoROSPublisher<MessageType> : FTempoROSPublisher
{
	TTempoROSPublisher(const IPublisherSupportInterface* PublisherSupport, const FString& Topic)
		: Publisher(PublisherSupport->GetImageTransport()->advertise(TCHAR_TO_UTF8(*PrependNodeName(PublisherSupport->GetNode(), Topic)), 0)) {}
	
	void Publish(const MessageType& Message) const
	{
		Publisher.publish(TImplicitToROSConverter<MessageType>::Convert(Message));
	}
	
	virtual FName GetMessageType() const override
	{
		return TMessageTypeTraits<MessageType>::MessageTypeDescriptor;
	}

	virtual bool HasSubscriptions() const override
	{
		return Publisher.getNumSubscribers() > 0;
	}

private:
	image_transport::Publisher Publisher;
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
