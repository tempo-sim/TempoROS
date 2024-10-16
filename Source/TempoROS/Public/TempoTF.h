// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSAllocator.h"
#include "TempoROSCommonConverters.h"
#include "TempoROSConversion.h"
#include "TempoROSPublisher.h"
#include "TempoROSSubscription.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

struct FStampedTransform
{
	FStampedTransform(double TimestampIn, const FString& FromFrameIn, const FString& ToFrameIn, const FTransform& TransformIn)
		: Timestamp(TimestampIn), FromFrame(FromFrameIn), ToFrame(ToFrameIn), Transform(TransformIn) {}

	double Timestamp;
	FString FromFrame;
	FString ToFrame;
	FTransform Transform;
};

template <>
struct TImplicitToROSConverter<FStampedTransform> : TToROSConverter<geometry_msgs::msg::TransformStamped, FStampedTransform>
{
	static ToType Convert(const FStampedTransform& TempoTransform)
	{
		geometry_msgs::msg::TransformStamped ROSTransform;
		ROSTransform.transform = TImplicitToROSConverter<FTransform>::Convert(TempoTransform.Transform);
		ROSTransform.header.stamp = TToROSConverter<builtin_interfaces::msg::Time, double>::Convert(TempoTransform.Timestamp);
		ROSTransform.header.frame_id = TToROSConverter<std::string, FString>::Convert(TempoTransform.FromFrame).c_str();
		ROSTransform.child_frame_id = TToROSConverter<std::string, FString>::Convert(TempoTransform.ToFrame).c_str();
		return ROSTransform;
	}
};

template <>
struct TImplicitFromROSConverter<FStampedTransform> : TFromROSConverter<geometry_msgs::msg::TransformStamped, FStampedTransform>
{
	static ToType Convert(const FromType& ROSTransform)
	{
		return FStampedTransform(TFromROSConverter<builtin_interfaces::msg::Time, double>::Convert(ROSTransform.header.stamp),
			TFromROSConverter<std::string, FString>::Convert(ROSTransform.header.frame_id.c_str()),
			TFromROSConverter<std::string, FString>::Convert(ROSTransform.child_frame_id.c_str()),
			TImplicitFromROSConverter<FTransform>::Convert(ROSTransform.transform));
	}
};

struct FTempoStaticTFPublisher
{
	FTempoStaticTFPublisher(const std::shared_ptr<rclcpp::Node>& Node)
		: Broadcaster(Node, tf2_ros::StaticBroadcasterQoS(), TempoROSPublisherOptions()) {}

	void PublishTransform(const FStampedTransform& StampedTransform)
	{
		Broadcaster.sendTransform(TImplicitToROSConverter<FStampedTransform>::Convert(StampedTransform));
	}

private:
	tf2_ros::StaticTransformBroadcaster Broadcaster;
};

struct FTempoDynamicTFPublisher
{
	FTempoDynamicTFPublisher(const std::shared_ptr<rclcpp::Node>& Node)
		: Broadcaster(Node, tf2_ros::DynamicBroadcasterQoS(), TempoROSPublisherOptions()) {}

	void PublishTransform(const FStampedTransform& StampedTransform)
	{
		Broadcaster.sendTransform(TImplicitToROSConverter<FStampedTransform>::Convert(StampedTransform));
	}

private:
	tf2_ros::TransformBroadcaster Broadcaster;
};

struct FTempoTFListener
{
	static rclcpp::SubscriptionOptions SubOptions()
	{
		rclcpp::SubscriptionOptions Options = TempoROSSubscriptionOptions(GetPolymorphicUnrealAllocator());
		Options.qos_overriding_options = rclcpp::QosOverridingOptions{
			rclcpp::QosPolicyKind::Depth,
			rclcpp::QosPolicyKind::Durability,
			rclcpp::QosPolicyKind::History,
			rclcpp::QosPolicyKind::Reliability};
		return Options;
	}

	static rclcpp::SubscriptionOptions StaticSubOptions()
	{
		rclcpp::SubscriptionOptions Options = TempoROSSubscriptionOptions(GetPolymorphicUnrealAllocator());
		Options.qos_overriding_options = rclcpp::QosOverridingOptions{
			rclcpp::QosPolicyKind::Depth,
			rclcpp::QosPolicyKind::History,
			rclcpp::QosPolicyKind::Reliability};
		return Options;
	}
	
	FTempoTFListener(const std::shared_ptr<rclcpp::Node>& Node)
		: Listener(Buffer, Node, false,
			tf2_ros::DynamicListenerQoS(), tf2_ros::DynamicListenerQoS(),
			SubOptions(), StaticSubOptions()) {}

	bool GetTransform(const FString& FromFrame, const FString& ToFrame, const double Timestamp, FTransform& TransformOut) const
	{
		if (!Buffer.canTransform(TToROSConverter<std::string, FString>::Convert(FromFrame),
			TToROSConverter<std::string, FString>::Convert(ToFrame), tf2::timeFromSec(Timestamp), nullptr))
		{
			UE_LOG(LogTempoROS, Warning, TEXT("Unable to lookup transform from %s to %s at %f."), *FromFrame, *ToFrame, Timestamp);
			return false;
		}
		TransformOut = TImplicitFromROSConverter<FTransform>::Convert(Buffer.lookupTransform(TToROSConverter<std::string, FString>::Convert(FromFrame),
			TToROSConverter<std::string, FString>::Convert(ToFrame),
			tf2::timeFromSec(Timestamp)).transform);
		return true;
	}

private:
	tf2_ros::TransformListener Listener;
	tf2::BufferCore Buffer;
};
