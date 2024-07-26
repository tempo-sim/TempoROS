// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"
#include "TempoROSTypes.h"
#include "TempoROSCommonConverters.h"

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
		ROSTransform.header.frame_id = TToROSConverter<std::string, FString>::Convert(TempoTransform.FromFrame);
		ROSTransform.child_frame_id = TToROSConverter<std::string, FString>::Convert(TempoTransform.ToFrame);
		return ROSTransform;
	}
};

template <>
struct TImplicitFromROSConverter<FStampedTransform> : TFromROSConverter<geometry_msgs::msg::TransformStamped, FStampedTransform>
{
	static ToType Convert(const FromType& ROSTransform)
	{
		return FStampedTransform(TFromROSConverter<builtin_interfaces::msg::Time, double>::Convert(ROSTransform.header.stamp),
			TFromROSConverter<std::string, FString>::Convert(ROSTransform.header.frame_id),
			TFromROSConverter<std::string, FString>::Convert(ROSTransform.child_frame_id),
			TImplicitFromROSConverter<FTransform>::Convert(ROSTransform.transform));
	}
};

struct FTempoStaticTFPublisher
{
	FTempoStaticTFPublisher(const std::shared_ptr<rclcpp::Node>& Node)
		: Broadcaster(Node) {}

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
		: Broadcaster(Node) {}

	void PublishTransform(const FStampedTransform& StampedTransform)
	{
		Broadcaster.sendTransform(TImplicitToROSConverter<FStampedTransform>::Convert(StampedTransform));
	}
	
private:
	tf2_ros::TransformBroadcaster Broadcaster;
};

struct FTempoTFListener
{
	FTempoTFListener(const std::shared_ptr<rclcpp::Node>& Node)
		: Listener(Buffer, Node, true /*spin_thread*/) {}

	FTransform GetTransform(const FString& FromFrame, const FString& ToFrame, const double Timestamp) const
	{
		std::string ErrorMsg;
		if (!Buffer.canTransform(TToROSConverter<std::string, FString>::Convert(FromFrame),
			TToROSConverter<std::string, FString>::Convert(ToFrame), tf2::timeFromSec(Timestamp), &ErrorMsg))
		{
			UE_LOG(LogTempoROS, Warning, TEXT("Unable to lookup transform from %s to %s at %f: %s"), *FromFrame, *ToFrame, Timestamp, UTF8_TO_TCHAR(ErrorMsg.c_str()));
			return FTransform::Identity;
		}
		return TImplicitFromROSConverter<FTransform>::Convert(Buffer.lookupTransform(TToROSConverter<std::string, FString>::Convert(FromFrame),
			TToROSConverter<std::string, FString>::Convert(ToFrame),
			tf2::timeFromSec(Timestamp)).transform);
	}

private:
	tf2_ros::TransformListener Listener;
	tf2::BufferCore Buffer;
};
