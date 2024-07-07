// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "std_msgs/msg/String.hpp"
#include "geometry_msgs/msg/transform.hpp"

DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FString)
DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FTransform)

template <>
struct TImplicitToROSConverter<FString> : TToROSConverter<std_msgs::msg::String, FString> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& FromValue)
	{
		std_msgs::msg::String ROSString;
		ROSString.data = TCHAR_TO_UTF8(*FromValue);
		return ROSString;
	}
};

template <>
struct TImplicitFromROSConverter<FString> : TFromROSConverter<std_msgs::msg::String, FString> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& FromValue)
	{
		return FString(UTF8_TO_TCHAR(FromValue.data.c_str()));
	}
};

template <>
struct TImplicitToROSConverter<FTransform> : TToROSConverter<geometry_msgs::msg::Transform, FTransform> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& TempoTransform)
	{
		geometry_msgs::msg::Transform ROSTransform;
		ROSTransform.translation.x = TempoTransform.GetLocation().X / 100.0;
		ROSTransform.translation.y = TempoTransform.GetLocation().Y / 100.0;
		ROSTransform.translation.z = TempoTransform.GetLocation().Z / 100.0;
		return ROSTransform;
	}
};

template <>
struct TImplicitFromROSConverter<FTransform> : TFromROSConverter<geometry_msgs::msg::Transform, FTransform> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& ROSTransform)
	{
		FTransform TempoTransform;
		TempoTransform.SetTranslation(FVector(
			100.0 * ROSTransform.translation.x,
			100.0 * ROSTransform.translation.y,
			100.0 * ROSTransform.translation.z));
		return TempoTransform;
	}
};

// FRotator
// Vector