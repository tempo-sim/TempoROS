// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "std_msgs/msg/String.hpp"

template <>
struct TImplicitToROSConverter<FString> : TToROSConverter<std_msgs::msg::String, FString>
{
	static ToType Convert(const FromType& FromValue)
	{
		std_msgs::msg::String ROSString;
		ROSString.data = TCHAR_TO_UTF8(*FromValue);
		return ROSString;
	}
};

template <>
struct TImplicitFromROSConverter<FString> : TFromROSConverter<std_msgs::msg::String, FString>
{
	static ToType Convert(const FromType& FromValue)
	{
		return FString(UTF8_TO_TCHAR(FromValue.data.c_str()));
	}
};

DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FString)
