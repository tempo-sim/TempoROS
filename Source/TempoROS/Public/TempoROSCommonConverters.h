// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"
#include "TempoROSTypes.h"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"

DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FString)
DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FTransform)

template <>
struct TToROSConverter<std::string, FString> : TConverter<TToROSConverter<std::string, FString>>
{
	static std::string Convert(const FString& FromValue)
	{
		return TCHAR_TO_UTF8(*FromValue);
	}
};

template <>
struct TFromROSConverter<std::string, FString> : TConverter<TFromROSConverter<std::string, FString>>
{
	static FString Convert(const std::string& FromValue)
	{
		return FString(UTF8_TO_TCHAR(FromValue.c_str()));
	}
};

template <>
struct TImplicitToROSConverter<FString> : TToROSConverter<std_msgs::msg::String, FString> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& FromValue)
	{
		std_msgs::msg::String ROSString;
		ROSString.data = TToROSConverter<std::string, FString>::Convert(FromValue);
		return ROSString;
	}
};

template <>
struct TImplicitFromROSConverter<FString> : TFromROSConverter<std_msgs::msg::String, FString> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& FromValue)
	{
		return TFromROSConverter<std::string, FString>::Convert(FromValue.data);
	}
};

template <>
struct TToROSConverter<builtin_interfaces::msg::Time, double> : TConverter<TToROSConverter<builtin_interfaces::msg::Time, double>>
{
	static builtin_interfaces::msg::Time Convert(double FromValue)
	{
		builtin_interfaces::msg::Time ROSValue;
		ROSValue.sec = FromValue;
		ROSValue.nanosec = 1e9 * (FromValue - ROSValue.sec);
		return ROSValue;
	}
};

template <>
struct TFromROSConverter<builtin_interfaces::msg::Time, double> : TConverter<TFromROSConverter<builtin_interfaces::msg::Time, double>>
{
	static double Convert(builtin_interfaces::msg::Time FromValue)
	{
		return FromValue.sec + FromValue.nanosec / 1.e9;
	}
};

template <>
struct TImplicitToROSConverter<FVector> : TToROSConverter<geometry_msgs::msg::Vector3, FVector> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& TempoVector)
	{
		ToType ROSVector;
		ROSVector.x = 0.01 * TempoVector.X;
		ROSVector.y = -0.01 * TempoVector.Y;
		ROSVector.z = 0.01 * TempoVector.Z;
		return ROSVector;
	}
};

template <>
struct TImplicitFromROSConverter<FVector> : TFromROSConverter<geometry_msgs::msg::Vector3, FVector> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& ROSVector)
	{
		return ToType(100.0 * ROSVector.x, -100.0 * ROSVector.y, 100.0 * ROSVector.z);
	}
};

template <>
struct TToROSConverter<geometry_msgs::msg::Quaternion, FQuat> : TConverter<TToROSConverter<geometry_msgs::msg::Quaternion, FQuat>>
{
	static geometry_msgs::msg::Quaternion Convert(const FQuat& TempoValue)
	{
		geometry_msgs::msg::Quaternion ROSRotation;
		ROSRotation.w = TempoValue.W;
		ROSRotation.x = -TempoValue.X;
		ROSRotation.y = TempoValue.Y;
		ROSRotation.z = -TempoValue.Z;
		return ROSRotation;
	}
};

template <>
struct TFromROSConverter<geometry_msgs::msg::Quaternion, FQuat> : TConverter<TFromROSConverter<geometry_msgs::msg::Quaternion, FQuat>>
{
	static FQuat Convert(const geometry_msgs::msg::Quaternion& ROSValue)
	{
		return FQuat(-ROSValue.x, ROSValue.y, -ROSValue.z, ROSValue.w);
	}
};

template <>
struct TImplicitToROSConverter<FRotator> : TToROSConverter<geometry_msgs::msg::Quaternion, FRotator> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& TempoRotation)
	{
		return TToROSConverter<ToType, FQuat>::Convert(TempoRotation.Quaternion());
	}
};

template <>
struct TImplicitFromROSConverter<FRotator> : TFromROSConverter<geometry_msgs::msg::Quaternion, FRotator> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& ROSRotation)
	{
		return FRotator(TFromROSConverter<geometry_msgs::msg::Quaternion, FQuat>::Convert(ROSRotation));
	}
};

template <>
struct TImplicitToROSConverter<FTransform> : TToROSConverter<geometry_msgs::msg::Transform, FTransform> // TempoROS__BPSupport
{
	static geometry_msgs::msg::Transform Convert(const FTransform& TempoTransform)
	{
		geometry_msgs::msg::Transform ROSTransform;
		ROSTransform.translation = TImplicitToROSConverter<FVector>::Convert(TempoTransform.GetLocation());
		ROSTransform.rotation = TToROSConverter<geometry_msgs::msg::Quaternion, FQuat>::Convert(TempoTransform.GetRotation());
		return ROSTransform;
	}
};

template <>
struct TImplicitFromROSConverter<FTransform> : TFromROSConverter<geometry_msgs::msg::Transform, FTransform> // TempoROS__BPSupport
{
	static FTransform Convert(const geometry_msgs::msg::Transform& ROSTransform)
	{
		return FTransform(TFromROSConverter<geometry_msgs::msg::Quaternion, FQuat>::Convert(ROSTransform.rotation),
			TImplicitFromROSConverter<FVector>::Convert(ROSTransform.translation));
	}
};

template <>
struct TImplicitFromROSConverter<FTwist> : TFromROSConverter<geometry_msgs::msg::Twist, FTwist> // TempoROS__BPSupport
{
	static ToType Convert(const FromType& FromValue)
	{
		FTwist TempoValue;
		TempoValue.LinearVelocity = TImplicitFromROSConverter<FVector>::Convert(FromValue.linear);
		TempoValue.AngularVelocity = FMath::RadiansToDegrees(FVector(-FromValue.angular.x, FromValue.angular.y, -FromValue.angular.z));
		return TempoValue;
	}
};
