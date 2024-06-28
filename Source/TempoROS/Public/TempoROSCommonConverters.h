// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "std_msgs/msg/String.hpp"

template <>
struct TToROSConverter<FString> : TConverter<TToROSConverter<FString>> // TempoROS__BPSupport
{
	using FromType = FString;
	using ToType = std_msgs::msg::String;
	static ToType Convert(const FromType& FromValue)
	{
		std_msgs::msg::String ROSString;
		ROSString.data = TCHAR_TO_UTF8(*FromValue);
		return ROSString;
	}
};

template <>
struct TFromROSConverter<FString> : TConverter<TFromROSConverter<FString>> // TempoROS__BPSupport
{
	using ToType = FString;
	using FromType = std_msgs::msg::String;
	static ToType Convert(const FromType& FromValue)
	{
		return FString(UTF8_TO_TCHAR(FromValue.data.c_str()));
	}
};

// DECLARE_TEMPOROS_NAMED_TYPE(FString)

// DECLARE_TEMPOROS_NAMED_TYPE(void)

// template <> struct NamedType<"void"> { using Type = void; };

// UCLASS(BlueprintType)
// class TEMPOROS_API UCommonConvertersBPFunctions : public UBlueprintFunctionLibrary
// {
// 	GENERATED_BODY()

	// DECLARE_DYNAMIC_DELEGATE_OneParam(FTempoROSFStringReceived, FString, Value);
	// DEFINE_TEMPOROS_BP_METHODS(FString)

	// UFUNCTION(BlueprintCallable)
	// static bool Publish(UTempoROSNode* Node, const FString& Topic, const FTempoROSMessage& Message)
	// {
	// 	return Node->Publish<FString>(Topic, Message);
	// }
	// UFUNCTION(BlueprintCallable)
	// static bool AddFStringPublisher(UTempoROSNode* Node, const FString& Topic)
	// {
	// 	return Node->AddPublisher<FString>(Topic);
	// }
	// UFUNCTION(BlueprintCallable)
	// static bool AddFStringSubscription(UTempoROSNode* Node, const FString& Topic, const FTempoROSFStringReceived& TempoROSMessageReceivedEvent)
	// {
	// return Node->AddSubscription<FString>(Topic, TROSSubscriptionDelegate<FString>::CreateLambda([TempoROSMessageReceivedEvent](const FString& Value)
	// 	{
	// 		TempoROSMessageReceivedEvent.ExecuteIfBound(Value);
	// 	}));
	// }
// };

DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(FString)
