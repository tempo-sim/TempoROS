// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

// #include "TempoROSConversion.generated.h"

template <typename T>
struct TConverter
{
	template <typename X = T>
	static typename X::ToType Convert(const typename X::FromType& FromValue)
	{
		static_assert(std::is_same_v<typename X::ToType, typename X::FromType>);
		return FromValue;
	}
};

template <typename T>
struct TToROSConverter : TConverter<TToROSConverter<T>>
{
	using FromType = T;
	using ToType = T;
};

template <typename T>
struct TFromROSConverter : TConverter<TToROSConverter<T>>
{
	using FromType = T;
	using ToType = T;
};

template <typename MessageType>
struct TMessageTypeTraits
{
	static inline FName MessageTypeDescriptor = FName(NAME_None);
};

// USTRUCT(BlueprintType)
// struct FTempoROSMessage
// {
// 	GENERATED_BODY()
// 	virtual ~FTempoROSMessage() = default;
// 	virtual FName GetType() const { return FName(NAME_Name); }
// };
//
// template <typename MessageType>
// struct TTempoROSMessageWrapper : FTempoROSMessage
// {
// 	virtual FName GetType() const override { return TMessageTypeTraits<MessageType>::MessageTypeDescriptor; }
// };

// Credit: https://vector-of-bool.github.io/2021/10/22/string-templates.html
// template<unsigned Length>
// struct FixedString 
// {
// 	char Buf[Length + 1]{};
// 	constexpr FixedString(char const* s) 
// 	{
// 		for (unsigned i = 0; i != Length; ++i) Buf[i] = s[i];
// 	}
// 	constexpr operator char const*() const { return Buf; }
// };
// template<unsigned Length> FixedString(char const (&)[Length]) -> FixedString<Length - 1>;
//
// template <FixedString>
// struct NamedType { using type = void; };

#define DECLARE_TEMPOROS_NAMED_TYPE(MessageType) \
	template <> struct NamedType<#MessageType> { using Type = MessageType; };

#define DEFINE_TEMOPROS_BP_EVENT(MessageType) \
	DECLARE_DYNAMIC_DELEGATE_OneParam(FTempoROSMessageReceived, MessageType, Value);

#define DEFINE_TEMPOROS_BP_METHODS(MessageType) \
	UFUNCTION(BlueprintCallable) \
	static bool PublishMessageType(UTempoROSNode* Node, const FString& Topic, const MessageType& Message) \
	{ \
		return Node->Publish<MessageType>(Topic, Message); \
	} \
	UFUNCTION(BlueprintCallable) \
	static bool Add##MessageTypePublisher(UTempoROSNode* Node, const FString& Topic) \
	{ \
		return Node->AddPublisher<MessageType>(Topic); \
	} \
	UFUNCTION(BlueprintCallable) \
	static bool Add##MessageTypeSubscription(UTempoROSNode* Node, const FString& Topic, const FTempoROS##MessageType##Received& TempoROSMessageReceivedEvent) \
	{ \
		return Node->AddSubscription<MessageType>(Topic, TROSSubscriptionDelegate<MessageType>::CreateLambda([TempoROSMessageReceivedEvent](const MessageType& Value) \
		{ \
			TempoROSMessageReceivedEvent.ExecuteIfBound(Value); \
		})); \
	}

#define DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(MessageType) \
	template <> inline FName TMessageTypeTraits<MessageType>::MessageTypeDescriptor = FName(#MessageType);
