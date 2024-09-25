// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

template <typename MessageType>
struct TMessageTypeTraits
{
	static FName MessageTypeDescriptor;
};

template <typename MessageType>
FName TMessageTypeTraits<MessageType>::MessageTypeDescriptor = FName(NAME_None);

#define DEFINE_TEMPOROS_MESSAGE_TYPE_TRAITS(MessageType) \
template <> inline FName TMessageTypeTraits<MessageType>::MessageTypeDescriptor = FName(#MessageType);

template <typename T>
struct TConverter
{
	// From kennytm's answer here: https://stackoverflow.com/questions/8113878/c-crtp-and-accessing-deriveds-nested-typedefs-from-base
	template <typename X = T>
	using ToType = typename X::ToType;

	template <typename X = T>
	using FromType = typename X::FromType;

	template <typename X = T>
	static typename X::ToType Convert(const typename X::FromType& FromValue)
	{
		static_assert(std::is_same_v<typename X::ToType, typename X::FromType>, "No specialization found to convert these types");
		return FromValue;
	}
};

template <typename ROSType, typename TempoType>
struct TToROSConverter : TConverter<TToROSConverter<ROSType, TempoType>>
{
	using ToType = ROSType;
	using FromType = TempoType;
};

template <typename T>
struct TImplicitToROSConverter : TToROSConverter<T, T>
{

};

template <typename ROSType, typename TempoType>
struct TFromROSConverter : TConverter<TFromROSConverter<ROSType, TempoType>>
{
	using ToType = TempoType;
	using FromType = ROSType;
};

template <typename T>
struct TImplicitFromROSConverter : TFromROSConverter<T, T>
{

};
