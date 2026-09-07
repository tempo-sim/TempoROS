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

// Base class for all converters. Parameterized directly on the To/From types so that the default
// (identity) Convert can be declared without ever needing the derived type. Specializations provide
// their own static Convert, which hides this default.
template <typename ToT, typename FromT>
struct TConverter
{
	using ToType = ToT;
	using FromType = FromT;

	static ToType Convert(const FromType& FromValue)
	{
		static_assert(std::is_same_v<ToType, FromType>, "No specialization found to convert these types");
		return FromValue;
	}
};

template <typename ROSType, typename TempoType>
struct TToROSConverter : TConverter<ROSType, TempoType>
{
};

template <typename T>
struct TImplicitToROSConverter : TToROSConverter<T, T>
{

};

template <typename ROSType, typename TempoType>
struct TFromROSConverter : TConverter<TempoType, ROSType>
{
};

template <typename T>
struct TImplicitFromROSConverter : TFromROSConverter<T, T>
{

};
