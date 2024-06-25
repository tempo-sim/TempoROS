// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"

template <class ServiceType>
using TROSServiceDelegate = TDelegate<typename ServiceType::Response(const typename ServiceType::Request&)>;

template <typename ServiceType>
struct TEMPOROS_API TTempoROSService
{
	using RequestType = typename ServiceType::Request;
	using ResponseType = typename ServiceType::Response;
	
	using ROSServiceType = typename TToROSConverter<ServiceType>::ToType;
	using ROSRequestType = typename TFromROSConverter<typename ServiceType::Request>::FromType;
	using ROSResponseType = typename TToROSConverter<typename ServiceType::Response>::ToType;

	static_assert(std::is_same_v<ROSRequestType, typename ROSServiceType::Request>);
	static_assert(std::is_same_v<ROSResponseType, typename ROSServiceType::Response>);
	
	TTempoROSService(const std::shared_ptr<rclcpp::Node>& Node, const FString& ServiceName, const TROSServiceDelegate<ServiceType>& Callback)
		: Subscription(Node->create_service<ROSServiceType>(TCHAR_TO_UTF8(*ServiceName), [Callback](const std::shared_ptr<typename ROSServiceType::Request> Request, const std::shared_ptr<typename ROSServiceType::Response> Response)
		{
			if (Callback.IsBound())
			{
				*Response.get() = TToROSConverter<ResponseType>::Convert(Callback.Execute(TFromROSConverter<RequestType>::Convert(*Request.get())));
			}
		})) {}
	
private:
	std::shared_ptr<rclcpp::Service<ROSServiceType>> Subscription;
};
