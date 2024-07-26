// Copyright Tempo Simulation, LLC. All Rights Reserved

#pragma once

#include "TempoROSConversion.h"

#include "rclcpp.h"

template <class ServiceType>
using TROSServiceDelegate = TDelegate<typename ServiceType::Response(const typename ServiceType::Request&)>;

struct FTempoROSService
{
	virtual ~FTempoROSService() = default;
};

template <typename ServiceType>
struct TEMPOROS_API TTempoROSService : FTempoROSService
{
	using RequestType = typename ServiceType::Request;
	using ResponseType = typename ServiceType::Response;
	
	using ROSServiceType = typename TImplicitToROSConverter<ServiceType>::ToType;
	using ROSRequestType = typename ROSServiceType::Request;
	using ROSResponseType = typename ROSServiceType::Response;
	
	TTempoROSService(const std::shared_ptr<rclcpp::Node>& Node, const FString& ServiceName, const TROSServiceDelegate<ServiceType>& Callback)
		: Service(Node->create_service<ROSServiceType>(TCHAR_TO_UTF8(*ServiceName), [Callback](const std::shared_ptr<typename ROSServiceType::Request> Request, std::shared_ptr<typename ROSServiceType::Response> Response)
		{
			if (Callback.IsBound())
			{
				*Response.get() = TToROSConverter<ROSResponseType, ResponseType>::Convert(Callback.Execute(TFromROSConverter<ROSRequestType, RequestType>::Convert(*Request.get())));
			}
		})) {}
	
private:
	std::shared_ptr<rclcpp::Service<ROSServiceType>> Service;
};
