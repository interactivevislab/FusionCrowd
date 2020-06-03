#pragma once

#include "FcWebApi.h"


namespace FusionCrowdWeb
{
	enum FC_WEB_API RequestCode : unsigned char
	{
		InitSimulation,
		DoStep
	};

	enum FC_WEB_API ResponseCode : unsigned char
	{
		Success,
		Error
	};

	union FC_WEB_API WebCode
	{
		RequestCode AsRequestCode;
		ResponseCode AsResponseCode;

		WebCode() {}
		WebCode(RequestCode inRequestCode) : AsRequestCode(inRequestCode) {}
		WebCode(ResponseCode inResponseCode) : AsResponseCode(inResponseCode) {}
	};

	struct FC_WEB_API WebMessageHead
	{
		WebCode WebCode;
		size_t MessageLength;
	};
}
