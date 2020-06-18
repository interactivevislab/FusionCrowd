#pragma once

#include "FcWebApi.h"

#include <stdexcept>
#include <string.h>


namespace FusionCrowdWeb
{
	class FC_WEB_API FcWebException
	{
	public:

		FcWebException() noexcept;
		FcWebException(char const* const inMessage) noexcept;
		FcWebException(FcWebException const& inOther) noexcept;

		virtual ~FcWebException() noexcept;

		FcWebException& operator=(FcWebException const& inOther) noexcept;

		virtual char const* What() const;

	private:
		char* _message = nullptr;
	};


	class FC_WEB_API WsException : public FcWebException
	{
	public:
		WsException(const char* inMessage);
		int GetWsErrorCode() const;

	private:
		int _errorCode;
	};
}
