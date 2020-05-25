#include "WsException.h"

#include <WinSock2.h>


namespace FusionCrowdWeb
{
	FcWebException::FcWebException() noexcept
	{
	}

	FcWebException::FcWebException(char const* const _Message) noexcept
	{
		auto len = strlen(_Message) + 1;
		_message = new char[len];
		strcpy_s(_message, len, _Message);
	}

	FcWebException::FcWebException(FcWebException const& _Other) noexcept
	{
		auto len = strlen(_Other._message) + 1;
		_message = new char[len];
		strcpy_s(_message, len, _Other._message);
	}

	FcWebException& FcWebException::operator=(FcWebException const& _Other) noexcept
	{
		if (this == &_Other)
		{
			return *this;
		}

		if (_message)
		{
			delete _message;
		}
		auto len = strlen(_Other._message) + 1;
		_message = new char[len];
		strcpy_s(_message, len, _Other._message);

		return *this;
	}

	FcWebException::~FcWebException() noexcept
	{
		if (_message)
		{
			delete _message;
		}
	}

	char const* FcWebException::What() const
	{
		return _message ? _message : "Unknown exception";
	}


	WsException::WsException(const char* inMessage) : FcWebException(inMessage)
	{
		_errorCode = WSAGetLastError();
	}

	int WsException::GetWsErrorCode() const
	{
		return _errorCode;
	}
}
