#pragma once

#include "FcWebApi.h"


namespace FusionCrowdWeb
{
	/**
	* \class FcWebException
	* \brief Main exception class for web part of FusionCrowd.
	*/
	class FC_WEB_API FcWebException
	{
	public:

		FcWebException() noexcept;
		FcWebException(char const* const inMessage) noexcept;
		FcWebException(FcWebException const& inOther) noexcept;

		virtual ~FcWebException() noexcept;

		FcWebException& operator=(FcWebException const& inOther) noexcept;

		/** Returns error message. */
		virtual char const* What() const;

	private:
		/** Error message. */
		char* _message = nullptr;
	};

	/**
	* \class WsException
	* \brief Exception class for Windows Sockets errors.
	*
	* @see FcWebException
	*/
	class FC_WEB_API WsException : public FcWebException
	{
	public:
		WsException(const char* inMessage);

		/** Returns the error status for failed Windows Sockets operation. */
		int GetWsErrorCode() const;

	private:
		/** Error status for failed Windows Sockets operation. */
		int _errorCode;
	};
}
