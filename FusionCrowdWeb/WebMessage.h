#pragma once

#include "FcWebApi.h"


namespace FusionCrowdWeb
{
	/** Enum for request types. */
	enum class FC_WEB_API RequestCode : unsigned char
	{
		Undefined = 0,	/**< Unset value. */
		InitSimulation,	/**< Initialization request. */ 
		DoStep			/**< Simulation step request. */ 
	};


	/** Enum for response types. */
	enum class FC_WEB_API ResponseCode : unsigned char
	{
		Undefined = 0,	/**< Unset value. */
		Success,		/**< Response that the operation succeeded. */ 
		Error			/**< Response that the operation failed. */ 
	};


	/**
	* \union WebCode
	* \brief Union for request and response types.
	*
	* @see RequestCode, ResponseCode
	*/
	union FC_WEB_API WebCode
	{
		/** WebCode as RequestCode. */
		RequestCode AsRequestCode = RequestCode::Undefined;

		/** WebCode as ResponseCode. */
		ResponseCode AsResponseCode;

		WebCode() {}
		WebCode(RequestCode inRequestCode) : AsRequestCode(inRequestCode) {}
		WebCode(ResponseCode inResponseCode) : AsResponseCode(inResponseCode) {}
	};


	/** Network data header. */
	struct FC_WEB_API WebMessageHead
	{
		/** Message type. */
		WebCode WebCode;

		/** Message length in bytes. */
		size_t MessageLength = 0;
	};


	/** Network message. */
	struct FC_WEB_API WebMessage
	{
		/** Message type. */
		WebCode WebCode;

		/** Pointer to raw data. */
		const char* Data = nullptr;
	};
}
