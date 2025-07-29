/* ***************************************************************************
 * Kinova inc.
 * Project : 
 *
 * Copyright (c) 2006-2017 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __ITRANSPORTSERVER_H__
#define __ITRANSPORTSERVER_H__

#include <functional>
#include <string>
#include <vector>

enum eConnectionType
{
	kUnknown = 0,
	kUdp,
	kTcp,
};

struct ClientConnectionHandle
{
	uint32_t                        clientId;
	uint32_t                        currentConnectionId;
	eConnectionType                 currentConnectionType;
	std::string                     connectionInfo;     // e.g.: "type=UDP;ip=127.0.0.1;port=56536"
	std::vector<eConnectionType>    connectionTypeList;
	
	bool operator==(const ClientConnectionHandle &rhs) const
	{
		return clientId == rhs.clientId;
	}
	
	bool operator!=(const ClientConnectionHandle &rhs) const
	{
		return !operator==(rhs);
	}

	explicit ClientConnectionHandle(uint32_t id = 0) : clientId(id)
	{

    }
};


enum defaultConnectionSettings
{
	kDefaultMaxBufSize    			= 65507,
	kDefaultRXPollTimeOutUs			= 50,
	kDefaultTXPollTimeOutUs			= 50,
	kApiPort						= 10000,
};

enum RTConnectionSettings
{
        kApiPort_RT                 = 10001
};


class ITransportServer
{
public:

    virtual ~ITransportServer() {}

	virtual bool    Send(const ClientConnectionHandle& connectionHandle, char* TXBuffer, int txLen) = 0;
	virtual void    OnMessage(std::function<void(const ClientConnectionHandle&, uint8_t*,int)>) = 0;
	
	virtual void    Process() = 0;
	virtual char*   GetTxBuffer(size_t* pTxLen) = 0;
	virtual bool    CloseConnection(ClientConnectionHandle const& handle) = 0;
    virtual void    RegisterOnConnectionClosedCallback(std::function<void(ClientConnectionHandle)> onCloseCallback) = 0;
};

#endif // __ITRANSPORTSERVER_H__
