#ifndef __SESSIONCLIENT_H__
#define __SESSIONCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "Session.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace Session
{
	enum FunctionUids
	{
			eUidCreateSession = 0x10001,
			eUidCloseSession = 0x10002,
			eUidKeepAlive = 0x10003,
			eUidGetConnections = 0x10004,
			eUidAddTemporaryUserRole = 0x10005,
			eUidRemoveTemporaryUserRole = 0x10006,
			eUidSetAccessControl = 0x10007,
			eUidGetAccessControl = 0x10008,
	};
	
	class SessionClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdSession;
	public:
		SessionClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void CreateSession(const CreateSessionInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> CreateSession_async(const CreateSessionInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateSession_callback(const CreateSessionInfo& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void CloseSession(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> CloseSession_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CloseSession_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void KeepAlive(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> KeepAlive_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void KeepAlive_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ConnectionList GetConnections(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ConnectionList> GetConnections_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetConnections_callback(std::function<void(const Error&, const ConnectionList&)> callback, uint32_t deviceId = 0);

		void AddTemporaryUserRole(const TemporaryRoleRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> AddTemporaryUserRole_async(const TemporaryRoleRequest& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void AddTemporaryUserRole_callback(const TemporaryRoleRequest& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void RemoveTemporaryUserRole(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> RemoveTemporaryUserRole_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void RemoveTemporaryUserRole_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif