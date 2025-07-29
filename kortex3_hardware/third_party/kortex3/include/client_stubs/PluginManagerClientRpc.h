#ifndef __PLUGINMANAGERCLIENT_H__
#define __PLUGINMANAGERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "PluginManager.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace PluginManager
{
	enum FunctionUids
	{
			eUidInstallPlugin = 0x210001,
			eUidUpdatePlugin = 0x210002,
			eUidUninstallPlugin = 0x210003,
			eUidUninstallAllPlugins = 0x210004,
			eUidLaunchPlugin = 0x210005,
			eUidShutDownPlugin = 0x210006,
			eUidShutDownAllPlugins = 0x210007,
			eUidPluginUpdatedTopic = 0x210008,
			eUidPluginInstallationTopic = 0x210009,
			eUidUnsubscribe = 0x21000a,
			eUidGetPluginInfo = 0x21000b,
			eUidGetPluginsList = 0x21000c,
	};
	
	class PluginManagerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdPluginManager;
	public:
		PluginManagerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void LaunchPlugin(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> LaunchPlugin_async(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void LaunchPlugin_callback(const PluginHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ShutDownPlugin(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ShutDownPlugin_async(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ShutDownPlugin_callback(const PluginHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ShutDownAllPlugins(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ShutDownAllPlugins_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ShutDownAllPlugins_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationPluginUpdatedTopic(std::function<void(PluginUpdatedNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationPluginInstallationTopic(std::function<void(PluginInstallationNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		PluginInfo GetPluginInfo(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PluginInfo> GetPluginInfo_async(const PluginHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPluginInfo_callback(const PluginHandle& message, std::function<void(const Error&, const PluginInfo&)> callback, uint32_t deviceId = 0);

		PluginInfoList GetPluginsList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<PluginInfoList> GetPluginsList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetPluginsList_callback(std::function<void(const Error&, const PluginInfoList&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif