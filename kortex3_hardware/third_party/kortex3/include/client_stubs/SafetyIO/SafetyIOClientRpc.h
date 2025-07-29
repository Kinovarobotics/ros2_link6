#ifndef __SAFETYIOCLIENT_H__
#define __SAFETYIOCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "SafetyIO.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace SafetyIO
{
	enum FunctionUids
	{
			eUidSetSafetyIOConfiguration = 0x2f0001,
			eUidSetAllSafetyIOConfiguration = 0x2f0002,
			eUidGetSafetyIOConfiguration = 0x2f0003,
			eUidGetAllSafetyIOConfiguration = 0x2f0004,
			eUidGetSafetyIOStatus = 0x2f0005,
			eUidSafetyIOChangeTopic = 0x2f0006,
			eUidUnsubscribe = 0x2f0007,
	};
	
	class SafetyIOClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdSafetyIO;
	public:
		SafetyIOClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetSafetyIOConfiguration(const SafetyIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetSafetyIOConfiguration_async(const SafetyIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetSafetyIOConfiguration_callback(const SafetyIOConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetAllSafetyIOConfiguration(const SafetyIOConfigurationList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAllSafetyIOConfiguration_async(const SafetyIOConfigurationList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAllSafetyIOConfiguration_callback(const SafetyIOConfigurationList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		SafetyIOConfiguration GetSafetyIOConfiguration(const SafetyIOInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyIOConfiguration> GetSafetyIOConfiguration_async(const SafetyIOInfo& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetyIOConfiguration_callback(const SafetyIOInfo& message, std::function<void(const Error&, const SafetyIOConfiguration&)> callback, uint32_t deviceId = 0);

		SafetyIOConfigurationList GetAllSafetyIOConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyIOConfigurationList> GetAllSafetyIOConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllSafetyIOConfiguration_callback(std::function<void(const Error&, const SafetyIOConfigurationList&)> callback, uint32_t deviceId = 0);

		SafetyIOChannelStatus GetSafetyIOStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<SafetyIOChannelStatus> GetSafetyIOStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetSafetyIOStatus_callback(std::function<void(const Error&, const SafetyIOChannelStatus&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationSafetyIOChangeTopic(std::function<void(SafetyIOChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif