#ifndef __INDUSTRIALIOCLIENT_H__
#define __INDUSTRIALIOCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "IndustrialIO.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace IndustrialIO
{
	enum FunctionUids
	{
			eUidSetDigitalOutputConfiguration = 0x240001,
			eUidGetDigitalOutputConfiguration = 0x240002,
			eUidSetDigitalOutputHighState = 0x240003,
			eUidSetDigitalOutputLowState = 0x240004,
			eUidGetDigitalOutputInfo = 0x240005,
			eUidGetAllDigitalOutputInfo = 0x240006,
			eUidSetDigitalInputConfiguration = 0x240007,
			eUidGetDigitalInputConfiguration = 0x240008,
			eUidGetDigitalInputInfo = 0x240009,
			eUidGetAllDigitalInputInfo = 0x24000a,
			eUidSetAnalogIOConfiguration = 0x24000b,
			eUidGetAnalogIOConfiguration = 0x24000c,
			eUidSetAnalogValue = 0x24000d,
			eUidGetAnalogIOInfo = 0x24000e,
			eUidGetAllAnalogIOInfo = 0x24000f,
			eUidClearFaults = 0x240010,
			eUidUnsubscribe = 0x240011,
			eUidDigitalOutputChangeTopic = 0x240012,
			eUidDigitalInputChangeTopic = 0x240013,
			eUidAnalogIOChangeTopic = 0x240014,
			eUidWristAnalogIOChangeTopic = 0x240015,
	};
	
	class IndustrialIOClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdIndustrialIO;
	public:
		IndustrialIOClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetDigitalOutputConfiguration(const DigitalOutputConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalOutputConfiguration_async(const DigitalOutputConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalOutputConfiguration_callback(const DigitalOutputConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DigitalOutputConfiguration GetDigitalOutputConfiguration(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalOutputConfiguration> GetDigitalOutputConfiguration_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalOutputConfiguration_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&, const DigitalOutputConfiguration&)> callback, uint32_t deviceId = 0);

		void SetDigitalOutputHighState(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalOutputHighState_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalOutputHighState_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void SetDigitalOutputLowState(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalOutputLowState_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalOutputLowState_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DigitalOutputInfo GetDigitalOutputInfo(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalOutputInfo> GetDigitalOutputInfo_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalOutputInfo_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&, const DigitalOutputInfo&)> callback, uint32_t deviceId = 0);

		DigitalOutputInfoList GetAllDigitalOutputInfo(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalOutputInfoList> GetAllDigitalOutputInfo_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllDigitalOutputInfo_callback(std::function<void(const Error&, const DigitalOutputInfoList&)> callback, uint32_t deviceId = 0);

		void SetDigitalInputConfiguration(const DigitalInputConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetDigitalInputConfiguration_async(const DigitalInputConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetDigitalInputConfiguration_callback(const DigitalInputConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		DigitalInputConfiguration GetDigitalInputConfiguration(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalInputConfiguration> GetDigitalInputConfiguration_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalInputConfiguration_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&, const DigitalInputConfiguration&)> callback, uint32_t deviceId = 0);

		DigitalInputInfo GetDigitalInputInfo(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalInputInfo> GetDigitalInputInfo_async(const DigitalChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetDigitalInputInfo_callback(const DigitalChannelIdentifier& message, std::function<void(const Error&, const DigitalInputInfo&)> callback, uint32_t deviceId = 0);

		DigitalInputInfoList GetAllDigitalInputInfo(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DigitalInputInfoList> GetAllDigitalInputInfo_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllDigitalInputInfo_callback(std::function<void(const Error&, const DigitalInputInfoList&)> callback, uint32_t deviceId = 0);

		void SetAnalogIOConfiguration(const AnalogIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAnalogIOConfiguration_async(const AnalogIOConfiguration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAnalogIOConfiguration_callback(const AnalogIOConfiguration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		AnalogIOConfiguration GetAnalogIOConfiguration(const AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<AnalogIOConfiguration> GetAnalogIOConfiguration_async(const AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAnalogIOConfiguration_callback(const AnalogIOChannelIdentifier& message, std::function<void(const Error&, const AnalogIOConfiguration&)> callback, uint32_t deviceId = 0);

		void SetAnalogValue(const AnalogOutput& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetAnalogValue_async(const AnalogOutput& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetAnalogValue_callback(const AnalogOutput& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		AnalogIOInfo GetAnalogIOInfo(const AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<AnalogIOInfo> GetAnalogIOInfo_async(const AnalogIOChannelIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAnalogIOInfo_callback(const AnalogIOChannelIdentifier& message, std::function<void(const Error&, const AnalogIOInfo&)> callback, uint32_t deviceId = 0);

		AnalogIOInfoList GetAllAnalogIOInfo(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<AnalogIOInfoList> GetAllAnalogIOInfo_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllAnalogIOInfo_callback(std::function<void(const Error&, const AnalogIOInfoList&)> callback, uint32_t deviceId = 0);

		void ClearFaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ClearFaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ClearFaults_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		Kinova::Api::Common::NotificationHandle OnNotificationDigitalOutputChangeTopic(std::function<void(DigitalOutputChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationDigitalInputChangeTopic(std::function<void(DigitalInputChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationAnalogIOChangeTopic(std::function<void(AnalogIOChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationWristAnalogIOChangeTopic(std::function<void(AnalogIOChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif