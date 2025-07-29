#ifndef __TOOLPLUGINCLIENT_H__
#define __TOOLPLUGINCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ToolPlugin.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ToolPlugin
{
	enum FunctionUids
	{
			eUidJog = 0x2a0001,
			eUidGetToolInformation = 0x2a0002,
			eUidGetToolsInformation = 0x2a0003,
			eUidGetToolFeedback = 0x2a0004,
			eUidToolInformationTopic = 0x2a0005,
			eUidUnsubscribe = 0x2a0006,
			eUidMoveRelative = 0x2a0007,
	};
	
	class ToolPluginClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdToolPlugin;
	public:
		ToolPluginClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void Jog(const JogMessage& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> Jog_async(const JogMessage& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Jog_callback(const JogMessage& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ToolInformation GetToolInformation(const ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ToolInformation> GetToolInformation_async(const ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolInformation_callback(const ToolHandle& message, std::function<void(const Error&, const ToolInformation&)> callback, uint32_t deviceId = 0);

		ToolInformationList GetToolsInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ToolInformationList> GetToolsInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolsInformation_callback(std::function<void(const Error&, const ToolInformationList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::CustomData GetToolFeedback(const ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::Common::CustomData> GetToolFeedback_async(const ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolFeedback_callback(const ToolHandle& message, std::function<void(const Error&, const Kinova::Api::Common::CustomData&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationToolInformationTopic(std::function<void(ToolInformationNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void MoveRelative(const MoveMessage& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> MoveRelative_async(const MoveMessage& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void MoveRelative_callback(const MoveMessage& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif