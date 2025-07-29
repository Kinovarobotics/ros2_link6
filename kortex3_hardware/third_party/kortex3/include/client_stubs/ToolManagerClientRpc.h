#ifndef __TOOLMANAGERCLIENT_H__
#define __TOOLMANAGERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ToolManager.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ToolManager
{
	enum FunctionUids
	{
			eUidGetAllToolsInformation = 0x2b0001,
			eUidGetToolInformation = 0x2b0002,
			eUidGetActiveToolInformationList = 0x2b0003,
			eUidSelectActiveToolList = 0x2b0004,
			eUidCreateCustomTool = 0x2b0005,
			eUidDeleteCustomTool = 0x2b0006,
			eUidUpdateToolInformation = 0x2b0007,
			eUidActiveToolInformationChangedTopic = 0x2b0008,
			eUidToolInformationTopic = 0x2b0009,
			eUidUnsubscribe = 0x2b000a,
	};
	
	class ToolManagerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdToolManager;
	public:
		ToolManagerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		Kinova::Api::ToolPlugin::ToolInformationList GetAllToolsInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ToolPlugin::ToolInformationList> GetAllToolsInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllToolsInformation_callback(std::function<void(const Error&, const Kinova::Api::ToolPlugin::ToolInformationList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ToolPlugin::ToolInformation GetToolInformation(const Kinova::Api::ToolPlugin::ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ToolPlugin::ToolInformation> GetToolInformation_async(const Kinova::Api::ToolPlugin::ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolInformation_callback(const Kinova::Api::ToolPlugin::ToolHandle& message, std::function<void(const Error&, const Kinova::Api::ToolPlugin::ToolInformation&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ToolPlugin::ToolInformationList GetActiveToolInformationList(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ToolPlugin::ToolInformationList> GetActiveToolInformationList_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetActiveToolInformationList_callback(std::function<void(const Error&, const Kinova::Api::ToolPlugin::ToolInformationList&)> callback, uint32_t deviceId = 0);

		void SelectActiveToolList(const Kinova::Api::ToolPlugin::ToolHandleList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SelectActiveToolList_async(const Kinova::Api::ToolPlugin::ToolHandleList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SelectActiveToolList_callback(const Kinova::Api::ToolPlugin::ToolHandleList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::ToolPlugin::ToolHandle CreateCustomTool(const Kinova::Api::ToolPlugin::ToolInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Kinova::Api::ToolPlugin::ToolHandle> CreateCustomTool_async(const Kinova::Api::ToolPlugin::ToolInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateCustomTool_callback(const Kinova::Api::ToolPlugin::ToolInformation& message, std::function<void(const Error&, const Kinova::Api::ToolPlugin::ToolHandle&)> callback, uint32_t deviceId = 0);

		void DeleteCustomTool(const Kinova::Api::ToolPlugin::ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteCustomTool_async(const Kinova::Api::ToolPlugin::ToolHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteCustomTool_callback(const Kinova::Api::ToolPlugin::ToolHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void UpdateToolInformation(const Kinova::Api::ToolPlugin::ToolInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> UpdateToolInformation_async(const Kinova::Api::ToolPlugin::ToolInformation& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void UpdateToolInformation_callback(const Kinova::Api::ToolPlugin::ToolInformation& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationActiveToolInformationChangedTopic(std::function<void(ActiveToolNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationToolInformationTopic(std::function<void(ToolNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif