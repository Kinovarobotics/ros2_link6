#ifndef __PLUGINCLIENT_H__
#define __PLUGINCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "Plugin.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace Plugin
{
	enum FunctionUids
	{
			eUidGetStatus = 0x1f0001,
			eUidSetUp = 0x1f0002,
			eUidTearDown = 0x1f0003,
			eUidStart = 0x1f0004,
			eUidStop = 0x1f0005,
			eUidGetMetaData = 0x1f0006,
			eUidGetConfiguration = 0x1f0007,
			eUidGetConfigurationSchema = 0x1f0008,
			eUidSetConfiguration = 0x1f0009,
			eUidResetConfiguration = 0x1f000a,
			eUidGetActionTypes = 0x1f000b,
			eUidStartAction = 0x1f000d,
			eUidPauseAction = 0x1f000e,
			eUidResumeAction = 0x1f000f,
			eUidStopAction = 0x1f0010,
			eUidStateTopic = 0x1f0011,
			eUidConfigurationChangeTopic = 0x1f0012,
			eUidActionTopic = 0x1f0013,
			eUidUnsubscribe = 0x1f0014,
			eUidGetVisualizers = 0x1f0016,
			eUidSetUpWithConfiguration = 0x1f0017,
			eUidTriggerUpdate = 0x1f0018,
			eUidSetUpAction = 0x1f001a,
			eUidTearDownAction = 0x1f001b,
			eUidInfoChangeTopic = 0x1f001c,
			eUidGetFeedback = 0x1f001d,
			eUidGetFeedbackSchema = 0x1f001e,
			eUidUpdateAction = 0x1f001f,
			eUidValidateAction = 0x1f0020,
			eUidMigrateAction = 0x1f0021,
	};
	
	class PluginClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdPlugin;
	public:
		PluginClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		Status GetStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Status> GetStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetStatus_callback(std::function<void(const Error&, const Status&)> callback, uint32_t deviceId = 0);

		MetaData GetMetaData(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<MetaData> GetMetaData_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetMetaData_callback(std::function<void(const Error&, const MetaData&)> callback, uint32_t deviceId = 0);

		Configuration GetConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Configuration> GetConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetConfiguration_callback(std::function<void(const Error&, const Configuration&)> callback, uint32_t deviceId = 0);

		ConfigurationSchema GetConfigurationSchema(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ConfigurationSchema> GetConfigurationSchema_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetConfigurationSchema_callback(std::function<void(const Error&, const ConfigurationSchema&)> callback, uint32_t deviceId = 0);

		void SetConfiguration(const Configuration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetConfiguration_async(const Configuration& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetConfiguration_callback(const Configuration& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ResetConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ResetConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResetConfiguration_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ActionDescriptionList GetActionTypes(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ActionDescriptionList> GetActionTypes_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetActionTypes_callback(std::function<void(const Error&, const ActionDescriptionList&)> callback, uint32_t deviceId = 0);

		ActionInstanceHandle StartAction(const Action& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ActionInstanceHandle> StartAction_async(const Action& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void StartAction_callback(const Action& message, std::function<void(const Error&, const ActionInstanceHandle&)> callback, uint32_t deviceId = 0);

		void PauseAction(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> PauseAction_async(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void PauseAction_callback(const ActionInstanceHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ResumeAction(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ResumeAction_async(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ResumeAction_callback(const ActionInstanceHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void StopAction(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> StopAction_async(const ActionInstanceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void StopAction_callback(const ActionInstanceHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationStateTopic(std::function<void(StateNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationConfigurationChangeTopic(std::function<void(ConfigurationChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationActionTopic(std::function<void(ActionNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		VisualizerList GetVisualizers(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<VisualizerList> GetVisualizers_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetVisualizers_callback(std::function<void(const Error&, const VisualizerList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationInfoChangeTopic(std::function<void(InfoChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Feedback GetFeedback(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Feedback> GetFeedback_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetFeedback_callback(std::function<void(const Error&, const Feedback&)> callback, uint32_t deviceId = 0);

		FeedbackSchema GetFeedbackSchema(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<FeedbackSchema> GetFeedbackSchema_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetFeedbackSchema_callback(std::function<void(const Error&, const FeedbackSchema&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif