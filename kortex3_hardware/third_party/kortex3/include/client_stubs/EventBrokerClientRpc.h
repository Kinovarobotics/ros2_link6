#ifndef __EVENTBROKERCLIENT_H__
#define __EVENTBROKERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "EventBroker.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace EventBroker
{
	enum FunctionUids
	{
			eUidLogEvent = 0x310001,
			eUidLogEvents = 0x310002,
			eUidGetEvents = 0x310003,
			eUidClearEvent = 0x310004,
			eUidClearAllEvents = 0x310005,
			eUidEventTopic = 0x310006,
			eUidUnsubscribe = 0x310007,
	};
	
	class EventBrokerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdEventBroker;
	public:
		EventBrokerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void LogEvent(const Event& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> LogEvent_async(const Event& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void LogEvent_callback(const Event& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void LogEvents(const EventList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> LogEvents_async(const EventList& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void LogEvents_callback(const EventList& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		EventList GetEvents(const GetEventsParameter& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<EventList> GetEvents_async(const GetEventsParameter& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetEvents_callback(const GetEventsParameter& message, std::function<void(const Error&, const EventList&)> callback, uint32_t deviceId = 0);

		void ClearEvent(const EventHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ClearEvent_async(const EventHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ClearEvent_callback(const EventHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void ClearAllEvents(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> ClearAllEvents_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ClearAllEvents_callback(std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationEventTopic(std::function<void(EventNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif