#ifndef __PROTECTIONZONECLIENT_H__
#define __PROTECTIONZONECLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ProtectionZone.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ProtectionZone
{
	enum FunctionUids
	{
			eUidCreateProtectionZone = 0x2e0001,
			eUidUpdateProtectionZone = 0x2e0002,
			eUidReadProtectionZone = 0x2e0003,
			eUidDeleteProtectionZone = 0x2e0004,
			eUidReadAllProtectionZones = 0x2e0005,
			eUidSetToolSphere = 0x2e0006,
			eUidGetToolSphere = 0x2e0007,
			eUidProtectionZoneChangeTopic = 0x2e0008,
			eUidToolSphereChangeTopic = 0x2e0009,
			eUidUnsubscribe = 0x2e000a,
	};
	
	class ProtectionZoneClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdProtectionZone;
	public:
		ProtectionZoneClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		ProtectionZoneHandle CreateProtectionZone(const ProtectionZoneConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ProtectionZoneHandle> CreateProtectionZone_async(const ProtectionZoneConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateProtectionZone_callback(const ProtectionZoneConfig& message, std::function<void(const Error&, const ProtectionZoneHandle&)> callback, uint32_t deviceId = 0);

		void UpdateProtectionZone(const ProtectionZoneConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> UpdateProtectionZone_async(const ProtectionZoneConfig& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void UpdateProtectionZone_callback(const ProtectionZoneConfig& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ProtectionZoneConfig ReadProtectionZone(const ProtectionZoneHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ProtectionZoneConfig> ReadProtectionZone_async(const ProtectionZoneHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadProtectionZone_callback(const ProtectionZoneHandle& message, std::function<void(const Error&, const ProtectionZoneConfig&)> callback, uint32_t deviceId = 0);

		void DeleteProtectionZone(const ProtectionZoneHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteProtectionZone_async(const ProtectionZoneHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteProtectionZone_callback(const ProtectionZoneHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ProtectionZoneConfigList ReadAllProtectionZones(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ProtectionZoneConfigList> ReadAllProtectionZones_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllProtectionZones_callback(std::function<void(const Error&, const ProtectionZoneConfigList&)> callback, uint32_t deviceId = 0);

		void SetToolSphere(const ToolSphere& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetToolSphere_async(const ToolSphere& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetToolSphere_callback(const ToolSphere& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		ToolSphere GetToolSphere(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<ToolSphere> GetToolSphere_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetToolSphere_callback(std::function<void(const Error&, const ToolSphere&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationProtectionZoneChangeTopic(std::function<void(ProtectionZoneChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		Kinova::Api::Common::NotificationHandle OnNotificationToolSphereChangeTopic(std::function<void(ToolSphereChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif