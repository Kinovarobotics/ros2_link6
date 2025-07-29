#ifndef __DEVICEMANAGERCLIENT_H__
#define __DEVICEMANAGERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "DeviceManager.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace DeviceManager
{
	enum FunctionUids
	{
			eUidReadAllDevices = 0x170001,
	};
	
	class DeviceManagerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdDeviceManager;
	public:
		DeviceManagerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		DeviceHandles ReadAllDevices(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<DeviceHandles> ReadAllDevices_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void ReadAllDevices_callback(std::function<void(const Error&, const DeviceHandles&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif