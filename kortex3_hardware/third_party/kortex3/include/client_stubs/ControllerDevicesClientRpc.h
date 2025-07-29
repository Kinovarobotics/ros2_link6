#ifndef __CONTROLLERDEVICESCLIENT_H__
#define __CONTROLLERDEVICESCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "ControllerDevices.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace ControllerDevices
{
	enum FunctionUids
	{
			eUidGetStorageList = 0x320001,
			eUidGetFileSystemList = 0x320002,
			eUidGetLocalUpdateFileList = 0x320003,
			eUidFileWrite = 0x320004,
			eUidFileRead = 0x320005,
			eUidStorageTopic = 0x320006,
			eUidUnsubscribe = 0x320007,
			eUidGetConnectedDevicesList = 0x320008,
	};
	
	class ControllerDevicesClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdControllerDevices;
	public:
		ControllerDevicesClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif