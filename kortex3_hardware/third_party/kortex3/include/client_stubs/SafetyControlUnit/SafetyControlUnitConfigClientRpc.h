#ifndef __SAFETYCONTROLUNITCONFIGCLIENT_H__
#define __SAFETYCONTROLUNITCONFIGCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "SafetyControlUnitConfig.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace SafetyControlUnitConfig
{
	enum FunctionUids
	{
			eUidSetArmCalibration = 0x300001,
			eUidGetMpuSafetyParametersChecksum = 0x300002,
			eUidValidateSafetyParametersChecksum = 0x300003,
			eUidSafetyParametersChecksumChangeTopic = 0x300004,
			eUidUnsubscribe = 0x300005,
	};
	
	class SafetyControlUnitConfigClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdSafetyControlUnitConfig;
	public:
		SafetyControlUnitConfigClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif