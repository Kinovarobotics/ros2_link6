#ifndef __BASECYCLICCLIENT_H__
#define __BASECYCLICCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "BaseCyclic.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace BaseCyclic
{
	enum FunctionUids
	{
			eUidRefresh = 0x30001,
			eUidRefreshCommand = 0x30002,
			eUidRefreshFeedback = 0x30003,
			eUidRefreshCustomData = 0x30004,
	};
	
	class BaseCyclicClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdBaseCyclic;
	public:
		BaseCyclicClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		Feedback Refresh(const Command& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Feedback> Refresh_async(const Command& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Refresh_callback(const Command& message, std::function<void(const Error&, const Feedback&)> callback, uint32_t deviceId = 0);

		Feedback RefreshFeedback(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Feedback> RefreshFeedback_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void RefreshFeedback_callback(std::function<void(const Error&, const Feedback&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif