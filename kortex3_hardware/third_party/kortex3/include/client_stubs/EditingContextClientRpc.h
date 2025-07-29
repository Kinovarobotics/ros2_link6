#ifndef __EDITINGCONTEXTCLIENT_H__
#define __EDITINGCONTEXTCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "EditingContext.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace EditingContext
{
	enum FunctionUids
	{
			eUidSetEditingContext = 0x2d0001,
			eUidUnsetEditingContext = 0x2d0002,
	};
	
	class EditingContextClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdEditingContext;
	public:
		EditingContextClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		void SetEditingContext(const Context& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> SetEditingContext_async(const Context& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetEditingContext_callback(const Context& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		void UnsetEditingContext(const ContextSessionIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> UnsetEditingContext_async(const ContextSessionIdentifier& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void UnsetEditingContext_callback(const ContextSessionIdentifier& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif