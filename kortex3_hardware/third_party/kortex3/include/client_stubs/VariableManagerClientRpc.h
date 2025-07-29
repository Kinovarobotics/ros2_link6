#ifndef __VARIABLEMANAGERCLIENT_H__
#define __VARIABLEMANAGERCLIENT_H__

#include "ClientService.h"
#include "FrameTranslator.h"
#include "VariableManager.pb.h"

#include <functional>
#include <string>
#include <future>

namespace Kinova
{
namespace Api
{
namespace VariableManager
{
	enum FunctionUids
	{
			eUidSetVariable = 0x280001,
			eUidGetVariable = 0x280002,
			eUidDeleteVariable = 0x280003,
			eUidGetAllVariables = 0x280004,
			eUidGetAllNamespaces = 0x280005,
			eUidDeleteNamespace = 0x280006,
			eUidGetAllVariableJsonSchemas = 0x280007,
			eUidVariableChangeTopic = 0x280008,
			eUidUnsubscribe = 0x280009,
			eUidInterpolate = 0x28000a,
			eUidVariableJsonSchemasChangedTopic = 0x28000b,
			eUidCreateNamespace = 0x28000c,
	};
	
	class VariableManagerClient : public Kinova::Api::ClientService
	{
		static const uint32_t m_serviceVersion = 1;
		static const uint32_t m_serviceId = eIdVariableManager;
	public:
		VariableManagerClient(IRouterClient* clientRouter, const std::string& ns = "");
		static uint32_t getUniqueFctId(uint16_t fctId);

		VariableHandle SetVariable(const Variable& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<VariableHandle> SetVariable_async(const Variable& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void SetVariable_callback(const Variable& message, std::function<void(const Error&, const VariableHandle&)> callback, uint32_t deviceId = 0);

		Variable GetVariable(const VariableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Variable> GetVariable_async(const VariableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetVariable_callback(const VariableHandle& message, std::function<void(const Error&, const Variable&)> callback, uint32_t deviceId = 0);

		void DeleteVariable(const VariableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteVariable_async(const VariableHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteVariable_callback(const VariableHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		NamespaceVariables GetAllVariables(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<NamespaceVariables> GetAllVariables_async(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllVariables_callback(const NamespaceHandle& message, std::function<void(const Error&, const NamespaceVariables&)> callback, uint32_t deviceId = 0);

		Namespaces GetAllNamespaces(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<Namespaces> GetAllNamespaces_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllNamespaces_callback(std::function<void(const Error&, const Namespaces&)> callback, uint32_t deviceId = 0);

		void DeleteNamespace(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> DeleteNamespace_async(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void DeleteNamespace_callback(const NamespaceHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);

		VariableJsonSchemaList GetAllVariableJsonSchemas(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<VariableJsonSchemaList> GetAllVariableJsonSchemas_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void GetAllVariableJsonSchemas_callback(std::function<void(const Error&, const VariableJsonSchemaList&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationVariableChangeTopic(std::function<void(ConfigurationChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void Unsubscribe(const Kinova::Api::Common::NotificationHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		InterpolateString Interpolate(const InterpolateString& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<InterpolateString> Interpolate_async(const InterpolateString& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void Interpolate_callback(const InterpolateString& message, std::function<void(const Error&, const InterpolateString&)> callback, uint32_t deviceId = 0);

		Kinova::Api::Common::NotificationHandle OnNotificationVariableJsonSchemasChangedTopic(std::function<void(VariableJsonSchemasChangeNotification)> callback, const Kinova::Api::Common::NotificationOptions& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

		void CreateNamespace(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		std::future<void> CreateNamespace_async(const NamespaceHandle& message, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
		void CreateNamespace_callback(const NamespaceHandle& message, std::function<void(const Error&)> callback, uint32_t deviceId = 0);


	private:
		void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
	};
}
}
}
#endif