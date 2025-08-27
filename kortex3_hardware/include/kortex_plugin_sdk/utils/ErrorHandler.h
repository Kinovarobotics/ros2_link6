#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>

#include <string>
#include <list>


namespace Kinova
{
namespace Api
{

namespace Utilities
{

class CustomErrorHandler : public nlohmann::json_schema::basic_error_handler
{
    public:

        // Constructor
        CustomErrorHandler() = default;

        // Destructor
        ~CustomErrorHandler() noexcept = default;
        
        struct err_informations
        {
            nlohmann::json::json_pointer m_ptr;
            nlohmann::json m_json_instance;
            std::string m_error_message;
            std::string m_json_error_message;
        };
        
        std::list<err_informations> m_list_err_informations;

        void error(const nlohmann::json::json_pointer &ptr, const nlohmann::json &instance, const std::string &message) override;
        
        std::list<err_informations> get_err_informations_list(){return m_list_err_informations;};
};

}
}
}

#endif//ERROR_HANDLER_H