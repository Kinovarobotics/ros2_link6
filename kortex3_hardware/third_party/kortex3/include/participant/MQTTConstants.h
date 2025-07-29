#ifndef _KORTEXAPI_MQTTCONSTANTS_H_
#define _KORTEXAPI_MQTTCONSTANTS_H_

namespace Kinova
{
    namespace Api
    {
        // Raw topics for update/kill session
        static constexpr const char* SESSION_KILLED_RAW_TOPIC = "sessions/killed";
        static constexpr const char* SESSION_STATUS_RAW_TOPIC = "sessions/status";
    } // namespace Api
} // namespace Kinova

#endif //_KORTEXAPI_MQTTCONSTANTS_H_