#ifndef IROUTER_CLIENT_SERVER
#define IROUTER_CLIENT_SERVER

#include "IRouterClient.h"
#include "IRouterServer.h"

namespace Kinova
{
namespace Api
{

    class IRouterClientServer :
        public IRouterClient, 
        public IRouterServer
    {};

} // namespace  Api
} // namespace Kinova

#endif //IROUTER_CLIENT_SERVER