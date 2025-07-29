#ifndef _KORTEXAPI_FRAMETRANSLATOR_H_
#define _KORTEXAPI_FRAMETRANSLATOR_H_

#include "Frame.pb.h"

namespace Kinova{
namespace Api{

/**
 * Class used to translate a Kinova::Api::Frame into any other message type
 * This makes use of a templated casting operator to perform compile-time casting
 * operations into the message type
 */
class FrameTranslator
{
private:
    // Stored frame message 
    Kinova::Api::Frame mFrame;

public:

    /**
     * Constructor
     *
     * @param[in] frame Frame to store
     */
    explicit FrameTranslator(Kinova::Api::Frame frame):
        mFrame{frame}
    {}

    /**
     * Implicit Cast operation into another message type
     *
     * @tparam MessageType Message type to cast into
     * @pre <MessageType> must be a derived class of
     *      google::protbuf::Message
     * @return A message of type <MessageType> parsed from 
     *         the payload of the frame
     */
    template<typename MessageType>
    operator MessageType() const
    {
        MessageType ret;
        if(!ret.ParseFromString(mFrame.payload()))
        {
            // Parsing error - silently ignored
        }
        return ret;
    }
};

}
}

#endif