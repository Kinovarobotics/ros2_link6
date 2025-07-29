#ifndef KINOVAUTILAPI_H
#define KINOVAUTILAPI_H

#include "Frame.pb.h"

namespace Kinova
{
namespace Api
{

    /**
     * Unique identifier for any request used to trace back the response
     * Tuple containing: serviceId, functionId, messageId, sessionId, clientId
     */
    using ResponseIdentifier =
        std::tuple<
            uint32_t, // service Id
            uint32_t, // function Id
            uint32_t, // message Id
            uint32_t, // session Id
            std::string // client Id
    >;

    /**
     * Frame information bitfield union
     * Contains error codes, device ID, frame type, and header version
     */
    typedef union
    {
        uint32_t frame_info;
        struct
        {
            uint32_t errorSubCode: 12;
            uint32_t errorCode: 4;
            uint32_t deviceId: 8;

            uint32_t frameType: 4;
            uint32_t headerVersion : 4;
        };
    } FrameInfo;

    /**
     * Message information bitfield union
     * Contains message ID and session ID
     */
    typedef union
    {
        uint32_t message_info;
        struct
        {
            uint32_t messageId: 16;
            uint32_t sessionId: 16;
        };
    } MessageInfo;

    /**
     * Service information bitfield union
     * Contains function ID, service ID, service version, and function UID
     */
    typedef union
    {
        uint32_t service_info;
        struct
        {
            uint32_t functionId: 16;
            uint32_t serviceId: 12;
            uint32_t serviceVersion: 4;
        };

        struct
        {
            uint32_t functionUid: 28;
        };
    } ServiceInfo;

    /**
     * Payload information bitfield union
     * Contains payload length and reserved bits
     */
    typedef union
    {
        uint32_t payload_info;
        struct
        {
            uint32_t payloadLength: 24;
            uint32_t reserved: 8;
        };
    } PayloadInfo;

    /**
     * Header information container for Kortex API frames
     * Encapsulates frame, message, service, and payload information
     * for communication between client and server
     */
    class HeaderInfo
    {
    public:
        // Default constructor
        HeaderInfo();

        /**
         * Construct from protobuf Header
         *
         * @param[in] headerProto Protobuf header to parse
         */
        HeaderInfo(const Header& headerProto);

        /**
         * Construct with individual frame and message fields
         *
         * @param[in] errorCode Error code value
         * @param[in] errorSubCode Error sub-code value
         * @param[in] deviceId Device identifier
         * @param[in] frameType Type of frame
         * @param[in] headerVersion Version of header format
         * @param[in] messageId Message identifier
         * @param[in] sessionId Session identifier
         * @param[in] functionId Function identifier
         * @param[in] serviceId Service identifier
         * @param[in] serviceVersion Service version
         * @param[in] payloadLength Length of payload
         * @param[in] reserved Reserved bits
         */
        HeaderInfo(uint32_t errorCode, uint32_t errorSubCode, uint32_t deviceId, uint32_t frameType, uint32_t headerVersion, uint32_t messageId, uint32_t sessionId, uint32_t functionId, uint32_t serviceId, uint32_t serviceVersion, uint32_t payloadLength, uint32_t reserved);

        /**
         * Construct with function unique ID variant
         *
         * @param[in] errorCode Error code value
         * @param[in] errorSubCode Error sub-code value
         * @param[in] deviceId Device identifier
         * @param[in] frameType Type of frame
         * @param[in] headerVersion Version of header format
         * @param[in] messageId Message identifier
         * @param[in] sessionId Session identifier
         * @param[in] functionUniqueId Unique function identifier (28 bits)
         * @param[in] serviceVersion Service version
         * @param[in] payloadLength Length of payload
         * @param[in] reserved Reserved bits
         */
        HeaderInfo(uint32_t errorCode, uint32_t errorSubCode, uint32_t deviceId, uint32_t frameType, uint32_t headerVersion, uint32_t messageId, uint32_t sessionId, uint32_t functionUniqueId, uint32_t serviceVersion, uint32_t payloadLength, uint32_t reserved);

        /**
         * Value-equality operator
         *
         * @param[in] other Comparison target
         * @return True if <*this> and <other> are value-equivalent
         *         False otherwise
         */
        bool operator == (HeaderInfo const& other) const;

        /**
         * Create a protobuf Header from this HeaderInfo
         *
         * @return Protobuf Header object
         */
        Header createHeader();

        /**
         * Fill an existing protobuf Header with this HeaderInfo
         *
         * @param[out] pHeader Pointer to Header to fill
         */
        void fillHeader(Header* pHeader);

        // Public member variables for direct access to header components
        // These are intentionally public as HeaderInfo is a data structure
        // used for efficient manipulation of frame headers
        FrameInfo        m_frameInfo;      // Frame information (errors, device, type)
        MessageInfo      m_messageInfo;    // Message information (message ID, session ID)
        ServiceInfo      m_serviceInfo;    // Service information (function, service IDs)
        PayloadInfo      m_payloadInfo;    // Payload information (length, reserved)
    };


} // namespace Api
} // namespace Kinova

#endif
