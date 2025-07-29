#ifndef KINOVAEXCEPTION_H
#define KINOVAEXCEPTION_H

#include <string>
#include <sstream>
#include <exception>

namespace Kinova
{
namespace Api
{

    /**
     * Basic exception class for Kortex API
     *
     * KBasicException is the base exception class used by the Kortex API.
     * It extends std::runtime_error to provide consistent exception handling
     * across the API. For exceptions with detailed error information (error codes),
     * use KDetailedException instead.
     *
     * @note This class can be caught as std::runtime_error or std::exception
     */
    class KBasicException : public std::runtime_error
    {
    public:
        /**
         * Constructor with message string
         *
         * @param[in] msgStr Error message describing the exception
         */
        KBasicException(const std::string& msgStr);

        /**
         * Copy constructor
         *
         * @param[in] other Another KBasicException to copy from
         */
        KBasicException(const KBasicException &other);

        /**
         * Get the exception message
         *
         * @return C-string containing the error message
         */
        virtual const char* what() const throw() override;

        /**
         * Get a string representation of the exception
         *
         * @return String containing the error message
         */
        virtual std::string toString();
    };

    /**
     * Type alias for programming exceptions
     * Used for exceptions caused by incorrect API usage or programming errors
     */
    typedef KBasicException KProgException;

} // namespace Api
} // namespace Kinova

#endif
