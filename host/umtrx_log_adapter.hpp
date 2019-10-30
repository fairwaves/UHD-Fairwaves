#ifndef UMTRX_LOG_ADAPTER_HPP
#define UMTRX_LOG_ADAPTER_HPP

#ifdef UHD_HAS_MSG_HPP
#include <uhd/utils/msg.hpp>

#define UHD_LOG_FASTPATH(message) UHD_MSG(fastpath) << (message)

#else // UHD_HAS_MSG_HPP
#include <uhd/utils/log.hpp>

enum {
    _uhd_log_level_status,
    _uhd_log_level_warning,
    _uhd_log_level_error
};

#define UHD_MSG(severity) ((_uhd_log_level_##severity==_uhd_log_level_status)?UHD_LOGGER_INFO("UmTRX"): \
                           (_uhd_log_level_##severity==_uhd_log_level_warning)?UHD_LOGGER_WARNING("UmTRX"): \
                           UHD_LOGGER_ERROR("UmTRX"))

#endif // UHD_HAS_MSG_HPP

#endif // UMTRX_LOG_ADAPTER_HPP
