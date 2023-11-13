#ifndef INCLUDED_UMTRX_COMMON_HPP
#define INCLUDED_UMTRX_COMMON_HPP

#include <uhd/version.hpp>

#if UHD_VERSION >= 4000000
#define UMTRX_UHD_PTR_NAMESPACE std
#else
#define UMTRX_UHD_PTR_NAMESPACE boost
#endif

#endif /* INCLUDED_UMTRX_COMMON_HPP */
