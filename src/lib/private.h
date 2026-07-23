/* Internal helpers shared by the C API wrapper files. Not installed. */
#ifndef LIMESUITENG_CAPI_LIB_PRIVATE_H
#define LIMESUITENG_CAPI_LIB_PRIVATE_H

#include "limesuiteng/types.h"
#include "limesuiteng/sdrdevice.h"
#include "limesuiteng/rfstream.h"

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/RFStream.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/SDRDevice.hpp"
#include "limesuiteng/types.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

/* The C API casts status/direction/format straight to and from their C++
 * counterparts, so the enumerators must keep the same underlying values.
 * These checks fail the build if either side is ever reordered or renumbered. */
static_assert(static_cast<int>(lime::OpStatus::Success) == lime_OpStatus_Success, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::Error) == lime_OpStatus_Error, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::NotImplemented) == lime_OpStatus_NotImplemented, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::IOFailure) == lime_OpStatus_IOFailure, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::InvalidValue) == lime_OpStatus_InvalidValue, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::FileNotFound) == lime_OpStatus_FileNotFound, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::OutOfRange) == lime_OpStatus_OutOfRange, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::NotSupported) == lime_OpStatus_NotSupported, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::Timeout) == lime_OpStatus_Timeout, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::Busy) == lime_OpStatus_Busy, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::Aborted) == lime_OpStatus_Aborted, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::PermissionDenied) == lime_OpStatus_PermissionDenied, "OpStatus drift");
static_assert(static_cast<int>(lime::OpStatus::NotConnected) == lime_OpStatus_NotConnected, "OpStatus drift");

static_assert(static_cast<int>(lime::TRXDir::Rx) == lime_TRXDir_Rx, "TRXDir drift");
static_assert(static_cast<int>(lime::TRXDir::Tx) == lime_TRXDir_Tx, "TRXDir drift");

static_assert(static_cast<int>(lime::DataFormat::I16) == lime_DataFormat_I16, "DataFormat drift");
static_assert(static_cast<int>(lime::DataFormat::I12) == lime_DataFormat_I12, "DataFormat drift");
static_assert(static_cast<int>(lime::DataFormat::F32) == lime_DataFormat_F32, "DataFormat drift");

/* A C++ exception must never cross the extern "C" boundary. Every entry point
 * that calls into the library wraps its body in try { ... } and ends with one
 * of these, translating any exception to the function's error return. */
#define LIME_CATCH(retval) \
    catch (...) \
    { \
        return retval; \
    }
#define LIME_CATCH_VOID \
    catch (...) \
    { \
    }

/* Transitional: the generic device tree collapses onto SDRDevice, so the
 * device, SDR, GPIO, and SPI handles are all the same underlying pointer. */
static inline lime::SDRDevice* sdr(lime_SDRDevice* d)
{
    return reinterpret_cast<lime::SDRDevice*>(d);
}

static inline lime::TRXDir dir(lime_TRXDir d)
{
    return static_cast<lime::TRXDir>(d != lime_TRXDir_Rx);
}

static inline const lime::SDRDescriptor* desc(const lime_SDRDescriptor* d)
{
    return reinterpret_cast<const lime::SDRDescriptor*>(d);
}

/* The public C surface takes 32-bit indices so the ABI never has to widen;
 * the internal C++ addressing is currently 8-bit, so out-of-range values are
 * rejected rather than silently truncated. */
static inline bool narrows(uint32_t module, uint32_t channel)
{
    return module > std::numeric_limits<uint8_t>::max() || channel > std::numeric_limits<uint8_t>::max();
}

/* The antenna path names for a SoC and direction, or nullptr if the SoC index
 * is out of range or that direction exposes no paths. Shared by the
 * sdrdevice and sdrdescriptor wrappers. */
static inline const std::vector<std::string>* antennaPaths(const lime::SDRDescriptor* d, size_t soc, lime_TRXDir dr)
{
    if (soc >= d->rfSOC.size())
        return nullptr;
    const auto& paths = d->rfSOC[soc].pathNames;
    const auto it = paths.find(dir(dr));
    return it != paths.end() ? &it->second : nullptr;
}

/* The stream handle owns the RFStream and remembers the application sample
 * format so recv/send can dispatch the matching typed overload. */
struct lime_Stream {
    std::unique_ptr<lime::RFStream> impl;
    lime::DataFormat format;
};

#endif /* LIMESUITENG_CAPI_LIB_PRIVATE_H */
