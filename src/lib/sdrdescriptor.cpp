/* C wrapper over lime::SDRDescriptor: read-only capability accessors. */
#include "limesuiteng/sdrdescriptor.h"
#include "private.h"

extern "C" {

const char* lime_descriptor_name(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->name.c_str() : nullptr;
}

uint64_t lime_descriptor_serial(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->serialNumber : 0;
}

size_t lime_descriptor_rfsoc_count(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->rfSOC.size() : 0;
}

const char* lime_descriptor_rfsoc_name(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return nullptr;
    return desc(d)->rfSOC[soc].name.c_str();
}

uint32_t lime_descriptor_channel_count(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return 0;
    return desc(d)->rfSOC[soc].channelCount;
}

size_t lime_descriptor_antenna_count(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr)
{
    if (d == nullptr)
        return 0;
    const std::vector<std::string>* names = antennaPaths(desc(d), soc, dr);
    return names != nullptr ? names->size() : 0;
}

const char* lime_descriptor_antenna_name(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr, size_t index)
{
    if (d == nullptr)
        return nullptr;
    const std::vector<std::string>* names = antennaPaths(desc(d), soc, dr);
    return (names != nullptr && index < names->size()) ? (*names)[index].c_str() : nullptr;
}

} /* extern "C" */
