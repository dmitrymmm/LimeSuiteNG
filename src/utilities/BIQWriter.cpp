#include "BIQWriter.h"

#include <algorithm>
#include <cstring>
#include <limits>

namespace lime {

namespace {

constexpr uint16_t headerBytes = 64;
/// The 16 bit samples are stored so that full scale decodes to 1.0, which is a shift of the
/// block exponent by the number of fractional bits an int16 full scale represents.
constexpr int fullScaleShift = 15;

void PutU8(uint8_t* p, uint8_t v)
{
    p[0] = v;
}

void PutU16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void PutU32(uint8_t* p, uint32_t v)
{
    for (int i = 0; i < 4; ++i)
        p[i] = static_cast<uint8_t>(v >> (8 * i));
}

void PutU64(uint8_t* p, uint64_t v)
{
    for (int i = 0; i < 8; ++i)
        p[i] = static_cast<uint8_t>(v >> (8 * i));
}

void PutF64(uint8_t* p, double v)
{
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v), "double is not 64 bits");
    std::memcpy(&bits, &v, sizeof(bits));
    PutU64(p, bits);
}

/// @brief Smallest exponent that keeps every sample of the block inside the mantissa range.
/// @param peak Largest absolute value of any I or Q component in the block.
/// @param mantissaMax Largest positive mantissa, 2^(bits-1) - 1.
/// @return The exponent, which is negative for blocks well below full scale.
int ChooseExponent(int32_t peak, int32_t mantissaMax)
{
    // The reference calls this the ceil rule: the smallest e with mantissaMax * 2^e >= peak.
    // Searching integers avoids depending on how the platform rounds log2, which the format
    // notes is what byte for byte agreement with the reference vectors hinges on.
    for (int e = -31; e < 31; ++e)
    {
        const bool fits =
            e >= 0 ? (static_cast<int64_t>(mantissaMax) << e) >= peak : mantissaMax >= (static_cast<int64_t>(peak) << -e);
        if (fits)
            return e;
    }
    return 31;
}

/// @brief Scales one component down by the exponent, rounding halves away from zero.
int32_t Quantize(int32_t value, int exponent)
{
    if (exponent <= 0)
        return value << -exponent;

    const int32_t half = 1 << (exponent - 1);
    if (value >= 0)
        return (value + half) >> exponent;
    return -((-value + half) >> exponent);
}

/// @brief Packs mantissas into a plane, two's complement, LSB first, b bits each.
/// @param plane Destination, must hold blockSize * bits / 8 bytes.
void PackPlane(uint8_t* plane, const int32_t* mantissas, uint32_t count, uint8_t bits)
{
    const uint32_t mask = (1u << bits) - 1;
    uint64_t accumulator = 0;
    int held = 0;
    std::size_t out = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        accumulator |= static_cast<uint64_t>(static_cast<uint32_t>(mantissas[i]) & mask) << held;
        held += bits;
        while (held >= 8)
        {
            plane[out++] = static_cast<uint8_t>(accumulator);
            accumulator >>= 8;
            held -= 8;
        }
    }
    // blockSize is a multiple of 8, so blockSize * bits is always a whole number of bytes
    // and nothing can be left over here.
}

} // namespace

BIQWriter::BIQWriter()
{
}

BIQWriter::~BIQWriter()
{
    Close();
}

OpStatus BIQWriter::Open(const std::string& filename, const Config& config, const std::string& metadata)
{
    if (IsOpen())
        Close();

    if (config.mantissaBits < 2 || config.mantissaBits > 16)
        return OpStatus::InvalidValue;
    if (config.blockSize == 0 || config.blockSize % 8 != 0)
        return OpStatus::InvalidValue;

    mConfig = config;
    const uint32_t planeBytes = static_cast<uint32_t>(config.blockSize) * config.mantissaBits / 8;
    mBlockBytes = 1 + 2 * planeBytes;
    mBlock.assign(mBlockBytes, 0);
    mPending.clear();
    mPending.reserve(config.blockSize);
    mDataBytes = 0;

    // the metadata is padded with spaces so the block stream starts on an 8 byte boundary
    std::string meta = metadata;
    while (meta.size() % 8 != 0)
        meta.push_back(' ');
    if (meta.size() > std::numeric_limits<uint32_t>::max())
        return OpStatus::InvalidValue;

    mFile.open(filename, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
    if (!mFile.is_open())
        return OpStatus::IOFailure;

    uint8_t header[headerBytes] = { 0 };
    header[0] = 'B';
    header[1] = 'I';
    header[2] = 'Q';
    header[3] = '1';
    PutU16(header + 4, headerBytes);
    PutU8(header + 6, 0); // complex valued, the remaining flag bits are reserved and zero
    PutU8(header + 7, 1); // channels, reserved and fixed at 1 in this version
    PutU8(header + 8, config.mantissaBits);
    PutU8(header + 9, 0); // fill rule, 0 is the ceil rule that ChooseExponent implements
    PutU16(header + 10, config.blockSize);
    PutU32(header + 12, mBlockBytes); // no padding between blocks
    PutF64(header + 16, config.sampleRate_Hz);
    PutF64(header + 24, config.centerFrequency_Hz);
    PutU64(header + 32, static_cast<uint64_t>(config.startTime_unix_ns));
    PutF64(header + 40, config.refFullScale_dBm);
    PutU64(header + 48, 0); // payload length, filled in by Close()
    PutU32(header + 56, static_cast<uint32_t>(meta.size()));
    // bytes 60 to 63 are reserved and stay zero

    mFile.write(reinterpret_cast<const char*>(header), sizeof(header));
    if (!meta.empty())
        mFile.write(meta.data(), meta.size());

    return mFile.good() ? OpStatus::Success : OpStatus::IOFailure;
}

OpStatus BIQWriter::FlushBlock()
{
    const uint32_t count = mConfig.blockSize;
    const uint8_t bits = mConfig.mantissaBits;
    const int32_t mantissaMax = (1 << (bits - 1)) - 1;
    const uint32_t planeBytes = count * bits / 8;

    int32_t peak = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        const int32_t re = std::abs(static_cast<int32_t>(mPending[i].real()));
        const int32_t im = std::abs(static_cast<int32_t>(mPending[i].imag()));
        peak = std::max(peak, std::max(re, im));
    }

    std::fill(mBlock.begin(), mBlock.end(), static_cast<uint8_t>(0));

    if (peak == 0)
    {
        // -128 is the sentinel for a block that is entirely zero, the planes stay zeroed
        mBlock[0] = static_cast<uint8_t>(0x80);
    }
    else
    {
        const int exponent = ChooseExponent(peak, mantissaMax);

        std::vector<int32_t> mantissaI(count);
        std::vector<int32_t> mantissaQ(count);
        for (uint32_t i = 0; i < count; ++i)
        {
            // the clamp only comes into play when rounding pushes the peak one step over
            mantissaI[i] = std::clamp(Quantize(mPending[i].real(), exponent), -mantissaMax - 1, mantissaMax);
            mantissaQ[i] = std::clamp(Quantize(mPending[i].imag(), exponent), -mantissaMax - 1, mantissaMax);
        }

        mBlock[0] = static_cast<uint8_t>(static_cast<int8_t>(exponent - fullScaleShift));
        PackPlane(mBlock.data() + 1, mantissaI.data(), count, bits);
        PackPlane(mBlock.data() + 1 + planeBytes, mantissaQ.data(), count, bits);
    }

    mFile.write(reinterpret_cast<const char*>(mBlock.data()), mBlock.size());
    mDataBytes += mBlock.size();
    mPending.clear();
    return mFile.good() ? OpStatus::Success : OpStatus::IOFailure;
}

OpStatus BIQWriter::Write(const complex16_t* samples, uint32_t count)
{
    if (!IsOpen())
        return OpStatus::IOFailure;

    for (uint32_t i = 0; i < count; ++i)
    {
        mPending.push_back(samples[i]);
        if (mPending.size() == mConfig.blockSize)
        {
            const OpStatus status = FlushBlock();
            if (status != OpStatus::Success)
                return status;
        }
    }
    return OpStatus::Success;
}

OpStatus BIQWriter::Close()
{
    if (!IsOpen())
        return OpStatus::Success;

    OpStatus status = OpStatus::Success;
    if (!mPending.empty())
    {
        // a block is a fixed size record, so the tail has to be padded out
        mPending.resize(mConfig.blockSize, complex16_t{ 0, 0 });
        status = FlushBlock();
    }

    // record the payload length now that it is known. Leaving it at 0 also means read to
    // end of file, so a stream that cannot seek back still produces a readable capture.
    mFile.flush();
    if (mFile.good())
    {
        mFile.seekp(48);
        if (mFile.good())
        {
            uint8_t field[8];
            PutU64(field, mDataBytes);
            mFile.write(reinterpret_cast<const char*>(field), sizeof(field));
        }
        mFile.clear();
    }

    mFile.close();
    mPending.clear();
    return status;
}

} // namespace lime
