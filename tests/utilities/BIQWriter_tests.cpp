#include <gtest/gtest.h>

#include "utilities/BIQWriter.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace lime;

namespace {

/// Decoder written straight from the format description rather than from the writer, so the
/// two do not share an interpretation of the layout.
struct DecodedFile {
    std::string magic;
    uint16_t headerBytes{ 0 };
    uint8_t flags{ 0 };
    uint8_t channels{ 0 };
    uint8_t mantissaBits{ 0 };
    uint8_t fillRule{ 0 };
    uint16_t blockSize{ 0 };
    uint32_t blockStride{ 0 };
    double sampleRate{ 0 };
    double centerFrequency{ 0 };
    int64_t startTime{ 0 };
    double refFullScale{ 0 };
    uint64_t dataBytes{ 0 };
    uint32_t metaBytes{ 0 };
    uint32_t reserved{ 0 };
    std::string metadata;
    std::vector<int> exponents; ///< one per block, -128 marks an all zero block
    std::vector<double> real; ///< decoded I, in full scale units
    std::vector<double> imag; ///< decoded Q, in full scale units
};

template<class T> T ReadLE(const std::vector<uint8_t>& raw, std::size_t offset)
{
    T value{};
    std::memcpy(&value, raw.data() + offset, sizeof(T));
    return value;
}

/// Pulls mantissa 'index' out of a plane: b bits wide, two's complement, packed LSB first.
int ReadMantissa(const uint8_t* plane, uint32_t index, uint8_t bits)
{
    const uint32_t firstBit = index * bits;
    uint32_t value = 0;
    for (uint8_t k = 0; k < bits; ++k)
    {
        const uint32_t bit = firstBit + k;
        if (plane[bit / 8] & (1u << (bit % 8)))
            value |= 1u << k;
    }
    const uint32_t signBit = 1u << (bits - 1);
    return static_cast<int>((value ^ signBit) - signBit);
}

DecodedFile Decode(const std::string& path)
{
    std::ifstream file(path, std::ifstream::binary);
    std::vector<uint8_t> raw((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    DecodedFile out;
    out.magic.assign(reinterpret_cast<const char*>(raw.data()), 4);
    out.headerBytes = ReadLE<uint16_t>(raw, 4);
    out.flags = raw[6];
    out.channels = raw[7];
    out.mantissaBits = raw[8];
    out.fillRule = raw[9];
    out.blockSize = ReadLE<uint16_t>(raw, 10);
    out.blockStride = ReadLE<uint32_t>(raw, 12);
    out.sampleRate = ReadLE<double>(raw, 16);
    out.centerFrequency = ReadLE<double>(raw, 24);
    out.startTime = ReadLE<int64_t>(raw, 32);
    out.refFullScale = ReadLE<double>(raw, 40);
    out.dataBytes = ReadLE<uint64_t>(raw, 48);
    out.metaBytes = ReadLE<uint32_t>(raw, 56);
    out.reserved = ReadLE<uint32_t>(raw, 60);
    out.metadata.assign(reinterpret_cast<const char*>(raw.data()) + out.headerBytes, out.metaBytes);

    const uint32_t planeBytes = (out.blockSize * out.mantissaBits + 7) / 8;
    std::size_t offset = out.headerBytes + out.metaBytes;
    while (offset + out.blockStride <= raw.size())
    {
        const int exponent = static_cast<int8_t>(raw[offset]);
        out.exponents.push_back(exponent);
        const uint8_t* planeI = raw.data() + offset + 1;
        const uint8_t* planeQ = planeI + planeBytes;
        for (uint32_t i = 0; i < out.blockSize; ++i)
        {
            if (exponent == -128)
            {
                out.real.push_back(0.0);
                out.imag.push_back(0.0);
                continue;
            }
            out.real.push_back(ReadMantissa(planeI, i, out.mantissaBits) * std::pow(2.0, exponent));
            out.imag.push_back(ReadMantissa(planeQ, i, out.mantissaBits) * std::pow(2.0, exponent));
        }
        offset += out.blockStride;
    }
    return out;
}

std::string TempPath(const char* name)
{
    return (std::filesystem::temp_directory_path() / name).string();
}

} // namespace

TEST(BIQWriter, HeaderMatchesTheConfiguration)
{
    const std::string path = TempPath("limesuiteng_biq_header.biq");

    BIQWriter::Config config;
    config.sampleRate_Hz = 30.72e6;
    config.centerFrequency_Hz = 2.45e9;
    config.startTime_unix_ns = 1234567890123456789LL;
    config.refFullScale_dBm = -10.5;
    config.mantissaBits = 6;
    config.blockSize = 256;

    BIQWriter writer;
    ASSERT_EQ(writer.Open(path, config, "{\"core:recorder\":\"limeTRX\"}"), OpStatus::Success);
    const std::vector<complex16_t> samples(256, complex16_t(1000, -2000));
    EXPECT_EQ(writer.Write(samples.data(), samples.size()), OpStatus::Success);
    EXPECT_EQ(writer.Close(), OpStatus::Success);

    const DecodedFile decoded = Decode(path);
    EXPECT_EQ(decoded.magic, "BIQ1");
    EXPECT_EQ(decoded.headerBytes, 64);
    EXPECT_EQ(decoded.flags, 0); // complex valued
    EXPECT_EQ(decoded.channels, 1); // reserved, fixed at 1 in this version
    EXPECT_EQ(decoded.mantissaBits, 6);
    EXPECT_EQ(decoded.fillRule, 0);
    EXPECT_EQ(decoded.blockSize, 256);
    EXPECT_EQ(decoded.blockStride, 1 + 2 * (256 * 6 / 8));
    EXPECT_DOUBLE_EQ(decoded.sampleRate, 30.72e6);
    EXPECT_DOUBLE_EQ(decoded.centerFrequency, 2.45e9);
    EXPECT_EQ(decoded.startTime, 1234567890123456789LL);
    EXPECT_DOUBLE_EQ(decoded.refFullScale, -10.5);
    EXPECT_EQ(decoded.reserved, 0u);

    // the metadata is padded out to a multiple of 8 bytes so the blocks stay aligned
    EXPECT_EQ(decoded.metaBytes % 8, 0u);
    EXPECT_EQ(decoded.metadata.substr(0, 27), "{\"core:recorder\":\"limeTRX\"}");
    EXPECT_EQ(decoded.dataBytes, decoded.blockStride); // one block was written

    std::filesystem::remove(path);
}

TEST(BIQWriter, SamplesSurviveARoundTripWithinTheQuantisationStep)
{
    for (const uint8_t bits : { 4, 6, 8, 12 })
    {
        const std::string path = TempPath("limesuiteng_biq_roundtrip.biq");

        BIQWriter::Config config;
        config.sampleRate_Hz = 1e6;
        config.mantissaBits = bits;
        config.blockSize = 64;

        std::vector<complex16_t> samples;
        for (int i = 0; i < 64; ++i)
            samples.push_back(complex16_t(static_cast<int16_t>(-32768 + i * 1024), static_cast<int16_t>(32767 - i * 1024)));

        BIQWriter writer;
        ASSERT_EQ(writer.Open(path, config, ""), OpStatus::Success);
        ASSERT_EQ(writer.Write(samples.data(), samples.size()), OpStatus::Success);
        ASSERT_EQ(writer.Close(), OpStatus::Success);

        const DecodedFile decoded = Decode(path);
        ASSERT_EQ(decoded.exponents.size(), 1u);
        ASSERT_EQ(decoded.real.size(), samples.size());

        // block floating point holds every sample to within half a step of the block exponent
        const double step = std::pow(2.0, decoded.exponents[0]);
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            EXPECT_NEAR(decoded.real[i], samples[i].real() / 32768.0, step / 2) << "bits " << int(bits) << ", sample " << i;
            EXPECT_NEAR(decoded.imag[i], samples[i].imag() / 32768.0, step / 2) << "bits " << int(bits) << ", sample " << i;
        }

        std::filesystem::remove(path);
    }
}

TEST(BIQWriter, AnAllZeroBlockUsesTheSentinelExponent)
{
    const std::string path = TempPath("limesuiteng_biq_zero.biq");

    BIQWriter::Config config;
    config.mantissaBits = 6;
    config.blockSize = 8;

    // first block carries signal, second is silent
    std::vector<complex16_t> samples(8, complex16_t(4096, -4096));
    samples.resize(16, complex16_t(0, 0));

    BIQWriter writer;
    ASSERT_EQ(writer.Open(path, config, ""), OpStatus::Success);
    ASSERT_EQ(writer.Write(samples.data(), samples.size()), OpStatus::Success);
    ASSERT_EQ(writer.Close(), OpStatus::Success);

    const DecodedFile decoded = Decode(path);
    ASSERT_EQ(decoded.exponents.size(), 2u);
    EXPECT_NE(decoded.exponents[0], -128);
    EXPECT_EQ(decoded.exponents[1], -128);
    for (std::size_t i = 8; i < 16; ++i)
    {
        EXPECT_DOUBLE_EQ(decoded.real[i], 0.0);
        EXPECT_DOUBLE_EQ(decoded.imag[i], 0.0);
    }

    std::filesystem::remove(path);
}

TEST(BIQWriter, TrailingPartialBlockIsPaddedWithZeros)
{
    const std::string path = TempPath("limesuiteng_biq_tail.biq");

    BIQWriter::Config config;
    config.mantissaBits = 8;
    config.blockSize = 64;

    // 100 samples is one whole block plus a 36 sample remainder
    std::vector<complex16_t> samples(100, complex16_t(2048, 1024));

    BIQWriter writer;
    ASSERT_EQ(writer.Open(path, config, ""), OpStatus::Success);
    ASSERT_EQ(writer.Write(samples.data(), samples.size()), OpStatus::Success);
    ASSERT_EQ(writer.Close(), OpStatus::Success);

    const DecodedFile decoded = Decode(path);
    EXPECT_EQ(decoded.exponents.size(), 2u);
    EXPECT_EQ(decoded.real.size(), 128u);
    EXPECT_EQ(decoded.dataBytes, 2u * decoded.blockStride);
    for (std::size_t i = 100; i < 128; ++i)
    {
        EXPECT_DOUBLE_EQ(decoded.real[i], 0.0);
        EXPECT_DOUBLE_EQ(decoded.imag[i], 0.0);
    }

    std::filesystem::remove(path);
}

TEST(BIQWriter, RejectsConfigurationsTheFormatCannotHold)
{
    const std::string path = TempPath("limesuiteng_biq_reject.biq");
    BIQWriter writer;

    BIQWriter::Config config;
    config.mantissaBits = 1; // below the 2 bit minimum
    EXPECT_EQ(writer.Open(path, config, ""), OpStatus::InvalidValue);

    config.mantissaBits = 17; // above the 16 bit maximum
    EXPECT_EQ(writer.Open(path, config, ""), OpStatus::InvalidValue);

    config.mantissaBits = 6;
    config.blockSize = 100; // block size has to be a multiple of 8
    EXPECT_EQ(writer.Open(path, config, ""), OpStatus::InvalidValue);

    config.blockSize = 0;
    EXPECT_EQ(writer.Open(path, config, ""), OpStatus::InvalidValue);

    EXPECT_FALSE(writer.IsOpen());
}
