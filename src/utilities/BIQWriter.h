#ifndef LIME_BIQWRITER_H
#define LIME_BIQWRITER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"

namespace lime {

/// @brief Writer for the BIQ1 block floating point IQ capture container.
///
/// The format is described at https://github.com/ibelinp/biq-format . A file is a 64 byte
/// header, optional JSON metadata, then a stream of fixed size blocks. Each block holds one
/// shared exponent followed by a plane of I mantissas and a plane of Q mantissas, and a
/// decoder recovers a sample as mantissa * 2^exponent.
///
/// Samples are handed in as 16 bit integers and stored so that they decode to the -1 to 1
/// range, which is what the ref_dbm_full_scale header field is calibrated against. That
/// scaling is a shift of the block exponent, so no precision is lost to it.
class LIME_API BIQWriter
{
  public:
    /// @brief Values recorded in the file header.
    struct Config {
        double sampleRate_Hz{ 0 }; ///< Complex samples per second.
        double centerFrequency_Hz{ 0 }; ///< Centre frequency of the capture.
        int64_t startTime_unix_ns{ 0 }; ///< UTC nanoseconds of the first sample.
        double refFullScale_dBm{ 0 }; ///< dBm of a full scale sine, NaN when uncalibrated.
        uint8_t mantissaBits{ 6 }; ///< Mantissa width, 2 to 16. 4 survey, 6 general, 8 archive.
        uint16_t blockSize{ 256 }; ///< Samples per block, a non zero multiple of 8.
    };

    BIQWriter();
    ~BIQWriter();

    BIQWriter(const BIQWriter&) = delete;
    BIQWriter& operator=(const BIQWriter&) = delete;

    /// @brief Creates the file and writes the header and the metadata section.
    /// @param filename Path of the file to create.
    /// @param config Header values, see Config.
    /// @param metadata JSON metadata to embed, may be empty.
    /// @return Success, InvalidValue for an unrepresentable configuration, IOFailure otherwise.
    OpStatus Open(const std::string& filename, const Config& config, const std::string& metadata);

    /// @brief Appends samples. Whole blocks are encoded and written as they are filled.
    /// @param samples Interleaved complex samples.
    /// @param count Number of complex samples.
    /// @return Success, or IOFailure if the file is not open or a write failed.
    OpStatus Write(const complex16_t* samples, uint32_t count);

    /// @brief Zero pads the trailing partial block, records the payload length and closes.
    /// @return Success, or IOFailure if a write failed.
    OpStatus Close();

    /// @brief Whether a file is currently open for writing.
    bool IsOpen() const { return mFile.is_open(); }

  private:
    OpStatus FlushBlock();

    std::ofstream mFile;
    Config mConfig{};
    std::vector<complex16_t> mPending; ///< samples that do not yet form a whole block
    std::vector<uint8_t> mBlock; ///< scratch buffer for one encoded block
    uint32_t mBlockBytes{ 0 };
    uint64_t mDataBytes{ 0 };
};

} // namespace lime

#endif
