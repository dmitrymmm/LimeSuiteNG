#ifndef LIME_CHANNELALIGNMENT_H
#define LIME_CHANNELALIGNMENT_H

#include <cstddef>
#include <cstdint>
#include <functional>

#include "limesuiteng/OpStatus.h"

namespace lime {

class ISPI;
class LMS7002M;

/// Rx MIMO phase alignment for LMS7002M based devices, ported from LimeSuite
/// Streamer.cpp and the LimeSDR-USB channel alignment application note. The effects
/// being corrected are properties of the LMS7002M itself, which leaves its two Rx
/// channels with a random relative offset after resets: one LML sample between RxTSP
/// and LML, one TSP clock between AFE and RxTSP, up to three CGEN clocks in the AFE,
/// and a 180 degree flip of a quadrature generator. Alignment is a search: reset the
/// relevant blocks, measure the offset between channels on a known signal, repeat
/// until the offset matches the expectation.
///
/// The procedure itself is device neutral. A board wires it up by filling BoardPorts
/// with its own chip SPI, framer control and raw packet capture; LimeSDR-USB does so
/// in LimeSDR::AlignRxPhase. Boards with a single Rx channel have nothing to align.
namespace channelalignment {

/// One raw Rx packet as the alignment reads it: a 4096 byte transfer with a 16 byte
/// header, followed by 16 bit A/B interleaved IQ samples.
constexpr std::size_t packetSize = 4096;
constexpr std::size_t packetHeaderBytes = 16;
/// A+B complex pair takes 8 bytes, so a packet carries 510 samples per channel. The
/// legacy code summed 512 and read past its packet buffer for the last two; the DFT
/// below stays inside the packet.
constexpr int samplesPerChannel = static_cast<int>((packetSize - packetHeaderBytes) / 8);
/// The DFT length the bin numbers refer to, kept from the legacy implementation so the
/// bin frequencies (srate/16 at bin 32, srate/8 at 64) and tolerances stay comparable.
constexpr int dftLength = 512;

/// @brief Phase difference between the two Rx channels at one DFT bin.
/// @param interleavedSamples Payload of one packet: A.I, A.Q, B.I, B.Q, repeating.
/// @param samplesPerChannelCount Complex samples available per channel.
/// @param bin DFT bin of interest, relative to a transform of dftLength samples.
/// @return Phase of channel B minus phase of channel A, wrapped to [-180, 180] degrees.
double PhaseDifferenceAtBin(const int16_t* interleavedSamples, int samplesPerChannelCount, int bin);

/// The I/O the alignment needs from the board: the chip being aligned, its raw SPI,
/// control of the FPGA framer, and capture of one raw dual channel packet. Every
/// LMS7002M board can provide these; the unit tests bind them to fakes.
struct BoardPorts {
    LMS7002M* chip{ nullptr }; ///< high level chip control, used by the RF stages
    ISPI* chipSPI{ nullptr }; ///< raw batched LMS7002M SPI, 32 bit words, write = bit 31
    std::function<void(uint32_t addr, uint32_t value)> writeFpgaRegister;
    std::function<void()> startStreaming;
    std::function<void()> stopStreaming;
    std::function<void()> resetStreamBuffers; ///< discard any queued Rx data
    std::function<bool(uint8_t* dest, std::size_t length)> receivePacket; ///< one raw packet
};

/// @brief Runs the channel alignment procedures against the given board.
class RxChannelAligner
{
  public:
    explicit RxChannelAligner(const BoardPorts& ports);

    /// @brief The full alignment: RxTSP to LML, then clock offsets, then quadrature.
    /// Saves and restores the chip configuration around the search.
    /// @return Success when both searches converged, Error otherwise.
    OpStatus AlignRxRF();

    /// @brief Aligns the RxTSP to LML sample position by resetting until both channels
    /// produce identical test samples. Restores the registers it touches.
    void AlignRxTSP();

    /// @brief Retries quadrature generator resets until the 180 degree flip is gone.
    /// @return Success when the offset settled below 90 degrees.
    OpStatus AlignQuadrature();

    /// @brief Resets the Rx quadrature generators of both channels by gating their
    /// clock and toggling the RFE quadrature generator power down.
    void RstRxIQGen();

    /// @brief Reads one packet and measures the channel phase difference at a bin.
    /// @param bin DFT bin of interest.
    /// @param output Receives the phase difference in degrees.
    /// @return Success, or IOFailure when no packet arrived.
    OpStatus GetPhaseOffset(int bin, double* output);

  private:
    void PrepareCapture();

    BoardPorts mPorts;
};

} // namespace channelalignment

} // namespace lime

#endif
