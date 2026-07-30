#include <gtest/gtest.h>

#include "boards/ChannelAlignment.h"
#include "comms/SPI/ISPI.h"

#include <cmath>
#include <cstring>
#include <map>
#include <utility>
#include <vector>

using namespace lime;
using namespace lime::channelalignment;

namespace {

constexpr double pi = 3.14159265358979323846;

/// Builds one packet payload with a tone at the given bin on both channels, the B
/// channel shifted by phaseDegrees.
std::vector<int16_t> TonePayload(int bin, double phaseDegrees)
{
    std::vector<int16_t> payload(samplesPerChannel * 4);
    const double w = 2.0 * pi * bin / dftLength;
    const double phi = phaseDegrees * pi / 180.0;
    for (int n = 0; n < samplesPerChannel; ++n)
    {
        payload[4 * n + 0] = static_cast<int16_t>(std::lround(8000 * std::cos(w * n)));
        payload[4 * n + 1] = static_cast<int16_t>(std::lround(8000 * std::sin(w * n)));
        payload[4 * n + 2] = static_cast<int16_t>(std::lround(8000 * std::cos(w * n + phi)));
        payload[4 * n + 3] = static_cast<int16_t>(std::lround(8000 * std::sin(w * n + phi)));
    }
    return payload;
}

/// Chip SPI double: serves reads from a register map, records every write. The RxTSP
/// configuration registers are kept per channel, selected by the MAC bits of 0x0020,
/// so channel A and B backups can be told apart.
class FakeChipSPI : public ISPI
{
  public:
    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            const uint32_t word = MOSI[i];
            const uint16_t addr = (word >> 16) & 0x7FFF;
            if (word & (1u << 31))
            {
                const uint16_t value = word & 0xFFFF;
                writes.push_back({ addr, value });
                Store(addr, value);
            }
            else if (MISO)
            {
                MISO[i] = Load(addr);
            }
        }
        return OpStatus::Success;
    }

    void Store(uint16_t addr, uint16_t value)
    {
        if (addr < 0x0100)
        {
            registers[addr] = value;
            return;
        }
        const uint16_t mac = registers.count(0x0020) ? registers[0x0020] & 0x3 : 0x3;
        if (mac & 0x1)
            channelA[addr] = value;
        if (mac & 0x2)
            channelB[addr] = value;
    }

    uint16_t Load(uint16_t addr)
    {
        if (addr < 0x0100)
            return registers.count(addr) ? registers[addr] : 0;
        const uint16_t mac = registers.count(0x0020) ? registers[0x0020] & 0x3 : 0x3;
        auto& bank = (mac & 0x1) ? channelA : channelB;
        return bank.count(addr) ? bank[addr] : 0;
    }

    std::map<uint16_t, uint16_t> registers;
    std::map<uint16_t, uint16_t> channelA;
    std::map<uint16_t, uint16_t> channelB;
    std::vector<std::pair<uint16_t, uint16_t>> writes;
};

struct FakeBoard {
    FakeChipSPI spi;
    std::vector<std::pair<uint32_t, uint32_t>> fpgaWrites;
    int startCount{ 0 };
    int stopCount{ 0 };
    int packetsServed{ 0 };

    BoardPorts Ports()
    {
        BoardPorts ports;
        ports.chip = nullptr; // the sequences under test never touch the high level chip API
        ports.chipSPI = &spi;
        ports.writeFpgaRegister = [this](uint32_t addr, uint32_t value) { fpgaWrites.push_back({ addr, value }); };
        ports.startStreaming = [this]() { ++startCount; };
        ports.stopStreaming = [this]() { ++stopCount; };
        ports.resetStreamBuffers = []() {};
        ports.receivePacket = [this](uint8_t* dest, std::size_t length) {
            std::memset(dest, 0, length);
            uint32_t* words = reinterpret_cast<uint32_t*>(dest);
            ++packetsServed;
            // the first samples of channel A and B differ until the third packet
            words[4] = 1;
            words[5] = packetsServed >= 3 ? 1 : 2;
            return true;
        };
        return ports;
    }
};

} // namespace

TEST(ChannelAlignment, PhaseDifferenceRecoversTheInjectedShift)
{
    for (const double injected : { -150.0, -22.5, 0.0, 1.0, 90.0, 170.0 })
    {
        const std::vector<int16_t> payload = TonePayload(32, injected);
        const double measured = PhaseDifferenceAtBin(payload.data(), samplesPerChannel, 32);
        EXPECT_NEAR(measured, injected, 0.3) << "injected " << injected;
    }
}

TEST(ChannelAlignment, PhaseDifferenceWrapsInto180Range)
{
    const std::vector<int16_t> payload = TonePayload(32, 190.0);
    const double measured = PhaseDifferenceAtBin(payload.data(), samplesPerChannel, 32);
    EXPECT_NEAR(measured, -170.0, 0.3);
}

// One LML sample of delay on channel B is the defect signature of section 2.1 of the
// alignment note: at bin 32 of 512 it shows up as exactly -22.5 degrees.
TEST(ChannelAlignment, OneSampleDelayShowsAsBinProportionalPhase)
{
    std::vector<int16_t> payload(samplesPerChannel * 4);
    const double w = 2.0 * pi * 32 / dftLength;
    for (int n = 0; n < samplesPerChannel; ++n)
    {
        payload[4 * n + 0] = static_cast<int16_t>(std::lround(8000 * std::cos(w * n)));
        payload[4 * n + 1] = static_cast<int16_t>(std::lround(8000 * std::sin(w * n)));
        payload[4 * n + 2] = static_cast<int16_t>(std::lround(8000 * std::cos(w * (n - 1))));
        payload[4 * n + 3] = static_cast<int16_t>(std::lround(8000 * std::sin(w * (n - 1))));
    }
    const double measured = PhaseDifferenceAtBin(payload.data(), samplesPerChannel, 32);
    EXPECT_NEAR(measured, -360.0 * 32 / dftLength, 0.3);
}

TEST(ChannelAlignment, RstRxIQGenTogglesTheQuadratureGeneratorAndRestores)
{
    FakeBoard board;
    board.spi.registers[0x0020] = 0xBFFF;
    // 0xFFFD selects MAC=1, which addresses channel A / SXR, so that is the bank the
    // SXR forward divider register is read from
    board.spi.channelA[0x011C] = 0x4300;
    board.spi.channelA[0x010C] = 0x88A5;
    board.spi.channelB[0x010C] = 0x88A5;

    RxChannelAligner aligner(board.Ports());
    aligner.RstRxIQGen();

    // the sequence from the alignment note: gate the SXR forward divider, take direct
    // control of the power downs, pulse PD_QGEN_RFE on both channels, then restore
    const std::vector<std::pair<uint16_t, uint16_t>> expectedTail = {
        { 0x0020, 0xFFFD },
        { 0x011C, 0x4300 | 0x10 },
        { 0x0020, 0xFFFF },
        { 0x0124, 0x001F },
        { 0x010C, 0x88A5 | 0x8 },
        { 0x010C, 0x88A5 },
        { 0x0020, 0xFFFD },
        { 0x011C, 0x4300 },
        { 0x0020, 0xBFFF },
    };
    ASSERT_GE(board.spi.writes.size(), expectedTail.size());
    const std::size_t offset = board.spi.writes.size() - expectedTail.size();
    for (std::size_t i = 0; i < expectedTail.size(); ++i)
    {
        EXPECT_EQ(board.spi.writes[offset + i].first, expectedTail[i].first) << "write " << i;
        EXPECT_EQ(board.spi.writes[offset + i].second, expectedTail[i].second) << "write " << i;
    }
}

TEST(ChannelAlignment, AlignRxTSPResetsUntilChannelsMatchAndRestores)
{
    FakeBoard board;
    board.spi.registers[0x0020] = 0xFFFF;
    board.spi.channelA[0x0400] = 0x1111;
    board.spi.channelA[0x040C] = 0x2222;
    board.spi.channelB[0x0400] = 0x3333;
    board.spi.channelB[0x040C] = 0x4444;

    RxChannelAligner aligner(board.Ports());
    aligner.AlignRxTSP();

    // the fake produces matching channel samples on the third packet
    EXPECT_EQ(board.packetsServed, 3);
    EXPECT_EQ(board.startCount, 3);
    // one framer stop in the capture setup, one after each successful read
    EXPECT_EQ(board.stopCount, 4);

    // capture setup: chip select, 16 bit samples, both channels enabled
    ASSERT_GE(board.fpgaWrites.size(), 3u);
    EXPECT_EQ(board.fpgaWrites[0], std::make_pair(0xFFFFu, 0x1u));
    EXPECT_EQ(board.fpgaWrites[1], std::make_pair(0x0008u, 0x0100u));
    EXPECT_EQ(board.fpgaWrites[2], std::make_pair(0x0007u, 3u));

    // every reset pulse is the 0x55FE / 0xFFFD pair
    int resetPulses = 0;
    for (std::size_t i = 0; i + 1 < board.spi.writes.size(); ++i)
    {
        if (board.spi.writes[i] == std::make_pair<uint16_t, uint16_t>(0x0020, 0x55FE) &&
            board.spi.writes[i + 1] == std::make_pair<uint16_t, uint16_t>(0x0020, 0xFFFD))
            ++resetPulses;
    }
    EXPECT_EQ(resetPulses, 3);

    // the backed up per channel test signal registers come back and the original
    // channel selection is the last write
    const std::vector<std::pair<uint16_t, uint16_t>> expectedTail = {
        { 0x0020, 0xFFFD },
        { 0x0400, 0x1111 },
        { 0x040C, 0x2222 },
        { 0x0020, 0xFFFE },
        { 0x0400, 0x3333 },
        { 0x040C, 0x4444 },
        { 0x0020, 0xFFFF },
    };
    ASSERT_GE(board.spi.writes.size(), expectedTail.size());
    const std::size_t offset = board.spi.writes.size() - expectedTail.size();
    for (std::size_t i = 0; i < expectedTail.size(); ++i)
    {
        EXPECT_EQ(board.spi.writes[offset + i].first, expectedTail[i].first) << "write " << i;
        EXPECT_EQ(board.spi.writes[offset + i].second, expectedTail[i].second) << "write " << i;
    }
}

TEST(ChannelAlignment, GetPhaseOffsetReportsIOFailureWithoutData)
{
    FakeBoard board;
    BoardPorts ports = board.Ports();
    ports.receivePacket = [](uint8_t*, std::size_t) { return false; };

    RxChannelAligner aligner(ports);
    double offset = 12345.0;
    EXPECT_EQ(aligner.GetPhaseOffset(32, &offset), OpStatus::IOFailure);
}
