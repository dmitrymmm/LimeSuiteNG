#include "ChannelAlignment.h"

#include <cmath>
#include <complex>
#include <vector>

#include "comms/SPI/ISPI.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/LMS7002MCSR.h"
#include "limesuiteng/Logger.h"

using namespace std::literals::string_literals;

namespace lime {

namespace channelalignment {

namespace {

constexpr uint32_t spiWrite(uint16_t address, uint16_t value)
{
    return (1u << 31) | (static_cast<uint32_t>(address) << 16) | value;
}

constexpr uint32_t spiRead(uint16_t address)
{
    return static_cast<uint32_t>(address) << 16;
}

} // namespace

double PhaseDifferenceAtBin(const int16_t* interleavedSamples, int samplesPerChannelCount, int bin)
{
    const std::complex<double> iunit(0, 1);
    const double pi = std::acos(-1);
    std::complex<double> xA(0, 0);
    std::complex<double> xB(0, 0);
    for (int n = 0; n < samplesPerChannelCount; n++)
    {
        const std::complex<double> xAn(interleavedSamples[4 * n], interleavedSamples[4 * n + 1]);
        const std::complex<double> xBn(interleavedSamples[4 * n + 2], interleavedSamples[4 * n + 3]);
        const std::complex<double> mult = std::exp(-2.0 * iunit * pi * double(bin) * double(n) / double(dftLength));
        xA += xAn * mult;
        xB += xBn * mult;
    }
    const double phaseA = std::arg(xA) * 180.0 / pi;
    const double phaseB = std::arg(xB) * 180.0 / pi;
    double phasediff = phaseB - phaseA;
    if (phasediff < -180.0)
        phasediff += 360.0;
    if (phasediff > 180.0)
        phasediff -= 360.0;
    return phasediff;
}

RxChannelAligner::RxChannelAligner(const BoardPorts& ports)
    : mPorts(ports)
{
}

void RxChannelAligner::PrepareCapture()
{
    // capture setup used by every measurement: select the chip, stop the framer, use
    // 16 bit samples and enable both channels
    mPorts.stopStreaming();
    mPorts.writeFpgaRegister(0xFFFF, 0x1);
    mPorts.writeFpgaRegister(0x0008, 0x0100);
    mPorts.writeFpgaRegister(0x0007, 3);
}

OpStatus RxChannelAligner::GetPhaseOffset(int bin, double* output)
{
    std::vector<uint8_t> buf(packetSize);

    mPorts.resetStreamBuffers();
    mPorts.startStreaming();
    if (!mPorts.receivePacket(buf.data(), buf.size()))
    {
        lime::warning("Channel alignment: no Rx data"s);
        return OpStatus::IOFailure;
    }
    mPorts.stopStreaming();
    mPorts.resetStreamBuffers();

    const int16_t* samples = reinterpret_cast<const int16_t*>(buf.data() + packetHeaderBytes);
    *output = PhaseDifferenceAtBin(samples, samplesPerChannel, bin);
    return OpStatus::Success;
}

void RxChannelAligner::RstRxIQGen()
{
    uint32_t data[16];
    uint32_t reg20 = 0;
    uint32_t reg11C = 0;
    uint32_t reg10C = 0;
    data[0] = spiRead(0x0020);
    mPorts.chipSPI->Transact(data, &reg20, 1);
    data[0] = spiRead(0x010C);
    mPorts.chipSPI->Transact(data, &reg10C, 1);
    data[0] = spiWrite(0x0020, 0xFFFD); // channel B
    mPorts.chipSPI->Transact(data, nullptr, 1);
    data[0] = spiRead(0x011C);
    mPorts.chipSPI->Transact(data, &reg11C, 1);
    data[0] = spiWrite(0x0020, 0xFFFD); // SXR
    data[1] = spiWrite(0x011C, reg11C | 0x10); // PD_FDIV
    data[2] = spiWrite(0x0020, 0xFFFF); // mac 3 - both channels
    data[3] = spiWrite(0x0124, 0x001F); // direct control of powerdowns
    data[4] = spiWrite(0x010C, reg10C | 0x8); // PD_QGEN_RFE
    data[5] = spiWrite(0x010C, reg10C); // restore value
    data[6] = spiWrite(0x0020, 0xFFFD); // SXR
    data[7] = spiWrite(0x011C, reg11C); // restore value
    data[8] = spiWrite(0x0020, reg20); // restore value
    mPorts.chipSPI->Transact(data, nullptr, 9);
}

void RxChannelAligner::AlignRxTSP()
{
    uint32_t reg20 = 0;
    uint32_t regsA[2] = { 0, 0 };
    uint32_t regsB[2] = { 0, 0 };
    // backup values
    {
        const uint32_t bakAddr[2] = { spiRead(0x0400), spiRead(0x040C) };
        uint32_t data = spiRead(0x0020);
        mPorts.chipSPI->Transact(&data, &reg20, 1);
        data = spiWrite(0x0020, 0xFFFD);
        mPorts.chipSPI->Transact(&data, nullptr, 1);
        mPorts.chipSPI->Transact(bakAddr, regsA, 2);
        data = spiWrite(0x0020, 0xFFFE);
        mPorts.chipSPI->Transact(&data, nullptr, 1);
        mPorts.chipSPI->Transact(bakAddr, regsB, 2);
    }

    // alignment search: both RxTSP generate the same test signal, reset them together
    // until the first sample of both channels comes out identical
    {
        uint32_t dataWr[4];
        dataWr[0] = spiWrite(0x0020, 0xFFFF);
        dataWr[1] = spiWrite(0x0400, 0x8085);
        dataWr[2] = spiWrite(0x040C, 0x01FF);
        mPorts.chipSPI->Transact(dataWr, nullptr, 3);

        PrepareCapture();

        dataWr[0] = spiWrite(0x0020, 0x55FE);
        dataWr[1] = spiWrite(0x0020, 0xFFFD);

        std::vector<uint8_t> buf(packetSize);
        for (int i = 0; i < 100; i++)
        {
            mPorts.chipSPI->Transact(dataWr, nullptr, 2);
            mPorts.resetStreamBuffers();
            mPorts.startStreaming();
            if (!mPorts.receivePacket(buf.data(), buf.size()))
            {
                lime::warning("Channel alignment: no Rx data"s);
                break;
            }
            mPorts.stopStreaming();
            mPorts.resetStreamBuffers();
            const uint32_t* words = reinterpret_cast<const uint32_t*>(buf.data());
            if (words[4] == words[5])
                break;
        }
    }

    // restore values
    {
        uint32_t dataWr[7];
        dataWr[0] = spiWrite(0x0020, 0xFFFD);
        dataWr[1] = spiWrite(0x0400, regsA[0]);
        dataWr[2] = spiWrite(0x040C, regsA[1]);
        dataWr[3] = spiWrite(0x0020, 0xFFFE);
        dataWr[4] = spiWrite(0x0400, regsB[0]);
        dataWr[5] = spiWrite(0x040C, regsB[1]);
        dataWr[6] = spiWrite(0x0020, reg20);
        mPorts.chipSPI->Transact(dataWr, nullptr, 7);
    }
}

OpStatus RxChannelAligner::AlignQuadrature()
{
    LMS7002M* lms = mPorts.chip;
    LMS7002M_RegistersMap* regBackup = lms->BackupRegisterMap();

    // internal RF loopback test setup. Only Tx A transmits, Rx B relies on leakage, so
    // a possible 180 degree flip of a Tx quadrature generator cannot mask an Rx one.
    lms->SPI_write(0x20, 0xFFFF);
    lms->SetDefaults(LMS7002M::MemorySection::RBB);
    lms->SetDefaults(LMS7002M::MemorySection::TBB);
    lms->SetDefaults(LMS7002M::MemorySection::TRF);
    lms->SPI_write(0x113, 0x0046);
    lms->SPI_write(0x118, 0x418C);
    lms->SPI_write(0x100, 0x4039);
    lms->SPI_write(0x101, 0x7801);
    lms->SPI_write(0x108, 0x318C);
    lms->SPI_write(0x082, 0x8001);
    lms->SPI_write(0x200, 0x008D);
    lms->SPI_write(0x208, 0x01FB);
    lms->SPI_write(0x400, 0x8081);
    lms->SPI_write(0x40C, 0x01FF);
    lms->SPI_write(0x404, 0x0006);
    lms->LoadDC_REG_IQ(TRXDir::Tx, 0x3FFF, 0x3FFF);
    lms->SPI_write(0x20, 0xFFFE);
    lms->SPI_write(0x105, 0x0006);
    lms->SPI_write(0x100, 0x4038);
    lms->SPI_write(0x113, 0x007F);
    lms->SPI_write(0x119, 0x529B);
    uint16_t val = lms->Get_SPI_Reg_bits(LMS7002MCSR::SEL_PATH_RFE, true);
    lms->SPI_write(0x10D, val == 3 ? 0x18F : val == 2 ? 0x117 : 0x08F);
    lms->SPI_write(0x10C, val == 2 ? 0x88C5 : 0x88A5);
    lms->SPI_write(0x20, 0xFFFD);
    lms->SPI_write(0x103, val == 2 ? 0x612 : 0xA12);
    val = lms->Get_SPI_Reg_bits(LMS7002MCSR::SEL_PATH_RFE, true);
    lms->SPI_write(0x10D, val == 3 ? 0x18F : val == 2 ? 0x117 : 0x08F);
    lms->SPI_write(0x10C, val == 2 ? 0x88C5 : 0x88A5);
    lms->SPI_write(0x119, 0x5293);
    const double srate = lms->GetSampleRate(TRXDir::Rx, LMS7002M::Channel::ChA);
    const double freq = lms->GetFrequencySX(TRXDir::Rx);

    PrepareCapture();
    lms->SetFrequencySX(TRXDir::Tx, freq + srate / 16.0);
    bool found = false;
    for (int i = 0; i < 100; i++)
    {
        double offset = 0;
        if (GetPhaseOffset(32, &offset) != OpStatus::Success)
            break;
        if (std::fabs(offset) <= 90.0)
        {
            found = true;
            break;
        }
        RstRxIQGen();
    }

    lms->RestoreRegisterMap(regBackup); // frees the backup
    if (!found)
        lime::warning("Channel alignment failed"s);
    return found ? OpStatus::Success : OpStatus::Error;
}

OpStatus RxChannelAligner::AlignRxRF()
{
    LMS7002M* lms = mPorts.chip;
    const uint16_t reg20 = lms->SPI_read(0x20);
    LMS7002M_RegistersMap* regBackup = lms->BackupRegisterMap();

    // internal RF loopback test setup: both Tx share the SXT LO and send the same DC
    // test signal, so whatever reaches the two Rx channels is inherently in phase
    lms->SPI_write(0x20, 0xFFFF);
    lms->SetDefaults(LMS7002M::MemorySection::RFE);
    lms->SetDefaults(LMS7002M::MemorySection::RBB);
    lms->SetDefaults(LMS7002M::MemorySection::TBB);
    lms->SetDefaults(LMS7002M::MemorySection::TRF);
    lms->SPI_write(0x10C, 0x88C5);
    lms->SPI_write(0x10D, 0x0117);
    lms->SPI_write(0x113, 0x024A);
    lms->SPI_write(0x118, 0x418C);
    lms->SPI_write(0x100, 0x4039);
    lms->SPI_write(0x101, 0x7801);
    lms->SPI_write(0x103, 0x0612);
    lms->SPI_write(0x108, 0x318C);
    lms->SPI_write(0x082, 0x8001);
    lms->SPI_write(0x200, 0x008D);
    lms->SPI_write(0x208, 0x01FB);
    lms->SPI_write(0x400, 0x8081);
    lms->SPI_write(0x40C, 0x01FF);
    lms->SPI_write(0x404, 0x0006);
    lms->LoadDC_REG_IQ(TRXDir::Tx, 0x3FFF, 0x3FFF);
    const double srate = lms->GetSampleRate(TRXDir::Rx, LMS7002M::Channel::ChA);
    lms->SetFrequencySX(TRXDir::Rx, 450e6);
    int dec = lms->Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    if (dec > 4)
        dec = 0;

    // with a residual clock offset the phase difference grows linearly with signal
    // frequency, so it is measured at two frequencies and their difference has to
    // land on the expected slope for the offset to be zero
    const double offsets[] = { 1.15 / 60.0, 1.1 / 40.0, 0.55 / 20.0, 0.2 / 10.0, 0.18 / 5.0 };
    const double tolerance[] = { 0.9, 0.45, 0.25, 0.14, 0.06 };
    const double offset = offsets[dec] * srate / 1e6;

    PrepareCapture();
    bool found = false;
    for (int i = 0; i < 200; i++)
    {
        // toggle the CGEN forward divider to shuffle the clock phase, then realign the
        // sample position before measuring
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_FDIV_O_CGEN, 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_FDIV_O_CGEN, 0);
        AlignRxTSP();

        double offset1 = 0;
        lms->SetFrequencySX(TRXDir::Tx, 450e6 + srate / 16.0);
        if (GetPhaseOffset(32, &offset1) != OpStatus::Success)
            break;
        double offset2 = 0;
        lms->SetFrequencySX(TRXDir::Tx, 450e6 + srate / 8.0);
        if (GetPhaseOffset(64, &offset2) != OpStatus::Success)
            break;
        const double diff = offset1 - offset2;
        if (std::fabs(diff - offset) < tolerance[dec])
        {
            found = true;
            break;
        }
    }

    lms->RestoreRegisterMap(regBackup); // frees the backup

    OpStatus status = OpStatus::Error;
    if (found)
        status = AlignQuadrature();
    else
        lime::warning("Channel alignment failed"s);
    lms->SPI_write(0x20, reg20);
    return status;
}

} // namespace channelalignment

} // namespace lime
