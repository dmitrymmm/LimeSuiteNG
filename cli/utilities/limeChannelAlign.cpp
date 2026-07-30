#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/complex.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <iostream>
#include <string>
#include <vector>
#include "args.hxx"

#include "common.h"

using namespace lime;
using namespace lime::cli;
using namespace std;

// Hardware verification for the Rx channel phase alignment (issue #356). Feed BOTH Rx
// inputs the same continuous tone through a splitter, then compare how the phase offset
// between the channels behaves across repeated stream restarts with and without
// --align. Each restart re-runs the alignment when it is enabled.

static double PhaseDifferenceDegrees(
    const std::vector<complex16_t>& chA, const std::vector<complex16_t>& chB, std::size_t count, int bin, int fftSize)
{
    const double pi = std::acos(-1);
    std::complex<double> xA(0, 0);
    std::complex<double> xB(0, 0);
    for (std::size_t n = 0; n < count; ++n)
    {
        const std::complex<double> mult = std::exp(std::complex<double>(0, -2.0 * pi * double(bin) * double(n) / double(fftSize)));
        xA += std::complex<double>(chA[n].real(), chA[n].imag()) * mult;
        xB += std::complex<double>(chB[n].real(), chB[n].imag()) * mult;
    }
    double diff = (std::arg(xB) - std::arg(xA)) * 180.0 / pi;
    if (diff < -180.0)
        diff += 360.0;
    if (diff > 180.0)
        diff -= 360.0;
    return diff;
}

static int FindStrongestBin(const std::vector<complex16_t>& samples, std::size_t count, int fftSize)
{
    const double pi = std::acos(-1);
    int bestBin = 1;
    double bestPower = 0;
    // skip DC and the outermost bins, a real tone will land well inside
    for (int bin = 2; bin < fftSize / 2 - 2; ++bin)
    {
        std::complex<double> x(0, 0);
        for (std::size_t n = 0; n < count; ++n)
        {
            const std::complex<double> mult =
                std::exp(std::complex<double>(0, -2.0 * pi * double(bin) * double(n) / double(fftSize)));
            x += std::complex<double>(samples[n].real(), samples[n].imag()) * mult;
        }
        const double power = std::norm(x);
        if (power > bestPower)
        {
            bestPower = power;
            bestBin = bin;
        }
    }
    return bestBin;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeChannelAlign - measures Rx channel phase offset across stream restarts", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "name", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<double>         freqFlag(parser, "Hz", "Rx LO frequency. Default: 450 MHz", {'f', "freq"}, 450e6);
    args::ValueFlag<double>         samplerateFlag(parser, "Hz", "Sample rate. Default: 10 MHz", {'s', "samplerate"}, 10e6);
    args::ValueFlag<int>            runsFlag(parser, "count", "Stream restarts to measure. Default: 10", {'r', "runs"}, 10);
    args::Flag                      alignFlag(parser, "", "Enable the phase alignment on every restart", {'a', "align"});
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(args::get(deviceFlag));
    if (!device)
        return EXIT_FAILURE;

    const double frequency = args::get(freqFlag);
    const double sampleRate = args::get(samplerateFlag);
    const int runs = args::get(runsFlag);
    const bool align = alignFlag;

    SDRConfig config;
    for (int ch = 0; ch < 2; ++ch)
    {
        config.channel[ch].rx.enabled = true;
        config.channel[ch].rx.centerFrequency = frequency;
        config.channel[ch].rx.sampleRate = sampleRate;
        config.channel[ch].rx.oversample = 2;
        config.channel[ch].rx.path = 2; // LNAL
    }

    try
    {
        if (device->Configure(config, 0) != OpStatus::Success)
        {
            cerr << "Failed to configure device" << endl;
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    } catch (const std::exception& e)
    {
        cerr << "Failed to configure device: " << e.what() << endl;
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    constexpr int fftSize = 512;
    std::vector<complex16_t> chA(fftSize);
    std::vector<complex16_t> chB(fftSize);

    cout << "runs: " << runs << ", alignment " << (align ? "ENABLED" : "DISABLED") << endl;
    cout << "feed the same tone to both Rx inputs through a splitter" << endl;

    StreamConfig streamConfig;
    streamConfig.channels[TRXDir::Rx] = { 0, 1 };
    streamConfig.format = DataFormat::I16;
    streamConfig.linkFormat = DataFormat::I16;
    streamConfig.alignPhase = align;

    int completed = 0;
    double sum = 0;
    double sumSquares = 0;
    for (int run = 0; run < runs; ++run)
    {
        std::unique_ptr<RFStream> stream = device->StreamCreate(streamConfig, 0);
        if (!stream)
        {
            cerr << "Failed to create stream" << endl;
            break;
        }
        stream->Start();

        complex16_t* buffers[2] = { chA.data(), chB.data() };
        uint32_t got = stream->Receive(buffers, fftSize, nullptr);
        // drop the first batch, it may straddle the start
        got = stream->Receive(buffers, fftSize, nullptr);
        stream->Stop();
        stream.reset();

        if (got != fftSize)
        {
            cerr << "run " << run << ": received only " << got << " samples, skipping" << endl;
            continue;
        }

        const int bin = FindStrongestBin(chA, got, fftSize);
        const double offset = PhaseDifferenceDegrees(chA, chB, got, bin, fftSize);
        cout << "run " << run << ": bin " << bin << " phase B-A = " << offset << " deg" << endl;
        ++completed;
        sum += offset;
        sumSquares += offset * offset;
    }

    DeviceRegistry::freeDevice(device);

    if (completed == 0)
    {
        cerr << "No successful measurements" << endl;
        return EXIT_FAILURE;
    }

    const double mean = sum / completed;
    const double variance = sumSquares / completed - mean * mean;
    cout << "measurements: " << completed << ", mean " << mean << " deg, spread (stddev) " << std::sqrt(std::max(0.0, variance))
         << " deg" << endl;
    cout << "aligned channels should repeat the same offset on every run; without alignment" << endl;
    cout << "the offset jumps between runs by multiples of the clock offsets and by 180 deg" << endl;
    return EXIT_SUCCESS;
}
