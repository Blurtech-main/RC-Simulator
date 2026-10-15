#include <windows.h>
#include <mmsystem.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>

int wmain() {
  const UINT count = waveInGetNumDevs();
  std::wcout << L"CAPTURE_DEVICES=" << count << L"\n";
  for (UINT i = 0; i < count; ++i) {
    WAVEINCAPSW caps{};
    if (waveInGetDevCapsW(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR)
      std::wcout << L"DEVICE[" << i << L"]=" << caps.szPname << L"\n";
  }

  WAVEFORMATEX fmt{};
  bool foundFormat = false;
  for (DWORD rate : {48000u, 44100u, 22050u, 11025u}) {
    for (WORD channels : {WORD(1), WORD(2)}) {
      WAVEFORMATEX candidate{};
      candidate.wFormatTag = WAVE_FORMAT_PCM;
      candidate.nChannels = channels;
      candidate.nSamplesPerSec = rate;
      candidate.wBitsPerSample = 16;
      candidate.nBlockAlign = static_cast<WORD>(channels * 2);
      candidate.nAvgBytesPerSec = rate * candidate.nBlockAlign;
      HWAVEIN query{};
      MMRESULT q = waveInOpen(&query, WAVE_MAPPER, &candidate, 0, 0, WAVE_FORMAT_QUERY);
      std::cout << "FORMAT rate=" << rate << " channels=" << channels << " query=" << q << "\n";
      if (!foundFormat && q == MMSYSERR_NOERROR) { fmt = candidate; foundFormat = true; }
    }
  }
  if (!foundFormat) { std::cout << "NO_SUPPORTED_PCM_FORMAT\n"; return 2; }

  HWAVEIN input{};
  UINT selected = count > 1 ? count - 1 : WAVE_MAPPER;
  std::cout << "SELECTED_DEVICE=" << selected << "\n";
  MMRESULT rc = waveInOpen(&input, selected, &fmt, 0, 0, CALLBACK_NULL);
  if (rc != MMSYSERR_NOERROR) {
    std::cout << "OPEN_FAILED=" << rc << "\n";
    return 2;
  }

  std::vector<int16_t> samples(fmt.nSamplesPerSec * fmt.nChannels * 8);
  WAVEHDR hdr{};
  hdr.lpData = reinterpret_cast<LPSTR>(samples.data());
  hdr.dwBufferLength = static_cast<DWORD>(samples.size() * sizeof(int16_t));
  waveInPrepareHeader(input, &hdr, sizeof(hdr));
  waveInAddBuffer(input, &hdr, sizeof(hdr));
  waveInStart(input);
  std::cout << "CAPTURING_8_SECONDS_MOVE_STICKS_NOW\n";
  Sleep(8000);
  waveInStop(input);
  waveInReset(input);
  waveInUnprepareHeader(input, &hdr, sizeof(hdr));
  waveInClose(input);
  if (hdr.dwBytesRecorded == 0) {
    std::cout << "CAPTURE_FAILED_OR_SILENT_DRIVER bytes=" << hdr.dwBytesRecorded << "\n";
    return 3;
  }

  samples.resize(hdr.dwBytesRecorded / sizeof(int16_t));
  std::ofstream raw("ppm-capture.raw",std::ios::binary);raw.write(reinterpret_cast<const char*>(samples.data()),static_cast<std::streamsize>(samples.size()*sizeof(int16_t)));
  auto [mn, mx] = std::minmax_element(samples.begin(), samples.end());
  double sum2 = 0.0;
  for (auto s : samples) sum2 += double(s) * double(s);
  const double rms = std::sqrt(sum2 / samples.size());
  const int peak = std::max(std::abs(int(*mn)), std::abs(int(*mx)));
  std::cout << "SAMPLES=" << samples.size() << " MIN=" << *mn << " MAX=" << *mx
            << " RMS=" << rms << " PEAK=" << peak << "\n";

  size_t bestPpm = 0, bestSync = 0, bestEdges = 0; int bestThreshold = 0;
  for (int threshold : {150, 250, 400, 650, 1000, 1600, 2500}) {
    std::vector<size_t> edges; bool high = false;
    for (size_t i = 0; i < samples.size(); i += fmt.nChannels) {
      bool now = std::abs(int(samples[i])) >= threshold;
      if (now && !high && (edges.empty() || i - edges.back() > size_t(fmt.nChannels * 5))) edges.push_back(i);
      high = now;
    }
    size_t ppmIntervals = 0, syncGaps = 0;
    for (size_t i = 1; i < edges.size(); ++i) {
      double us = double(edges[i] - edges[i - 1]) * 1000000.0 / (fmt.nSamplesPerSec * fmt.nChannels);
      if (us >= 700 && us <= 2300) ++ppmIntervals;
      if (us >= 3000 && us <= 30000) ++syncGaps;
    }
    std::cout << "SCAN threshold=" << threshold << " edges=" << edges.size() << " channels=" << ppmIntervals << " sync=" << syncGaps << "\n";
    if (ppmIntervals > bestPpm) { bestPpm=ppmIntervals; bestSync=syncGaps; bestEdges=edges.size(); bestThreshold=threshold; }
  }
  const size_t ppmIntervals=bestPpm, syncGaps=bestSync;
  std::cout << "BEST_THRESHOLD=" << bestThreshold << " EDGES=" << bestEdges << " PPM_CHANNEL_INTERVALS=" << ppmIntervals
            << " FRAME_SYNC_GAPS=" << syncGaps << "\n";
  const bool level = peak > 500 && rms > 40.0;
  const bool ppm = ppmIntervals >= 20 && syncGaps >= 2;
  std::cout << "SIGNAL=" << (level ? "PRESENT" : "NOT_DETECTED")
            << " PPM=" << (ppm ? "LIKELY" : "NOT_CONFIRMED") << "\n";
  return level ? 0 : 4;
}
