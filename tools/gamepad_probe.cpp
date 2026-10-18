#include <windows.h>
#include <dinput.h>
#include <array>
#include <chrono>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {
IDirectInputDevice8W* device{};
std::wstring deviceName;

BOOL CALLBACK enumerate(const DIDEVICEINSTANCEW* instance, VOID* context) {
  auto* directInput = static_cast<IDirectInput8W*>(context);
  if (SUCCEEDED(directInput->CreateDevice(instance->guidInstance, &device, nullptr))) {
    deviceName = instance->tszProductName;
    return DIENUM_STOP;
  }
  return DIENUM_CONTINUE;
}
}

int wmain() {
  UINT rawCount{};GetRawInputDeviceList(nullptr,&rawCount,sizeof(RAWINPUTDEVICELIST));std::vector<RAWINPUTDEVICELIST> rawDevices(rawCount);GetRawInputDeviceList(rawDevices.data(),&rawCount,sizeof(RAWINPUTDEVICELIST));
  for(const auto& item:rawDevices){if(item.dwType!=RIM_TYPEHID)continue;RID_DEVICE_INFO info{};info.cbSize=sizeof(info);UINT infoSize=sizeof(info);if(GetRawInputDeviceInfoW(item.hDevice,RIDI_DEVICEINFO,&info,&infoSize)==UINT(-1)||info.hid.dwVendorId!=0x054c)continue;UINT chars{};GetRawInputDeviceInfoW(item.hDevice,RIDI_DEVICENAME,nullptr,&chars);std::wstring path(chars,L'\0');if(chars)GetRawInputDeviceInfoW(item.hDevice,RIDI_DEVICENAME,path.data(),&chars);std::wcout<<L"RAW_HID VID="<<std::hex<<info.hid.dwVendorId<<L" PID="<<info.hid.dwProductId<<std::dec<<L" USAGE_PAGE="<<info.hid.usUsagePage<<L" USAGE="<<info.hid.usUsage<<L" PATH="<<path.c_str()<<L"\n";}
  IDirectInput8W* directInput{};
  if (FAILED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION,
                                IID_IDirectInput8W, reinterpret_cast<void**>(&directInput), nullptr))) {
    std::wcerr << L"DirectInput8Create failed\n";
    return 1;
  }
  directInput->EnumDevices(DI8DEVCLASS_GAMECTRL, enumerate, directInput, DIEDFL_ATTACHEDONLY);
  if (!device) {
    std::wcerr << L"No DirectInput game controller found\n";
    directInput->Release();
    return 2;
  }
  if (FAILED(device->SetDataFormat(&c_dfDIJoystick2))) {
    std::wcerr << L"SetDataFormat failed\n";
    device->Release(); directInput->Release();
    return 3;
  }
  DIPROPRANGE range{};
  range.diph.dwSize = sizeof(range);
  range.diph.dwHeaderSize = sizeof(DIPROPHEADER);
  range.diph.dwHow = DIPH_DEVICE;
  range.lMin = -1000;
  range.lMax = 1000;
  device->SetProperty(DIPROP_RANGE, &range.diph);
  device->Acquire();

  std::array<long, 8> minimum{}, maximum{}, previous{};
  minimum.fill(std::numeric_limits<long>::max());
  maximum.fill(std::numeric_limits<long>::min());
  bool havePrevious = false;
  std::array<unsigned, 8> changeCount{};
  unsigned samples = 0, changedSamples = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  while (std::chrono::steady_clock::now() < deadline) {
    HRESULT result = device->Poll();
    if (FAILED(result)) { device->Acquire(); result = device->Poll(); }
    DIJOYSTATE2 state{};
    if (SUCCEEDED(result) && SUCCEEDED(device->GetDeviceState(sizeof(state), &state))) {
      const std::array<long, 8> values{state.lX, state.lY, state.lZ, state.lRx,
                                      state.lRy, state.lRz, state.rglSlider[0], state.rglSlider[1]};
      bool changed = false;
      for (size_t i = 0; i < values.size(); ++i) {
        minimum[i] = (std::min)(minimum[i], values[i]);
        maximum[i] = (std::max)(maximum[i], values[i]);
        if (havePrevious && values[i] != previous[i]) { changed = true; ++changeCount[i]; }
      }
      changedSamples += changed;
      previous = values;
      havePrevious = true;
      ++samples;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::wcout << L"DEVICE=" << deviceName << L" SAMPLES=" << samples
             << L" CHANGED_SAMPLES=" << changedSamples << L"\n";
  for (size_t i = 0; i < minimum.size(); ++i)
    std::wcout << L"AXIS" << (i + 1) << L" MIN=" << minimum[i] << L" MAX=" << maximum[i]
               << L" SPAN=" << maximum[i] - minimum[i] << L" CHANGES=" << changeCount[i] << L"\n";
  device->Unacquire(); device->Release(); directInput->Release();
  return samples ? 0 : 4;
}
