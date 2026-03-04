#ifdef _WIN32
#include "WindowsSapiReader.h"
#include "core/Logging.h"
#include <sapi.h>
#include <string>
#include <locale>
#include <codecvt>

// Helper: UTF-8 -> wstring for SAPI's wide-char API
static std::wstring toWide(const std::string& s)
{
    std::wstring out(s.size(), L'\0');
    size_t n = 0;
    mbstowcs_s(&n, out.data(), out.size() + 1, s.c_str(), out.size());
    out.resize(n > 0 ? n - 1 : 0);  // mbstowcs_s includes null terminator in count
    return out;
}

bool WindowsSapiReader::initialize()
{
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    if (FAILED(hr) && hr != S_FALSE) {
        LOG_ERROR("CoInitializeEx failed (SAPI): 0x%08X", (unsigned)hr);
        return false;
    }

    hr = CoCreateInstance(CLSID_SpVoice, nullptr, CLSCTX_ALL,
                          IID_ISpVoice, reinterpret_cast<void**>(&voice_));
    if (FAILED(hr)) {
        LOG_ERROR("CoCreateInstance ISpVoice failed: 0x%08X", (unsigned)hr);
        return false;
    }

    LOG_INFO("Windows SAPI TTS initialized");
    return true;
}

void WindowsSapiReader::speak(const PaceNote& note)
{
    if (!voice_) return;
    std::string text = renderNote(note);
    std::wstring wtext = toWide(text);
    // SPF_ASYNC: non-blocking; SPF_PURGEBEFORESPEAK: interrupt previous
    voice_->Speak(wtext.c_str(),
                  SPF_ASYNC | SPF_PURGEBEFORESPEAK,
                  nullptr);
    LOG_INFO("[Pacenote/SAPI] %s", text.c_str());
}

void WindowsSapiReader::shutdown()
{
    if (voice_) {
        voice_->Release();
        voice_ = nullptr;
    }
    CoUninitialize();
}

#endif // _WIN32
