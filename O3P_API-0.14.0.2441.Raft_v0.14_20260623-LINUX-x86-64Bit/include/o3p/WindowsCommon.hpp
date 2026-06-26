/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#ifndef WINDOWSCOMMON_HPP
#define WINDOWSCOMMON_HPP

#include <windows.h>
#include <cfgmgr32.h>
#include <setupapi.h>
#include <usbiodef.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

/**
 * @brief Convert a given wstring to std::string
 *
 * @param wstr The wstring that should be converted.
 *
 * @returns std::string result
 */
inline std::string wstringToUtf8(const std::wstring &wstr) {
    if (wstr.empty())
        return {};

    int sizeNeeded = WideCharToMultiByte(
        CP_UTF8, 0, wstr.data(), (int)wstr.size(), nullptr, 0, nullptr, nullptr);

    std::string result(sizeNeeded, 0);

    WideCharToMultiByte(
        CP_UTF8, 0, wstr.data(), (int)wstr.size(), &result[0], sizeNeeded, nullptr, nullptr);

    return result;
}

/**
 * @brief Format hex input
 *
 * @param val the hex input
 *
 * @returns std::string formatted input
 */
inline std::string formatHex(uint16_t val) {
    std::ostringstream oss;
    oss << std::hex << std::nouppercase << std::setfill('0') << std::setw(4) << val;
    return oss.str();
}

/**
 * @brief Get Device Model ID of device
 *
 * @param deviceId The device model id
 * @param vid The Vendor ID
 * @param pid The product ID
 * @param mi
 */
inline void GetDeviceModelId(const std::string &deviceId, uint32_t &vid, uint32_t &pid, uint32_t &mi) {
    const std::string vidPrefix = "vid_";
    const std::string pidPrefix = "pid_";
    const std::string miPrefix = "mi_";

    const size_t vidLocation = deviceId.find(vidPrefix);
    if (vidLocation == std::string::npos) {
        return;
    }
    const size_t pidLocation = deviceId.find(pidPrefix);
    if (pidLocation == std::string::npos) {
        return;
    }
    const size_t miLocation = deviceId.find(miPrefix);
    if (miLocation == std::string::npos) {
        return;
    }
    const std::string vidStr =
        deviceId.substr(vidLocation + vidPrefix.size(), 4);
    const std::string pidStr =
        deviceId.substr(pidLocation + pidPrefix.size(), 4);
    const std::string miStr =
        deviceId.substr(miLocation + miPrefix.size(), 2);

    {
        {
            std::stringstream ss;
            ss << std::hex << vidStr;
            ss >> vid;
        }

        {
            std::stringstream ss;
            ss << std::hex << pidStr;
            ss >> pid;
        }

        {
            std::stringstream ss;
            ss << std::dec << miStr;
            ss >> mi;
        }
    }
}

/**
 * @brief Converts LPCWSTR to std::String
 *
 * @param lpcwszStr The string that should be converted
 *
 * @return std:string version of lpcwszStr
 */
inline std::string ConvertLPCWSTRToString(LPCWSTR lpcwszStr) {
    const int strLength = WideCharToMultiByte(CP_UTF8, 0, lpcwszStr, -1,
                                              nullptr, 0, nullptr, nullptr);
    std::string str(strLength, 0);
    WideCharToMultiByte(CP_UTF8, 0, lpcwszStr, -1, &str[0],
                        strLength, nullptr, nullptr);
    return str;
}

/**
 * @brief Get the unique device path
 *
 * @param symbolicLink
 *
 * @returns std::string devicePath
 */
inline std::string GetUniqueDevicePath(const std::string &symbolicLink) {
    auto devicePath = symbolicLink;
    size_t hashLocation = symbolicLink.find("#");

    devicePath = devicePath.substr(hashLocation + 1);
    hashLocation = devicePath.find("#");
    devicePath = devicePath.substr(hashLocation + 1);
    hashLocation = devicePath.find("#");
    devicePath = devicePath.substr(0, hashLocation - 4);

    return devicePath;
}

/**
 * @brief Get Parent Device Serial
 *
 * @param deviceId DeviceID of parent device
 *
 * @returns std::string serial number
 */
inline std::string GetParentDeviceSerial(const std::string &deviceId) {
    HDEVINFO devInfoSet = SetupDiGetClassDevs(
        nullptr, nullptr, nullptr,
        DIGCF_ALLCLASSES | DIGCF_PRESENT);

    SP_DEVINFO_DATA devInfo = {};
    devInfo.cbSize = sizeof(devInfo);

    WCHAR instanceId[MAX_DEVICE_ID_LEN];

    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfoSet, i, &devInfo); ++i) {
        if (CM_Get_Device_IDW(devInfo.DevInst, instanceId, MAX_DEVICE_ID_LEN, 0) == CR_SUCCESS) {
            std::wstring instanceIdLower = instanceId;
            std::transform(instanceIdLower.begin(), instanceIdLower.end(), instanceIdLower.begin(), ::towlower);

            std::wstring deviceIdW(deviceId.begin(), deviceId.end());
            std::transform(deviceIdW.begin(), deviceIdW.end(), deviceIdW.begin(), ::towlower);

            if (instanceIdLower.find(deviceIdW) != std::wstring::npos) {
                DEVINST parentInst;
                if (CM_Get_Parent(&parentInst, devInfo.DevInst, 0) == CR_SUCCESS) {
                    WCHAR parentId[MAX_DEVICE_ID_LEN];
                    if (CM_Get_Device_IDW(parentInst, parentId, MAX_DEVICE_ID_LEN, 0) == CR_SUCCESS) {
                        SetupDiDestroyDeviceInfoList(devInfoSet);

                        std::wstring parentIdStr = parentId;

                        size_t first = parentIdStr.find(L'\\');
                        if (first == std::wstring::npos)
                            return "";

                        size_t second = parentIdStr.find(L'\\', first + 1);
                        if (second == std::wstring::npos)
                            return "";

                        std::wstring resultW = parentIdStr.substr(second + 1);
                        return wstringToUtf8(resultW);
                    }
                }
            }
        }
    }

    SetupDiDestroyDeviceInfoList(devInfoSet);
    return "";
}

static std::vector<std::string> GetMultiSzProperty(
    HDEVINFO hDevInfo,
    SP_DEVINFO_DATA& devInfo,
    DWORD prop)
{
    DWORD regType = 0, needed = 0;

    SetupDiGetDeviceRegistryPropertyA(
        hDevInfo, &devInfo, prop,
        &regType, nullptr, 0, &needed);

    if (!needed)
        return {};

    std::vector<BYTE> buffer(needed);

    if (!SetupDiGetDeviceRegistryPropertyA(
            hDevInfo, &devInfo, prop,
            &regType,
            buffer.data(),
            (DWORD)buffer.size(),
            nullptr))
        return {};

    std::vector<std::string> result;

    const char* p = (const char*)buffer.data();

    while (*p)
    {
        std::string s(p);
        result.push_back(s);
        p += s.size() + 1;
    }

    return result;
}

static bool ParseHexByteAfter(
    const std::string& s,
    const std::string& key,
    uint8_t& out)
{
    auto pos = s.find(key);
    if (pos == std::string::npos)
        return false;

    pos += key.size();

    if (pos + 2 > s.size())
        return false;

    std::string hex = s.substr(pos, 2);
    out = (uint8_t)strtoul(hex.c_str(), nullptr, 16);

    return true;
}

static bool ParseUsbClassFromIds(
    const std::vector<std::string>& ids, uint8_t &outClass, uint8_t &outSub)
{
    for (const auto& id : ids)
    {
        if (id.rfind("USB\\Class_", 0) != 0)
            continue;

        uint8_t cls, sub;
        bool okC = ParseHexByteAfter(id, "Class_", cls);
        bool okS = ParseHexByteAfter(id, "SubClass_", sub);

        if (okC && okS) {
            outClass = cls;
            outSub = sub;
            return true;
        }
    }

    return false;
}

static bool GetUsbClassFromDevicePath(
    const std::string& targetDevicePath, uint8_t& outClass, uint8_t& outSub, GUID ifGuid = GUID_DEVINTERFACE_USB_DEVICE)
{
    bool parsed = false;
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(
    &ifGuid,
    nullptr,
    nullptr,
    DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

    if (hDevInfo == INVALID_HANDLE_VALUE)
        return false;


    for (DWORD i = 0;; i++)
    {
        SP_DEVICE_INTERFACE_DATA ifData{};
        ifData.cbSize = sizeof(ifData);

        if (!SetupDiEnumDeviceInterfaces(
                hDevInfo,
                nullptr,
                &ifGuid,
                i,
                &ifData))
            break;

        DWORD needed = 0;

        SetupDiGetDeviceInterfaceDetailA(
            hDevInfo,
            &ifData,
            nullptr,
            0,
            &needed,
            nullptr);

        if (!needed)
            continue;

        std::vector<BYTE> buffer(needed);

        auto detail =
            (SP_DEVICE_INTERFACE_DETAIL_DATA_A*)buffer.data();

        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        SP_DEVINFO_DATA devInfo{};
        devInfo.cbSize = sizeof(devInfo);

        if (!SetupDiGetDeviceInterfaceDetailA(
                hDevInfo,
                &ifData,
                detail,
                needed,
                nullptr,
                &devInfo))
            continue;

        if (_stricmp(detail->DevicePath,
                     targetDevicePath.c_str()) != 0)
            continue;

        auto hw = GetMultiSzProperty(hDevInfo, devInfo,
                                     SPDRP_HARDWAREID);

        if (ParseUsbClassFromIds(hw, outClass, outSub))
        {
            parsed = true;
            break;
        }

        auto compat = GetMultiSzProperty(hDevInfo, devInfo,
                                         SPDRP_COMPATIBLEIDS);

        if (ParseUsbClassFromIds(compat, outClass, outSub))
        {
            parsed = true;
            break;
        }

        break;
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);

    return parsed;
}

#endif // WINDOWSCOMMON_HPP