/*
// Copyright (c) 2020 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "cpuinfo.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include <optional>
#include <sstream>
#include <string>

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

#include <peci.h>

#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>

namespace phosphor
{
namespace cpu_info
{

static constexpr const char* cpuPath =
    "/xyz/openbmc_project/inventory/system/chassis/motherboard/cpu";

struct ProcessorInfo
{
    uint64_t ppin;
    std::string sspec;
};

using CPUMap =
    boost::container::flat_map<size_t,
                               std::pair<int, std::shared_ptr<CPUInfo>>>;

static CPUMap cpuMap = {};

static std::optional<std::string> readSSpec(uint8_t bus, uint8_t slaveAddr,
                                            uint8_t regAddr, size_t count)
{
    unsigned long funcs = 0;
    std::string devPath = "/dev/i2c-" + std::to_string(bus);

    int fd = ::open(devPath.c_str(), O_RDWR);
    if (fd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error in open!",
            phosphor::logging::entry("PATH=%s", devPath.c_str()),
            phosphor::logging::entry("SLAVEADDR=0x%x", slaveAddr));
        return std::nullopt;
    }

    if (::ioctl(fd, I2C_FUNCS, &funcs) < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error in I2C_FUNCS!",
            phosphor::logging::entry("PATH=%s", devPath.c_str()),
            phosphor::logging::entry("SLAVEADDR=0x%x", slaveAddr));
        ::close(fd);
        return std::nullopt;
    }

    if (!(funcs & I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "i2c bus does not support read!",
            phosphor::logging::entry("PATH=%s", devPath.c_str()),
            phosphor::logging::entry("SLAVEADDR=0x%x", slaveAddr));
        ::close(fd);
        return std::nullopt;
    }

    if (::ioctl(fd, I2C_SLAVE_FORCE, slaveAddr) < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error in I2C_SLAVE_FORCE!",
            phosphor::logging::entry("PATH=%s", devPath.c_str()),
            phosphor::logging::entry("SLAVEADDR=0x%x", slaveAddr));
        ::close(fd);
        return std::nullopt;
    }

    int value = 0;
    std::string sspec;
    sspec.reserve(count);

    for (size_t i = 0; i < count; i++)
    {
        value = ::i2c_smbus_read_byte_data(fd, regAddr++);
        if (value < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error in i2c read!",
                phosphor::logging::entry("PATH=%s", devPath.c_str()),
                phosphor::logging::entry("SLAVEADDR=0x%x", slaveAddr));
            ::close(fd);
            return std::nullopt;
        }
        if (!std::isprint(static_cast<unsigned char>(value)))
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Non printable value in sspec, ignored.");
            continue;
        }
        sspec.push_back(static_cast<unsigned char>(value));
    }
    ::close(fd);
    return sspec;
}

// PECI Client Address Map
static void getPECIAddrMap(CPUMap& cpuMap)
{
    int idx = 0;
    for (size_t i = MIN_CLIENT_ADDR; i <= MAX_CLIENT_ADDR; i++)
    {
        if (peci_Ping(i) == PECI_CC_SUCCESS)
        {
            cpuMap.emplace(std::make_pair(i, std::make_pair(idx, nullptr)));
            idx++;
        }
    }
}

static std::shared_ptr<CPUInfo>
    createCPUInfo(std::shared_ptr<sdbusplus::asio::connection>& conn,
                  const int& cpu)
{
    std::string path = cpuPath + std::to_string(cpu);
    std::shared_ptr<CPUInfo> cpuInfo = std::make_shared<CPUInfo>(
        static_cast<sdbusplus::bus::bus&>(*conn), path);
    return cpuInfo;
}

// constants for reading QDF string from PIROM
// Currently, they are the same for platforms with icx
// \todo: move into configuration file to be more robust
static constexpr uint8_t i2cBus = 13;
static constexpr uint8_t slaveAddr0 = 0x50;
static constexpr uint8_t regAddr = 0xf;
static constexpr uint8_t sspecSize = 4;

static void updateSerialNumber(boost::asio::steady_timer& peciWaitTimer,
                               const uint8_t clientAddr,
                               std::shared_ptr<CPUInfo>& cpuInfo,
                               uint8_t retryCount)
{
    // get processor ID
    static constexpr uint8_t u8Size = 4; // default to a DWORD
    static constexpr uint8_t u8PPINPkgIndex = 19;
    static constexpr uint16_t u16PPINPkgParamHigh = 2;
    static constexpr uint16_t u16PPINPkgParamLow = 1;
    uint32_t u32PkgValueLow = 0;
    uint32_t u32PkgValueHigh = 0;
    uint8_t cc = 0;

    if (PECI_CC_SUCCESS != peci_RdPkgConfig(clientAddr, u8PPINPkgIndex,
                                            u16PPINPkgParamLow, u8Size,
                                            (uint8_t*)&u32PkgValueLow, &cc))
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "peci read package config failed at address",
            phosphor::logging::entry("PECIADDR=0x%x", clientAddr),
            phosphor::logging::entry("CC=0x%x", cc));
        u32PkgValueLow = 0;
    }

    if (PECI_CC_SUCCESS != peci_RdPkgConfig(clientAddr, u8PPINPkgIndex,
                                            u16PPINPkgParamHigh, u8Size,
                                            (uint8_t*)&u32PkgValueHigh, &cc))
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "peci read package config failed at address",
            phosphor::logging::entry("PECIADDR=0x%x", clientAddr),
            phosphor::logging::entry("CC=0x%x", cc));
        u32PkgValueHigh = 0;
    }

    // If cpuPPIN is valid, set serial number and exit.
    if (0 != u32PkgValueLow && 0 != u32PkgValueHigh)
    {
        uint64_t cpuPPIN =
            u32PkgValueLow | (static_cast<uint64_t>(u32PkgValueHigh) << 32);
        cpuPPIN = u32PkgValueLow;
        cpuPPIN |= static_cast<uint64_t>(u32PkgValueHigh) << 32;
        std::stringstream stream;
        stream << std::hex << cpuPPIN;
        std::string serialNumber(stream.str());
        cpuInfo->serialNumber(serialNumber);
        return;
    }

    // If PPIN is not valid, Retry for 3 times with 60 seconds delay.
    if (0 == retryCount)
    {
        return;
    }

    retryCount--;
    peciWaitTimer.expires_after(
        std::chrono::seconds(6 * phosphor::cpu_info::peciCheckInterval));
    peciWaitTimer.async_wait([&peciWaitTimer, clientAddr(std::move(clientAddr)),
                              &cpuInfo, retryCount(std::move(retryCount))](
                                 const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled
            // before completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    " async_wait to Get CPU PPIN failed.",
                    phosphor::logging::entry("EC=0x%x", ec.value()));
            }
            return;
        }
        phosphor::cpu_info::updateSerialNumber(peciWaitTimer, clientAddr,
                                               cpuInfo, retryCount);
    });
}

static void getProcessorInfo(boost::asio::steady_timer& peciWaitTimer,
                             std::shared_ptr<sdbusplus::asio::connection>& conn,
                             sdbusplus::asio::object_server& objServer,
                             CPUMap& cpuMap)
{

    for (auto& cpu : cpuMap)
    {
        uint8_t cc = 0;
        CPUModel model{};
        uint8_t stepping = 0;
        uint8_t retryCount = 3;

        std::shared_ptr<CPUInfo> cpuInfo =
            createCPUInfo(conn, cpu.second.first);
        cpu.second.second = cpuInfo;

        if (peci_GetCPUID(cpu.first, &model, &stepping, &cc) != PECI_CC_SUCCESS)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Cannot get CPUID!",
                phosphor::logging::entry("PECIADDR=0x%x", cpu.first));
            continue;
        }

        switch (model)
        {
            case icx:
            {
                updateSerialNumber(peciWaitTimer, cpu.first, cpu.second.second,
                                   retryCount);

                // assuming the slaveAddress will be incrementing like peci
                // client address
                std::optional<std::string> sspec = readSSpec(
                    i2cBus, static_cast<uint8_t>(slaveAddr0 + cpu.second.first),
                    regAddr, sspecSize);
                cpuInfo->model(sspec.value_or(""));
                break;
            }
            default:
                phosphor::logging::log<phosphor::logging::level::INFO>(
                    "in-compatible cpu for cpu asset info");
                break;
        }
    }
}

static bool isPECIAvailable(void)
{
    for (size_t i = MIN_CLIENT_ADDR; i <= MAX_CLIENT_ADDR; i++)
    {
        if (peci_Ping(i) == PECI_CC_SUCCESS)
        {
            return true;
        }
    }
    return false;
}

static void
    peciAvailableCheck(boost::asio::steady_timer& peciWaitTimer,
                       boost::asio::io_service& io,
                       std::shared_ptr<sdbusplus::asio::connection>& conn,
                       sdbusplus::asio::object_server& objServer)
{
    bool peciAvailable = isPECIAvailable();
    if (peciAvailable)
    {
        // get the PECI client address list
        getPECIAddrMap(cpuMap);
        getProcessorInfo(peciWaitTimer, conn, objServer, cpuMap);
    }
    if (!peciAvailable || !cpuMap.size())
    {
        peciWaitTimer.expires_after(
            std::chrono::seconds(6 * peciCheckInterval));
        peciWaitTimer.async_wait([&peciWaitTimer, &io, &conn, &objServer](
                                     const boost::system::error_code& ec) {
            if (ec)
            {
                // operation_aborted is expected if timer is canceled
                // before completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "PECI Available Check async_wait failed",
                        phosphor::logging::entry("EC=0x%x", ec.value()));
                }
                return;
            }
            peciAvailableCheck(peciWaitTimer, io, conn, objServer);
        });
    }
}

} // namespace cpu_info
} // namespace phosphor

int main(int argc, char* argv[])
{
    // setup connection to dbus
    boost::asio::io_service io;
    std::shared_ptr<sdbusplus::asio::connection> conn =
        std::make_shared<sdbusplus::asio::connection>(io);

    // CPUInfo Object
    conn->request_name(phosphor::cpu_info::cpuInfoObject);
    sdbusplus::asio::object_server server =
        sdbusplus::asio::object_server(conn);
    sdbusplus::bus::bus& bus = static_cast<sdbusplus::bus::bus&>(*conn);
    sdbusplus::server::manager::manager objManager(
        bus, "/xyz/openbmc_project/inventory");

    // Start the PECI check loop
    boost::asio::steady_timer peciWaitTimer(
        io, std::chrono::seconds(phosphor::cpu_info::peciCheckInterval));
    peciWaitTimer.async_wait([&peciWaitTimer, &io, &conn,
                              &server](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled
            // before completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "PECI Available Check async_wait failed ",
                    phosphor::logging::entry("EC=0x%x", ec.value()));
            }
            return;
        }
        phosphor::cpu_info::peciAvailableCheck(peciWaitTimer, io, conn, server);
    });

    io.run();

    return 0;
}
