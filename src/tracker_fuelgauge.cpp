/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Particle.h"
#include "tracker_fuelgauge.h"
#include "model_gauge.h"

using namespace particle::power;

const model_config_t model_config_lg18650_1S4P = {
    .EmptyAdjustment=0,
    .FullAdjustment=100,
    .RCOMP0 = 123,
    .TempCoUp = -0.0,
    .TempCoDown = -0.0,
    .OCVTest = 56176,
    .SOCCheckA = 225,
    .SOCCheckB = 227,
    .bits = 19,
    .model_data = {
        0x99, 0x20, 0xA6, 0xA0, 0xA9, 0x50, 0xAC, 0x40, 0xB0, 0x60, 0xB3, 0x20, 0xB4, 0xF0, 0xB7, 0x60,
        0xBB, 0xF0, 0xBE, 0xC0, 0xC2, 0x00, 0xC5, 0x50, 0xC8, 0xF0, 0xCB, 0x10, 0xCD, 0x10, 0xD1, 0x70,
        0x01, 0x20, 0x14, 0x40, 0x0A, 0xA0, 0x0C, 0x40, 0x1A, 0x00, 0x23, 0x20, 0x1D, 0xE0, 0x0F, 0xA0,
        0x0A, 0x60, 0x13, 0x80, 0x11, 0xE0, 0x0F, 0x00, 0x11, 0x40, 0x27, 0x80, 0x0A, 0xA0, 0x0A, 0xA0
    }
};


TrackerFuelGauge *TrackerFuelGauge::_instance = nullptr;
static ModelGauge model_gauge(model_config_lg18650_1S4P);

void TrackerFuelGauge::init()
{
    // load model config when power on
    model_gauge.load_config();

    /*
        Notify DVOS to switch SOC bits.
        DVOS 3.3.0 and above
    */
    auto cfg = System.getPowerConfiguration();
    //Log.info("## soc_bits == %d ##",cfg.socBitPrecision());
    if(model_config_lg18650_1S4P.bits == 19)
    {// 19 Bits
        cfg.socBitPrecision(SOC_19_BIT_PRECISION);
    }
    else
    {// 18 Bits
        cfg.socBitPrecision(DEFAULT_SOC_18_BIT_PRECISION);
    }
    System.setPowerConfiguration(cfg);

    verify_model();
}

void TrackerFuelGauge::verify_model()
{
    // verify model, reload model if verify failed
    auto ret = model_gauge.verify_model();
    verifyCount++;
    if (ModelGaugeStatus::NONE != ret) {
        verifyFail++;
    }
}

void TrackerFuelGauge::loop()
{
    // verify model every 1 hour
    if(System.uptime() - last_1h >= 3600)
    {
        last_1h = System.uptime();
        verify_model();
    }
#ifdef FUEL_GAUGE_TEST
    test();
#endif
}

/**
 * @brief get soc percentage
 * @return soc percentage value
 */
float TrackerFuelGauge::getSoC()
{
    return model_gauge.get_soc();
}

/**
 * @brief get battery voltage
 * @return voltage value
 */
float TrackerFuelGauge::getVolt()
{
    return model_gauge.get_volt();
}


#ifdef FUEL_GAUGE_TEST
int readPmicRegister(uint8_t address, uint8_t reg, uint8_t* val, int length) {
    Wire1.beginTransmission(address);
    Wire1.write(&reg, 1);
    CHECK_TRUE(Wire1.endTransmission(false) == 0, SYSTEM_ERROR_INTERNAL);

    auto remaining = std::min<int>(length, I2C_BUFFER_LENGTH);
    auto readLength = (int)Wire1.requestFrom((int)address, remaining);
    if (readLength != remaining) {
        Wire1.endTransmission();
        return SYSTEM_ERROR_INTERNAL;
    }

    while (Wire1.available() && remaining--) {
        *val++ = Wire1.read();
    }

    return SYSTEM_ERROR_NONE;
}

#include "temperature.h"
void TrackerFuelGauge::enable_publish_pmic_regs(bool enable)
{
    publishPMICRegs = enable;
}


void TrackerFuelGauge::test()
{
    static uint32_t publicInterval = 0;
    FuelGauge fuel;
    extern float stsTemperature;
    if (System.uptime() - publicInterval > 60 && Particle.connected())
    {
        float thTemperature = get_temperature();
        auto source = System.powerSource();
        int state = System.batteryState();
        float batterySocSystem = System.batteryCharge();
        const String szPowerSources[] = {
            "unknown", "vin", "usb host", "usb adapter",
            "usb otg", "battery"
        };
        const String szBatteryState[] =
        {
            "UNKNOWN","NOT_CHARGING","CHARGING","CHARGED","DISCHARGING","FAULT","DISCONNECTED"
        };
        if (state > BATTERY_STATE_DISCONNECTED)
        {
            state = BATTERY_STATE_UNKNOWN;
        }




        CloudService &cloud_service = CloudService::instance();
        cloud_service.beginCommand("FUEL_GUAGE_TEST");
        cloud_service.writer().name("FUEL_GUAGE_TEST").beginObject();
        cloud_service.writer().name("source").value(szPowerSources[source].c_str());
        cloud_service.writer().name("state").value(szBatteryState[state].c_str());
        cloud_service.writer().name("sys_soc").value(batterySocSystem,2);
        cloud_service.writer().name("fg_soc").value(fuel.getSoC(),2);
        cloud_service.writer().name("mg_soc").value(getSoC(),2);
        cloud_service.writer().name("voltage").value(getVolt(),3);
        cloud_service.writer().name("verify").value(verifyCount);
        cloud_service.writer().name("reload").value(verifyFail);
        cloud_service.writer().name("stemp").value((double)stsTemperature, 1);
        cloud_service.writer().name("ttemp").value((double)thTemperature, 1);
        if (publishPMICRegs)
        {
            uint8_t regs[11] {};
            {
                PMIC pmic(true);
                auto ret = readPmicRegister(0x6b, 0, regs, ARRAY_SIZE(regs));
            }
            auto powerConfig = System.getPowerConfiguration();
            cloud_service.writer().name("charge_current").value(powerConfig.batteryChargeCurrent());
            cloud_service.writer().name("regs").beginArray();
            for (int i = 0;i < ARRAY_SIZE(regs);i++) {
                cloud_service.writer().value((unsigned int)regs[i]);
            }
            cloud_service.writer().endArray();
        }


        cloud_service.writer().endObject();

        cloud_service.lock();
        int rval = cloud_service.send();
        cloud_service.unlock();
        //Log.info("%.*s", cloud_service.writer().dataSize(), cloud_service.writer().buffer());
        //Log.info("### Publish FuelGauge Data %s ###",rval == -EBUSY ? "FAIL" : "SUCCESS");

        publicInterval = System.uptime();
    }
}
#endif
