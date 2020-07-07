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

#include "tracker_rgb.h"
#include "tracker_cellular.h"

#define RGB_CONTROL_TIMER_PERIOD_MS (250)
#define RGB_CONTROL_FAST_FADE_PERIOD_MS (500)
#define RGB_CONTROL_SLOW_FADE_PERIOD_MS (1000)

// on a 0%-100% scale to mark transition betweeen merely OK to GOOD signal
#define RGB_CONTROL_CELL_STRENGTH_GOOD (70)

TrackerRGB *TrackerRGB::_instance = nullptr;
static LEDStatus ledStatus(RGB_COLOR_RED, LED_PATTERN_SOLID, LED_SPEED_NORMAL, LED_PRIORITY_CRITICAL);
static Timer * rgb_control_timer;

static struct {
    RGBControlType type;
    struct {
        int32_t brightness;
        int32_t red;
        int32_t green;
        int32_t blue;
    } direct;
} rgb_config = {
    .type = RGBControlType::APP_PARTICLE,
    .direct = {
        .brightness = 255,
        .red = 0,
        .green = 0,
        .blue = 255,
    }
};

// actual led control driven by a periodic timer
static void rgb_control_timer_cb()
{
    switch(rgb_config.type)
    {
        case RGBControlType::APP_DIRECT:
        {
            RGB.brightness(rgb_config.direct.brightness);
            RGB.color(rgb_config.direct.red,
                rgb_config.direct.green,
                rgb_config.direct.blue);
            break;
        }
        case RGBControlType::APP_TRACKER: // fall-thru
        case RGBControlType::APP_GRADIENT:
        {
            CellularSignal signal;

            if(TrackerCellular::instance().getSignal(signal))
            {
                // not connected to cell
                // error or not recent enough signal (which is also error)
                ledStatus.setPattern(LED_PATTERN_FADE);
                ledStatus.setPeriod(RGB_CONTROL_FAST_FADE_PERIOD_MS);
                ledStatus.setColor(RGB_COLOR_RED);
            }
            else
            {
                // connected to cell (recent enough signal)
                ledStatus.setPattern(Particle.connected() ? LED_PATTERN_SOLID : LED_PATTERN_FADE);
                ledStatus.setPeriod(RGB_CONTROL_SLOW_FADE_PERIOD_MS);

                if(signal.getStrength() < RGB_CONTROL_CELL_STRENGTH_GOOD)
                {
                    ledStatus.setColor(RGB_COLOR_YELLOW);
                }
                else
                {
                    ledStatus.setColor(RGB_COLOR_GREEN);
                }
            }
        }
        default:
            break;
    }
}

// get callback for config management
static int rgb_control_get_type_cb(int32_t &value, const void *context)
{
    value = (int32_t) TrackerRGB::getType();
    return 0;
}

// set callback for config management
static int rgb_control_set_type_cb(int32_t value, const void *context)
{
    return TrackerRGB::setType((RGBControlType) value);
}

void TrackerRGB::init()
{
    rgb_control_timer = new Timer(RGB_CONTROL_TIMER_PERIOD_MS, rgb_control_timer_cb);
    setType(rgb_config.type);

    static ConfigObject rgb_control_desc("rgb", {
        ConfigStringEnum(
            "type",
            {
                {"particle", (int32_t) RGBControlType::APP_PARTICLE},
                {"off", (int32_t) RGBControlType::APP_OFF},
                {"tracker", (int32_t) RGBControlType::APP_TRACKER},
                {"direct", (int32_t) RGBControlType::APP_DIRECT},
                {"gradient", (int32_t) RGBControlType::APP_GRADIENT},
            },
            rgb_control_get_type_cb,
            rgb_control_set_type_cb
        ),
        ConfigObject("direct", {
            ConfigInt("brightness", &rgb_config.direct.brightness, 0, 255),
            ConfigInt("red", &rgb_config.direct.red, 0, 255),
            ConfigInt("green", &rgb_config.direct.green, 0, 255),
            ConfigInt("blue", &rgb_config.direct.blue, 0, 255),
        })
    });
    ConfigService::instance().registerModule(rgb_control_desc);

    rgb_control_timer->start();
}

int TrackerRGB::setType(RGBControlType type)
{
    switch(type)
    {
        case RGBControlType::APP_PARTICLE:
            ledStatus.setActive(false);
            if(rgb_config.type != RGBControlType::APP_PARTICLE)
            {
                RGB.brightness(255);
                RGB.control(false);
            }
            break;
        case RGBControlType::APP_OFF:
            ledStatus.setActive(false);
            RGB.control(true);
            RGB.brightness(0);
            break;
        case RGBControlType::APP_TRACKER: // fall-thru
        case RGBControlType::APP_GRADIENT: // fall-thru
        case RGBControlType::APP_DIRECT:
            RGB.brightness(255);
            ledStatus.setActive(type != RGBControlType::APP_DIRECT);
            RGB.control(type == RGBControlType::APP_DIRECT);
            break;
        default:
            return -EINVAL;
    }
    rgb_config.type = type;

    return 0;
}

RGBControlType TrackerRGB::getType()
{
    return rgb_config.type;
}
