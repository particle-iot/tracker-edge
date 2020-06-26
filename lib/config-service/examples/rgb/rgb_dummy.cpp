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

#include "config_service.h"

int set_brightness_cb(const char *name, config_node_type_t node_type, int32_t value, const void *context)
{
    if(value < 0 || value > 100)
    {
        return -EINVAL;
    }
    *(int32_t *)context = value;
    return 0;
}

int set_color_cb(const char *name, config_node_type_t node_type, int32_t value, const void *context)
{
    if(value < 0 || value > 255)
    {
        return -EINVAL;
    }
    *(int32_t *)context = value;
    return 0;
}

struct {
    int32_t brightness;
    int32_t red;
    int32_t green;
    int32_t blue;
} rgb_config = {100,0,0,255};

uint32_t calc_rgb_crc()
{
    return HAL_Core_Compute_CRC32((uint8_t *) &rgb_config, sizeof(rgb_config));
}

auto config_brightness = ConfigInt("brightness", config_get_int32_cb, config_set_int32_cb, &rgb_config.brightness, 0, 100);
auto config_red = ConfigInt("red", config_get_int32_cb, config_set_int32_cb, &rgb_config.red, 0, 255);
auto config_green = ConfigInt("green", config_get_int32_cb, config_set_int32_cb, &rgb_config.green, 0, 255);
auto config_blue = ConfigInt("blue", config_get_int32_cb, config_set_int32_cb, &rgb_config.blue, 0, 255);

ConfigNode *rgb_children[] = {
    &config_brightness,
    &config_red,
    &config_green,
    &config_blue,
    nullptr,
};

ConfigObject rgb_desc("rgb", rgb_children);

void rgb_dummy_init(ConfigService *config_service)
{
    config_service->register_module(rgb_desc, calc_rgb_crc);
}

void rgb_dummy_tick()
{
    static uint32_t last_sec = System.uptime();

    if(last_sec != System.uptime())
    {
        last_sec = System.uptime();

        // let Particle RGB have control when not connected
        if(!Particle.connected())
        {
            RGB.control(false);
        }
        else
        {
            RGB.control(true);
            RGB.brightness(rgb_config.brightness);
            RGB.color(rgb_config.red, rgb_config.green, rgb_config.blue);
        }
    }
}
