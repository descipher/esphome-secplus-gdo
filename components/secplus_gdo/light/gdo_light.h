/*
 * Copyright (C) 2024  Konnected Inc.
 * Copyright (C) 2024  Gelidus Research
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "esphome/components/light/light_output.h"
#include "esphome/core/component.h"
#include "include/gdo.h"


namespace esphome {
namespace secplus_gdo {

class GDOLight : public light::LightOutput, public Component {
public:
  void setup_state(light::LightState *state) override { this->state_ = state; }
  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::ON_OFF});
    return traits;
  }
  void write_state(light::LightState *state) override {
    if (!this->synced_) {
      return;
    }
    bool binary;
    state->current_values_as_binary(&binary);

    // Use a simple timeout to defer the call instead of defer()
    // This ensures we don't block the web server thread
    this->set_timeout("light_action", 1, [binary]() {
      if (binary) {
        gdo_light_on();
      } else {
        gdo_light_off();
      }
    });
  }

  void set_state(gdo_light_state_t state) {
    if (state == this->light_state_) {
      return;
    }

    this->light_state_ = state;
    ESP_LOGI(TAG, "Light state: %s", gdo_light_state_to_string(state));
    bool is_on = state == GDO_LIGHT_STATE_ON;
    this->state_->current_values.set_state(is_on);
    this->state_->remote_values.set_state(is_on);
    this->state_->publish_state();
  }

  void set_sync_state(bool synced) { this->synced_ = synced; }

private:
  light::LightState *state_{nullptr};
  gdo_light_state_t light_state_{GDO_LIGHT_STATE_MAX};
  bool synced_{false};
  static constexpr auto TAG{"GDOLight"};
}; // GDOLight
} // namespace secplus_gdo
} // namespace esphome
