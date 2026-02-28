#include "sgp4x.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>

namespace esphome {
namespace sgp4x {

static const char *const TAG = "sgp4x";

void SGP4xComponent::setup() {
  // Serial Number identification
  uint16_t raw_serial_number[3];
  if (!this->get_register(SGP4X_CMD_GET_SERIAL_ID, raw_serial_number, 3, 1)) {
    ESP_LOGE(TAG, "Get serial number failed");
    this->error_code_ = SERIAL_NUMBER_IDENTIFICATION_FAILED;
    this->mark_failed();
    return;
  }
  this->serial_number_ = (uint64_t(raw_serial_number[0]) << 24) | (uint64_t(raw_serial_number[1]) << 16) |
                         (uint64_t(raw_serial_number[2]));
  ESP_LOGD(TAG, "Serial number: %" PRIu64, this->serial_number_);

  // Featureset identification for future use
  uint16_t featureset;
  if (!this->get_register(SGP4X_CMD_GET_FEATURESET, featureset, 1)) {
    ESP_LOGD(TAG, "Get feature set failed");
    this->mark_failed();
    return;
  }
  featureset &= 0x1FF;
  if (featureset == SGP40_FEATURESET) {
    this->sgp_type_ = SGP40;
    this->self_test_time_ = SPG40_SELFTEST_TIME;
    this->measure_time_ = SGP40_MEASURE_TIME;
    if (this->nox_sensor_) {
      ESP_LOGE(TAG, "SGP41 required for NOx");
      this->nox_sensor_->set_disabled_by_default(true);
      this->nox_sensor_->set_internal(true);
      this->nox_sensor_->state = NAN;
      this->nox_sensor_ = nullptr;
    }
  } else if (featureset == SGP41_FEATURESET) {
    this->sgp_type_ = SGP41;
    this->self_test_time_ = SPG41_SELFTEST_TIME;
    this->measure_time_ = SGP41_MEASURE_TIME;
  } else {
    ESP_LOGD(TAG, "Unknown feature set 0x%0X", featureset);
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Version 0x%0X", featureset);

  if (this->store_baseline_) {
    uint32_t hash = fnv1a_hash_extend(App.get_config_version_hash(), this->serial_number_);
    this->pref_ = global_preferences->make_preference<SGP4xBaselines>(hash, true);

    if (this->pref_.load(&this->voc_baselines_storage_)) {
      this->voc_state0_ = this->voc_baselines_storage_.state0;
      this->voc_state1_ = this->voc_baselines_storage_.state1;
      ESP_LOGV(TAG, "Loaded VOC baseline state0: 0x%04" PRIX32 ", state1: 0x%04" PRIX32,
               this->voc_baselines_storage_.state0, voc_baselines_storage_.state1);
    }

    this->seconds_since_last_store_ = 0;

    if (this->voc_baselines_storage_.state0 > 0 && this->voc_baselines_storage_.state1 > 0) {
      ESP_LOGV(TAG, "Setting VOC baseline from save state0: 0x%04" PRIX32 ", state1: 0x%04" PRIX32,
               this->voc_baselines_storage_.state0, voc_baselines_storage_.state1);
      voc_algorithm_.set_states(this->voc_baselines_storage_.state0, this->voc_baselines_storage_.state1);
    } else if (this->voc_baseline_ != 0) {
      voc_algorithm_.set_states(this->voc_baseline_, this->voc_baseline_);
      ESP_LOGV(TAG, "Setting VOC baseline from config: 0x%04" PRIX16, this->voc_baseline_);
    }
  }
  if (this->voc_sensor_ && this->voc_tuning_params_.has_value()) {
    voc_algorithm_.set_tuning_parameters(
        voc_tuning_params_.value().index_offset, voc_tuning_params_.value().learning_time_offset_hours,
        voc_tuning_params_.value().learning_time_gain_hours, voc_tuning_params_.value().gating_max_duration_minutes,
        voc_tuning_params_.value().std_initial, voc_tuning_params_.value().gain_factor);
  }

  if (this->nox_sensor_ && this->nox_tuning_params_.has_value()) {
    nox_algorithm_.set_tuning_parameters(
        nox_tuning_params_.value().index_offset, nox_tuning_params_.value().learning_time_offset_hours,
        nox_tuning_params_.value().learning_time_gain_hours, nox_tuning_params_.value().gating_max_duration_minutes,
        nox_tuning_params_.value().std_initial, nox_tuning_params_.value().gain_factor);
  }

  this->self_test_();

  ESP_LOGV(TAG, "Component requires sampling of 1Hz, setting up background sampler");
  this->set_interval(1000, [this]() { this->take_sample(); });

  this->nox_conditioning_start_ = millis();

  // If heater should start disabled, ensure we send heater-off command
  if (!this->heater_enabled_) {
    this->heater_off_();
  }

  if (this->power_switch_ != nullptr) {
    this->power_switch_->publish_state(this->heater_enabled_);
  }
}

void SGP4xComponent::self_test_() {
  ESP_LOGD(TAG, "Starting self-test");
  if (!this->write_command(SGP4X_CMD_SELF_TEST)) {
    this->error_code_ = COMMUNICATION_FAILED;
    ESP_LOGD(TAG, ESP_LOG_MSG_COMM_FAIL);
    this->mark_failed();
  }

  this->set_timeout(this->self_test_time_, [this]() {
    uint16_t reply = 0;
    if (!this->read_data(reply) || (reply != 0xD400)) {
      this->error_code_ = SELF_TEST_FAILED;
      ESP_LOGW(TAG, "Self-test failed (0x%X)", reply);
      this->mark_failed();
      return;
    }

    this->self_test_complete_ = true;
    ESP_LOGD(TAG, "Self-test complete");
  });
}

void SGP4xComponent::reset_algorithms_() {
  this->voc_algorithm_.reset();
  this->nox_algorithm_.reset();
  this->samples_read_ = 0;
  this->seconds_since_last_store_ = 0;
  this->voc_index_ = 0;
  this->nox_index_ = 0;
  this->voc_state0_ = 0;
  this->voc_state1_ = 0;
  this->nox_conditioning_start_ = millis();
}

void SGP4xComponent::heater_off_() {
  // Datasheet command: 0x36 0x15
  if (!this->write_command(SGP4X_CMD_TURN_HEATER_OFF)) {
    ESP_LOGW(TAG, "Heater off command failed (%d)", this->last_error_);
    this->status_set_warning(LOG_STR("heater off command failed"));
  } else {
    this->status_clear_warning();
    ESP_LOGD(TAG, "Heater turned off, sensor idle");
  }
}

void SGP4xComponent::set_heater_enabled(bool enabled) {
  if (enabled == this->heater_enabled_)
    return;

  this->heater_enabled_ = enabled;

  if (this->power_switch_ != nullptr) {
    this->power_switch_->publish_state(enabled);
  }

  if (enabled) {
    this->reset_algorithms_();
    ESP_LOGI(TAG, "Heater enabled; algorithms reset and re-stabilizing");
  } else {
    this->heater_off_();
    ESP_LOGI(TAG, "Heater disabled; sampling paused and heater off command sent");
  }
}

void SGP4xComponent::update_gas_indices_() {
  this->voc_index_ = this->voc_algorithm_.process(this->voc_sraw_);
  if (this->nox_sensor_ != nullptr)
    this->nox_index_ = this->nox_algorithm_.process(this->nox_sraw_);
  ESP_LOGV(TAG, "VOC: %" PRId32 ", NOx: %" PRId32, this->voc_index_, this->nox_index_);
  if (this->store_baseline_ && this->seconds_since_last_store_ > SHORTEST_BASELINE_STORE_INTERVAL) {
    this->voc_algorithm_.get_states(this->voc_state0_, this->voc_state1_);
    if (std::abs(this->voc_baselines_storage_.state0 - this->voc_state0_) > MAXIMUM_STORAGE_DIFF ||
        std::abs(this->voc_baselines_storage_.state1 - this->voc_state1_) > MAXIMUM_STORAGE_DIFF) {
      this->seconds_since_last_store_ = 0;
      this->voc_baselines_storage_.state0 = this->voc_state0_;
      this->voc_baselines_storage_.state1 = this->voc_state1_;

      if (this->pref_.save(&this->voc_baselines_storage_)) {
        ESP_LOGV(TAG, "Stored VOC baseline state0: 0x%04" PRIX32 ", state1: 0x%04" PRIX32,
                 this->voc_baselines_storage_.state0, this->voc_baselines_storage_.state1);
      } else {
        ESP_LOGW(TAG, "Storing VOC baselines failed");
      }
    }
  }

  if (this->samples_read_ < this->samples_to_stabilize_) {
    this->samples_read_++;
    ESP_LOGD(TAG, "Stabilizing (%d/%d); VOC index: %" PRIu32, this->samples_read_, this->samples_to_stabilize_,
             this->voc_index_);
  }
}

void SGP4xComponent::measure_raw_() {
  if (!this->heater_enabled_) {
    return;
  }

  float humidity = NAN;

  if (!this->self_test_complete_) {
    ESP_LOGW(TAG, "Self-test incomplete");
    return;
  }
  if (this->humidity_sensor_ != nullptr) {
    humidity = this->humidity_sensor_->state;
  }
  if (std::isnan(humidity) || humidity < 0.0f || humidity > 100.0f) {
    humidity = 50;
  }

  float temperature = NAN;
  if (this->temperature_sensor_ != nullptr) {
    temperature = float(this->temperature_sensor_->state);
  }
  if (std::isnan(temperature) || temperature < -40.0f || temperature > 85.0f) {
    temperature = 25;
  }

  uint16_t command;
  uint16_t data[2];
  size_t response_words;
  if (nox_sensor_ == nullptr) {
    command = SGP40_CMD_MEASURE_RAW;
    response_words = 1;
  } else {
    if (this->nox_conditioning_start_ == 0) {
      this->nox_conditioning_start_ = millis();
    }
    if (millis() - this->nox_conditioning_start_ < 10000) {
      command = SGP41_CMD_NOX_CONDITIONING;
      response_words = 1;
    } else {
      command = SGP41_CMD_MEASURE_RAW;
      response_words = 2;
    }
  }
  uint16_t rhticks = llround((uint16_t) ((humidity * 65535) / 100));
  uint16_t tempticks = (uint16_t) (((temperature + 45) * 65535) / 175);
  data[0] = rhticks;
  data[1] = tempticks;

  if (!this->write_command(command, data, 2)) {
    ESP_LOGD(TAG, "write error (%d)", this->last_error_);
    this->status_set_warning(LOG_STR("measurement request failed"));
    return;
  }

  this->set_timeout(this->measure_time_, [this, response_words]() {
    uint16_t raw_data[2];
    raw_data[1] = 0;
    if (!this->read_data(raw_data, response_words)) {
      ESP_LOGD(TAG, "read error (%d)", this->last_error_);
      this->status_set_warning(LOG_STR("measurement read failed"));
      this->voc_index_ = this->nox_index_ = UINT16_MAX;
      return;
    }
    this->voc_sraw_ = raw_data[0];
    this->nox_sraw_ = raw_data[1];
    this->status_clear_warning();
    this->update_gas_indices_();
  });
}

void SGP4xComponent::take_sample() {
  if (!this->self_test_complete_)
    return;
  if (!this->heater_enabled_)
    return;

  this->seconds_since_last_store_ += 1;
  this->measure_raw_();
}

void SGP4xComponent::update() {
  if (this->samples_read_ < this->samples_to_stabilize_) {
    return;
  }
  if (this->voc_sensor_ != nullptr) {
    if (this->voc_index_ != UINT16_MAX)
      this->voc_sensor_->publish_state(this->voc_index_);
  }
  if (this->nox_sensor_ != nullptr) {
    if (this->nox_index_ != UINT16_MAX)
      this->nox_sensor_->publish_state(this->nox_index_);
  }
}

void SGP4xComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SGP4x:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Store baseline: %s", YESNO(this->store_baseline_));
  ESP_LOGCONFIG(TAG, "  Heater enabled: %s", YESNO(this->heater_enabled_));

  if (this->is_failed()) {
    switch (this->error_code_) {
      case COMMUNICATION_FAILED:
        ESP_LOGW(TAG, ESP_LOG_MSG_COMM_FAIL);
        break;
      case SERIAL_NUMBER_IDENTIFICATION_FAILED:
        ESP_LOGW(TAG, "Get serial number failed");
        break;
      case SELF_TEST_FAILED:
        ESP_LOGW(TAG, "Self-test failed");
        break;
      default:
        ESP_LOGW(TAG, "Unknown error");
        break;
    }
  } else {
    ESP_LOGCONFIG(TAG,
                  "  Type: %s\n"
                  "  Serial number: %" PRIu64 "\n"
                  "  Minimum Samples: %f",
                  sgp_type_ == SGP41 ? "SGP41" : "SPG40", this->serial_number_, GasIndexAlgorithm_INITIAL_BLACKOUT);
  }
  LOG_UPDATE_INTERVAL(this);

  ESP_LOGCONFIG(TAG, "  Compensation:");
  if (this->humidity_sensor_ != nullptr || this->temperature_sensor_ != nullptr) {
    LOG_SENSOR("    ", "Temperature Source:", this->temperature_sensor_);
    LOG_SENSOR("    ", "Humidity Source:", this->humidity_sensor_);
  } else {
    ESP_LOGCONFIG(TAG, "    No source configured");
  }
  LOG_SENSOR("  ", "VOC", this->voc_sensor_);
  LOG_SENSOR("  ", "NOx", this->nox_sensor_);
  LOG_SWITCH("  ", "Power Switch", this->power_switch_);
}

void SGP4xHeaterSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_heater_enabled(state);
  }
  this->publish_state(state);
}

}  // namespace sgp4x
}  // namespace esphome