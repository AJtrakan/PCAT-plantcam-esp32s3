#pragma once

#include <Arduino.h>

// Set to 0 for a photo-only build without the Edge Impulse library.
#define CANOPY_AI_ENABLED 1

#if CANOPY_AI_ENABLED
#include <Trakan-project-1_inferencing.h>
#if EI_CLASSIFIER_OBJECT_DETECTION != 1
#error "The canopy locator must be an Edge Impulse object-detection model (FOMO)"
#endif
static_assert(EI_CLASSIFIER_INPUT_WIDTH == 96 &&
                  EI_CLASSIFIER_INPUT_HEIGHT == 96,
              "This adapter expects a 96x96 Squash model");
#endif

static const char *canopyAIState =
    CANOPY_AI_ENABLED ? "ready" : "model_not_installed";
static uint32_t canopyAIElapsedMs = 0;

#if CANOPY_AI_ENABLED
static uint8_t *canopyAISource = nullptr;

static int canopyAIGetData(size_t offset, size_t length, float *output) {
  if (!canopyAISource || offset > 96 * 96 || length > 96 * 96 - offset)
    return -1;
  for (size_t i = 0; i < length; ++i) {
    size_t n = offset + i;
    size_t x = ((n % 96) * 320) / 96;
    size_t y = ((n / 96) * 240) / 96;
    size_t pixel = (y * 320 + x) * 3;
    // fmt2rgb888 from esp32-camera supplies BGR bytes on this core. Edge
    // Impulse expects a packed RGB value.
    output[i] = (canopyAISource[pixel + 2] << 16) |
                (canopyAISource[pixel + 1] << 8) |
                canopyAISource[pixel];
  }
  return 0;
}
#endif

static bool findPlantWithAI(uint8_t *pixels, int &centerX, int &centerY,
                            float &confidence) {
  canopyAIElapsedMs = 0;
  canopyAIState = "model_not_installed";
  centerX = -1;
  centerY = -1;
  confidence = 0;
#if CANOPY_AI_ENABLED
  canopyAISource = pixels;
  ei::signal_t signal;
  signal.total_length = 96 * 96;
  signal.get_data = canopyAIGetData;
  ei_impulse_result_t result = {};
  uint32_t started = millis();
  EI_IMPULSE_ERROR error = run_classifier(&signal, &result, false);
  canopyAIElapsedMs = millis() - started;
  canopyAISource = nullptr;
  if (error != EI_IMPULSE_OK) {
    canopyAIState = "inference_error";
    return false;
  }

  for (size_t i = 0; i < result.bounding_boxes_count; ++i) {
    const auto &box = result.bounding_boxes[i];
    if (strcmp(box.label, "plant") != 0 || box.value < 0.60f ||
        box.value <= confidence)
      continue;
    confidence = box.value;
    float modelX = box.x + box.width * 0.5f;
    float modelY = box.y + box.height * 0.5f;
    centerX = constrain((int)lroundf(modelX * 320.0f / 96.0f), 0, 319);
    centerY = constrain((int)lroundf(modelY * 240.0f / 96.0f), 0, 239);
  }
  canopyAIState = centerX >= 0 ? "plant_found" : "plant_not_found";
  return centerX >= 0;
#else
  (void)pixels;
  return false;
#endif
}
