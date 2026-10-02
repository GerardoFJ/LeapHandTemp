// tactile_demo.cpp
//
// A full tour of what the G1 can do to your skin, with no motion capture, no
// tracker and no OpenVR anywhere in it. Everything here is driven straight from
// the hardware definition the Airpack reports, so the only thing that has to
// work is USB -> Airpack -> peripheral connector.
//
// Defaults to the LEFT hand only. Pass --hand right or --hand both to change
// that; the hand is picked by Glove::handedness, not by connector slot, so it
// follows the glove if you move it.
//
// The demo has two halves, because the SDK renders them through different
// paths and they must not both own the Airpack at once:
//
//   Scenes 1-5  go through Airpack::render() at 90 Hz. These are inflation
//               set points: steady pressure, shaped over time.
//   Scenes 6-8  go through VibroAirpackWrapper, driven by a VibroTimingManager
//               thread at VIBRO_SAMPLE_RATE_HZ (180 Hz). These are waveforms
//               played into the same tactors, so they buzz rather than press.
//
// Scene list:
//   1  region tour       every coverage region on the hand, in anatomical order
//   2  intensity ladder  one fingertip at 25/50/75/100% so you feel the range
//   3  fingertip wave    thumb -> pinky and back, fast, as a rolling wave
//   4  fingertip stroke  a contact line swept across one fingertip pad, using
//                        the tactors' own positions
//   5  whole-hand grip   every tactor ramping up and releasing, like closing
//                        on an object
//   6  frequency sweep   vibro sine at 10 / 20 / 40 / 80 Hz on the fingertips
//   7  waveform textures vibro square / sawtooth / inverted sawtooth / noise
//   8  travelling buzz   one vibro clip walked region by region up the hand
//
// Usage: tactile_demo [options]
//   --list             print the tactor inventory and exit; actuate nothing
//   --list-scenes      print the scene list and exit
//   --hand <h>         left | right | both                    (default left)
//   --scene <n>        run only scene n (1-8); repeatable is not supported
//   --scale <0..1>     fraction of full inflation height       (default 1.0)
//   --vibro-amp <m>    vibro peak amplitude in metres          (default 0.0004)
//   --speed <x>        multiply every duration by x            (default 1.0)
//   --loops <n>        repeat the whole demo, 0 = forever      (default 1)
//   --no-vibro         skip scenes 6-8
//   --help
//
// Safety: inflation is always expressed as a fraction of each tactor's own
// getMaxHeightM(), and DirectPneumaticCalculator clamps to that tactor's height
// and pressure limits on top, so --scale 1.0 is full rated inflation rather
// than an overpressure. Start at --scale 0.5 for a gentler first run.
//
// Note on --vibro-amp: VibroClip's factory functions take amplitudes in metres,
// matching tactor heights (full scale on a G1 fingertip is about 0.92 mm), but
// the VibroClip::data field is documented as newtons. The two do not agree, so
// the amplitude is exposed as a flag rather than guessed at -- if scenes 6-8
// feel like nothing, raise it.

#ifdef _WIN32
// Ahead of everything: windows.h defines min/max macros that fight <algorithm>.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <HaptxApi/airpack.h>
#include <HaptxApi/anim_frame.h>
#include <HaptxApi/default_hand_ik.h>
#include <HaptxApi/direct_pneumatic_calculator.h>
#include <HaptxApi/enum.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/haptic_frame.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/haptx_uuid.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/mocap_frame.h>
#include <HaptxApi/mocap_system.h>
#include <HaptxApi/names.h>
#include <HaptxApi/peripheral.h>
#include <HaptxApi/pneumatic_frame.h>
#include <HaptxApi/tactor.h>
#include <HaptxApi/user_profile.h>
#include <HaptxApi/user_profile_database.h>
#include <HaptxApi/vibro.h>
#include <HaptxApi/vibro_clip_handle.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace HaptxApi;

namespace {

// How often inflation set points are pushed in scenes 1-5 [Hz].
constexpr double RENDER_RATE_HZ = 90.0;

// Spelled out rather than taken from <cmath>: MSVC only defines M_PI when
// _USE_MATH_DEFINES is set before the include.
constexpr double PI = 3.14159265358979323846;

// Set from the SIGINT handler so a Ctrl-C still deflates the hand.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

enum class HandFilter { LEFT, RIGHT, BOTH };

struct Options {
  bool list_only = false;
  bool list_scenes = false;
  bool no_vibro = false;
  HandFilter hand = HandFilter::LEFT;
  int scene = 0;  // 0 means "all scenes"
  int loops = 1;  // 0 means "until interrupted"
  bool hand_view = false;
  bool window = false;
  bool plain = false;
  double view_time_s = 0.0;  // 0 means "until interrupted"
  float scale = 1.0f;
  float vibro_amp_m = 0.0004f;
  double speed = 1.0;
};

const char* const SCENE_NAMES[] = {
    "region tour       every coverage region on the hand, in anatomical order",
    "intensity ladder  one fingertip at 25/50/75/100% so you feel the range",
    "fingertip wave    thumb -> pinky and back, fast, as a rolling wave",
    "fingertip stroke  a contact line swept across one fingertip pad",
    "whole-hand grip   every tactor ramping up and releasing",
    "frequency sweep   vibro sine at 10 / 20 / 40 / 80 Hz on the fingertips",
    "waveform textures vibro square / sawtooth / inverted sawtooth / noise",
    "travelling buzz   one vibro clip walked region by region up the hand",
};
constexpr int SCENE_COUNT = static_cast<int>(sizeof(SCENE_NAMES) / sizeof(SCENE_NAMES[0]));
constexpr int FIRST_VIBRO_SCENE = 6;

void printUsage() {
  printf(
      "Usage: tactile_demo [options]\n"
      "  --list             print the tactor inventory and exit; actuate nothing\n"
      "  --list-scenes      print the scene list and exit\n"
      "  --hand <h>         left | right | both                    (default left)\n"
      "  --scene <n>        run only scene n (1-%d)\n"
      "  --scale <0..1>     fraction of full inflation height       (default 1.0)\n"
      "  --vibro-amp <m>    vibro peak amplitude in metres          (default 0.0004)\n"
      "  --speed <x>        multiply every duration by x            (default 1.0)\n"
      "  --loops <n>        repeat the whole demo, 0 = forever      (default 1)\n"
      "  --no-vibro         skip scenes %d-%d\n"
      "  --hand-view        live view of the fingers closing, instead of the scenes\n"
      "  --view-time <s>    how long to hold the hand view open, 0 = forever\n"
      "  --window           draw the hand view in a window instead of the terminal\n"
      "  --plain            hand view scrolls instead of redrawing in place\n"
      "  --help\n",
      SCENE_COUNT, FIRST_VIBRO_SCENE, SCENE_COUNT);
}

void printScenes() {
  printf("Scenes:\n");
  for (int i = 0; i < SCENE_COUNT; i++) {
    printf("  %d  %s\n", i + 1, SCENE_NAMES[i]);
  }
  printf("\nScenes 1-%d render through Airpack::render(); scenes %d-%d through\n",
         FIRST_VIBRO_SCENE - 1, FIRST_VIBRO_SCENE, SCENE_COUNT);
  printf("VibroAirpackWrapper on a VibroTimingManager thread at %d Hz.\n", VIBRO_SAMPLE_RATE_HZ);
}

std::string toLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

bool parseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; i++) {
    const char* arg = argv[i];
    const bool has_value = i + 1 < argc;

    if (strcmp(arg, "--list") == 0) {
      options->list_only = true;
    } else if (strcmp(arg, "--list-scenes") == 0) {
      options->list_scenes = true;
    } else if (strcmp(arg, "--no-vibro") == 0) {
      options->no_vibro = true;
    } else if (strcmp(arg, "--hand-view") == 0) {
      options->hand_view = true;
    } else if (strcmp(arg, "--window") == 0) {
      options->window = true;
      options->hand_view = true;  // --window implies the hand view
    } else if (strcmp(arg, "--plain") == 0) {
      options->plain = true;
    } else if (strcmp(arg, "--view-time") == 0 && has_value) {
      options->view_time_s = atof(argv[++i]);
    } else if (strcmp(arg, "--hand") == 0 && has_value) {
      const std::string hand = toLower(argv[++i]);
      if (hand == "left") {
        options->hand = HandFilter::LEFT;
      } else if (hand == "right") {
        options->hand = HandFilter::RIGHT;
      } else if (hand == "both") {
        options->hand = HandFilter::BOTH;
      } else {
        printf("Unrecognized --hand: %s (want left, right or both)\n", hand.c_str());
        return false;
      }
    } else if (strcmp(arg, "--scene") == 0 && has_value) {
      options->scene = atoi(argv[++i]);
    } else if (strcmp(arg, "--scale") == 0 && has_value) {
      options->scale = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--vibro-amp") == 0 && has_value) {
      options->vibro_amp_m = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--speed") == 0 && has_value) {
      options->speed = atof(argv[++i]);
    } else if (strcmp(arg, "--loops") == 0 && has_value) {
      options->loops = atoi(argv[++i]);
    } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      printUsage();
      return false;
    } else {
      printf("Unrecognized argument: %s\n\n", arg);
      printUsage();
      return false;
    }
  }

  if (options->scale < 0.0f || options->scale > 1.0f) {
    printf("--scale must be between 0 and 1.\n");
    return false;
  }
  if (options->speed <= 0.0) {
    printf("--speed must be positive.\n");
    return false;
  }
  if (options->scene < 0 || options->scene > SCENE_COUNT) {
    printf("--scene must be between 1 and %d.\n", SCENE_COUNT);
    return false;
  }
  if (options->loops < 0) {
    printf("--loops must not be negative.\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Hardware selection
// ---------------------------------------------------------------------------

// One peripheral this demo is allowed to drive.
struct Target {
  int slot = 0;
  std::shared_ptr<Peripheral> peripheral;
  RelativeDirection handedness = RD_UNKNOWN;
  std::string label;
};

// Tactor ids keyed by the slot they live on, with a level in [0, 1] each.
using LevelMap = std::unordered_map<int, float>;
using SlotLevels = std::map<int, LevelMap>;

// Vibro clip weights keyed by slot, then by tactor id.
using SlotVibro = std::map<int, std::unordered_map<int, VectorOfVibroHandleWeightPairs>>;

std::string describePeripheral(int slot, const Peripheral& peripheral) {
  std::string handedness = "non-glove";
  if (const auto* glove = dynamic_cast<const Glove*>(&peripheral)) {
    handedness = toString(glove->handedness);
  }
  return "slot " + std::to_string(slot) + " (" + handedness + ", " + peripheral.casual_name + ")";
}

bool handWanted(RelativeDirection handedness, HandFilter filter) {
  switch (filter) {
    case HandFilter::LEFT:
      return handedness == RD_LEFT;
    case HandFilter::RIGHT:
      return handedness == RD_RIGHT;
    case HandFilter::BOTH:
      return handedness == RD_LEFT || handedness == RD_RIGHT;
  }
  return false;
}

// Every coverage region, in anatomical order, that belongs to a given hand.
std::vector<CoverageRegion> regionsForHand(RelativeDirection handedness) {
  std::vector<CoverageRegion> regions;
  const bool want_right = handedness == RD_RIGHT;
  for (int i = 0; i < static_cast<int>(CoverageRegion::LAST); i++) {
    const auto region = static_cast<CoverageRegion>(i);
    // The enum runs all RIGHT_* regions first, then all LEFT_* ones.
    const bool is_right = i < static_cast<int>(CoverageRegion::LEFT_THUMB_METACARPAL);
    if (is_right == want_right) {
      regions.push_back(region);
    }
  }
  return regions;
}

// The five fingertip regions of a hand, thumb to pinky.
std::vector<CoverageRegion> fingertipsForHand(RelativeDirection handedness) {
  if (handedness == RD_RIGHT) {
    return {CoverageRegion::RIGHT_THUMB_DISTAL, CoverageRegion::RIGHT_INDEX_DISTAL,
            CoverageRegion::RIGHT_MIDDLE_DISTAL, CoverageRegion::RIGHT_RING_DISTAL,
            CoverageRegion::RIGHT_PINKY_DISTAL};
  }
  return {CoverageRegion::LEFT_THUMB_DISTAL, CoverageRegion::LEFT_INDEX_DISTAL,
          CoverageRegion::LEFT_MIDDLE_DISTAL, CoverageRegion::LEFT_RING_DISTAL,
          CoverageRegion::LEFT_PINKY_DISTAL};
}

// The drivable tactors of one coverage region on one peripheral.
std::vector<const Tactor*> tactorsInRegion(const Peripheral& peripheral, CoverageRegion region) {
  const HaptxName region_name = getName(region);
  std::vector<const Tactor*> tactors;
  for (const Tactor& tactor : peripheral.tactors) {
    if (tactor.coverage_region == region_name && tactor.isEnabled()) {
      tactors.push_back(&tactor);
    }
  }
  return tactors;
}

std::vector<int> idsOf(const std::vector<const Tactor*>& tactors) {
  std::vector<int> ids;
  ids.reserve(tactors.size());
  for (const Tactor* tactor : tactors) {
    ids.push_back(tactor->getId());
  }
  return ids;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

// Turns per-tactor levels into a HapticFrame for one peripheral.
//
// Every tactor is named -- the ones that should be flat are commanded to their
// own minimum height rather than left out -- so a frame fully describes the
// hardware state and nothing stays inflated by omission. The force actuators
// are explicitly held DISENGAGED: this demo is about the skin, and the brakes
// should be out of the way.
HapticFrame buildHapticFrame(const Peripheral& peripheral, const LevelMap& levels,
                             const std::unordered_map<int, VectorOfVibroHandleWeightPairs>* vibro,
                             float scale) {
  HapticFrame haptic_frame;

  for (const Tactor& tactor : peripheral.tactors) {
    const int id = tactor.getId();
    float level = 0.0f;
    if (tactor.isEnabled()) {
      const auto it = levels.find(id);
      if (it != levels.end()) {
        level = std::max(0.0f, std::min(1.0f, it->second));
      }
    }
    const float min_m = tactor.getMinHeightM();
    const float max_m = tactor.getMaxHeightM();
    haptic_frame.tactor_heights_m[id] = min_m + level * scale * (max_m - min_m);
  }

  for (const ForceActuator& force_actuator : peripheral.force_actuators) {
    haptic_frame.force_actuator_states[force_actuator.getId()] =
        PassiveForceActuator::State::DISENGAGED;
  }

  if (vibro != nullptr) {
    haptic_frame.vibro_clip_weights = *vibro;
  }
  return haptic_frame;
}

// Scenes 1-5: straight to the Airpack.
bool renderDirect(Airpack& airpack, const std::vector<Target>& targets, const SlotLevels& levels,
                  float scale) {
  PneumaticFrame pneumatic_frame;

  for (const Target& target : targets) {
    static const LevelMap kEmpty;
    const auto it = levels.find(target.slot);
    const LevelMap& slot_levels = it != levels.end() ? it->second : kEmpty;

    const HapticFrame haptic_frame =
        buildHapticFrame(*target.peripheral, slot_levels, nullptr, scale);
    if (!DirectPneumaticCalculator::addToPneumaticFrame(*target.peripheral, haptic_frame, airpack,
                                                        &pneumatic_frame)) {
      printf("DirectPneumaticCalculator::addToPneumaticFrame() failed for %s.\n",
             target.label.c_str());
      return false;
    }
  }

  if (auto ret = airpack.render(pneumatic_frame); !ret) {
    printf("Airpack::render() returned error code %d: %s.\n", static_cast<int>(ret.error()),
           toString(ret.error()).c_str());
    return false;
  }
  return true;
}

// Scenes 6-8: hand the frames to the wrapper and let its thread render them.
void publishVibro(VibroAirpackWrapper& wrapper, const std::vector<Target>& targets,
                  const SlotLevels& levels, const SlotVibro& vibro, float scale) {
  std::unordered_map<HaptxUuid, HapticFrame> frames;

  for (const Target& target : targets) {
    static const LevelMap kEmptyLevels;
    static const std::unordered_map<int, VectorOfVibroHandleWeightPairs> kEmptyVibro;

    const auto level_it = levels.find(target.slot);
    const LevelMap& slot_levels = level_it != levels.end() ? level_it->second : kEmptyLevels;
    const auto vibro_it = vibro.find(target.slot);
    const auto& slot_vibro = vibro_it != vibro.end() ? vibro_it->second : kEmptyVibro;

    frames[target.peripheral->id] =
        buildHapticFrame(*target.peripheral, slot_levels, &slot_vibro, scale);
  }

  wrapper.overwriteNonVibroFrames(frames);
}

// Produces the per-tactor levels for a moment in time.
using LevelFn = std::function<void(double elapsed_s, double duration_s, SlotLevels* out)>;

// Runs one timed scene on the direct render path.
bool runDirectScene(Airpack& airpack, const std::vector<Target>& targets, const Options& options,
                    double duration_s, const LevelFn& level_fn) {
  const auto period = std::chrono::duration<double>(1.0 / RENDER_RATE_HZ);
  const auto start = std::chrono::steady_clock::now();

  for (;;) {
    if (g_interrupted) {
      return false;
    }
    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    SlotLevels levels;
    level_fn(std::min(elapsed_s, duration_s), duration_s, &levels);
    if (!renderDirect(airpack, targets, levels, options.scale)) {
      return false;
    }
    if (elapsed_s >= duration_s) {
      return true;
    }
    std::this_thread::sleep_for(period);
  }
}

// Sleeps in small slices so Ctrl-C stays responsive. Returns false if interrupted.
bool sleepInterruptible(double seconds) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (g_interrupted) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return !g_interrupted;
}

// A sine that starts and ends deflated, so a region fades in rather than
// slamming on. Used wherever a region is "pulsed".
float pulse(double elapsed_s, double frequency_hz) {
  return static_cast<float>((1.0 - std::cos(2.0 * PI * frequency_hz * elapsed_s)) / 2.0);
}

// ---------------------------------------------------------------------------
// Scenes 1-5: inflation
// ---------------------------------------------------------------------------

bool sceneRegionTour(Airpack& airpack, const std::vector<Target>& targets, const Options& options) {
  const double hold_s = 1.5 * options.speed;

  for (const Target& target : targets) {
    for (const CoverageRegion region : regionsForHand(target.handedness)) {
      if (g_interrupted) {
        return false;
      }
      const std::vector<const Tactor*> tactors = tactorsInRegion(*target.peripheral, region);
      if (tactors.empty()) {
        continue;
      }
      const std::vector<int> ids = idsOf(tactors);

      printf("    %-26s %2d tactors\n", getName(region).getText().c_str(),
             static_cast<int>(ids.size()));
      std::flush(std::cout);

      const int slot = target.slot;
      if (!runDirectScene(airpack, targets, options, hold_s,
                          [&ids, slot](double elapsed_s, double duration_s, SlotLevels* out) {
                            // One full raised-cosine over the hold: in, and out.
                            const float level = pulse(elapsed_s, 0.5 / duration_s);
                            for (const int id : ids) {
                              (*out)[slot][id] = level;
                            }
                          })) {
        return false;
      }
    }
  }
  return true;
}

bool sceneIntensityLadder(Airpack& airpack, const std::vector<Target>& targets,
                          const Options& options) {
  const double step_s = 1.2 * options.speed;
  const float steps[] = {0.25f, 0.5f, 0.75f, 1.0f};

  for (const Target& target : targets) {
    const CoverageRegion region = fingertipsForHand(target.handedness)[1];  // index distal
    const std::vector<int> ids = idsOf(tactorsInRegion(*target.peripheral, region));
    if (ids.empty()) {
      continue;
    }
    printf("    %s, stepping 25%% -> 100%%\n", getName(region).getText().c_str());
    std::flush(std::cout);

    for (const float step : steps) {
      if (g_interrupted) {
        return false;
      }
      printf("      %3.0f%%\n", step * 100.0f);
      std::flush(std::cout);

      const int slot = target.slot;
      if (!runDirectScene(airpack, targets, options, step_s,
                          [&ids, slot, step](double elapsed_s, double duration_s,
                                             SlotLevels* out) {
                            // Hold flat, but ease in and out over 150 ms so the
                            // step is a step in pressure, not in jerk.
                            const double edge_s = std::min(0.15, duration_s / 4.0);
                            double envelope = 1.0;
                            if (elapsed_s < edge_s) {
                              envelope = elapsed_s / edge_s;
                            } else if (elapsed_s > duration_s - edge_s) {
                              envelope = (duration_s - elapsed_s) / edge_s;
                            }
                            for (const int id : ids) {
                              (*out)[slot][id] = step * static_cast<float>(envelope);
                            }
                          })) {
        return false;
      }
    }
  }
  return true;
}

bool sceneFingertipWave(Airpack& airpack, const std::vector<Target>& targets,
                        const Options& options) {
  const double pass_s = 1.2 * options.speed;
  const int passes = 4;

  for (const Target& target : targets) {
    // Gather the five fingertips in thumb-to-pinky order.
    std::vector<std::vector<int>> fingers;
    for (const CoverageRegion region : fingertipsForHand(target.handedness)) {
      const std::vector<int> ids = idsOf(tactorsInRegion(*target.peripheral, region));
      if (!ids.empty()) {
        fingers.push_back(ids);
      }
    }
    if (fingers.empty()) {
      continue;
    }
    printf("    %d fingertips, %d passes, alternating direction\n",
           static_cast<int>(fingers.size()), passes);
    std::flush(std::cout);

    const int slot = target.slot;
    for (int pass = 0; pass < passes; pass++) {
      if (g_interrupted) {
        return false;
      }
      const bool reverse = (pass % 2) == 1;
      if (!runDirectScene(
              airpack, targets, options, pass_s,
              [&fingers, slot, reverse](double elapsed_s, double duration_s, SlotLevels* out) {
                // A bump travelling along the finger axis. Each finger lights
                // up as the bump passes over its position.
                const double count = static_cast<double>(fingers.size());
                double head = (elapsed_s / duration_s) * (count + 1.0) - 0.5;
                if (reverse) {
                  head = count - head;
                }
                for (size_t i = 0; i < fingers.size(); i++) {
                  const double distance = std::fabs(static_cast<double>(i) - head);
                  const double level = std::max(0.0, 1.0 - distance);
                  for (const int id : fingers[i]) {
                    (*out)[slot][id] = static_cast<float>(level);
                  }
                }
              })) {
        return false;
      }
    }
  }
  return true;
}

bool sceneFingertipStroke(Airpack& airpack, const std::vector<Target>& targets,
                          const Options& options) {
  const double sweep_s = 2.0 * options.speed;
  const int sweeps = 3;

  for (const Target& target : targets) {
    const CoverageRegion region = fingertipsForHand(target.handedness)[1];  // index distal
    const std::vector<const Tactor*> tactors = tactorsInRegion(*target.peripheral, region);
    if (tactors.size() < 2) {
      continue;
    }

    // Tactor positions are expressed in the frame named by getParent(), so they
    // are only comparable to each other when they share a parent. Without the
    // mocap/anim stack there is nothing to resolve differing frames against, so
    // check rather than assume.
    const HaptxName parent = tactors.front()->getParent();
    for (const Tactor* tactor : tactors) {
      if (tactor->getParent() != parent) {
        printf("    %s spans more than one reference frame (%s vs %s) --\n",
               getName(region).getText().c_str(), parent.getText().c_str(),
               tactor->getParent().getText().c_str());
        printf("    skipping the geometric stroke, which needs one common frame.\n");
        return true;
      }
    }

    // Sweep along whichever axis the pad is widest on.
    Vector3D low = tactors.front()->getTransform().getTranslation();
    Vector3D high = low;
    for (const Tactor* tactor : tactors) {
      const Vector3D position = tactor->getTransform().getTranslation();
      low.x_ = std::min(low.x_, position.x_);
      low.y_ = std::min(low.y_, position.y_);
      low.z_ = std::min(low.z_, position.z_);
      high.x_ = std::max(high.x_, position.x_);
      high.y_ = std::max(high.y_, position.y_);
      high.z_ = std::max(high.z_, position.z_);
    }
    const float spans[3] = {high.x_ - low.x_, high.y_ - low.y_, high.z_ - low.z_};
    const int axis = static_cast<int>(std::max_element(spans, spans + 3) - spans);
    const float span = spans[axis];
    if (span <= 0.0f) {
      printf("    %s tactors are coincident on every axis -- skipping the stroke.\n",
             getName(region).getText().c_str());
      return true;
    }

    // Positions along the chosen axis, normalized to [0, 1].
    std::vector<std::pair<int, float>> positions;
    for (const Tactor* tactor : tactors) {
      const Vector3D p = tactor->getTransform().getTranslation();
      const float coordinate = axis == 0 ? p.x_ : (axis == 1 ? p.y_ : p.z_);
      const float low_coordinate = axis == 0 ? low.x_ : (axis == 1 ? low.y_ : low.z_);
      positions.emplace_back(tactor->getId(), (coordinate - low_coordinate) / span);
    }

    printf("    %s, %d tactors over %.1f mm on the %c axis, %d sweeps\n",
           getName(region).getText().c_str(), static_cast<int>(positions.size()), span * 1000.0f,
           "xyz"[axis], sweeps);
    std::flush(std::cout);

    const int slot = target.slot;
    for (int sweep = 0; sweep < sweeps; sweep++) {
      if (g_interrupted) {
        return false;
      }
      const bool reverse = (sweep % 2) == 1;
      if (!runDirectScene(
              airpack, targets, options, sweep_s,
              [&positions, slot, reverse](double elapsed_s, double duration_s, SlotLevels* out) {
                // A soft contact line crossing the pad, a third of its width.
                constexpr double kWidth = 0.33;
                double head = elapsed_s / duration_s;
                if (reverse) {
                  head = 1.0 - head;
                }
                for (const auto& entry : positions) {
                  const double distance = std::fabs(entry.second - head) / kWidth;
                  const double level = distance >= 1.0 ? 0.0 : (1.0 + std::cos(PI * distance)) / 2.0;
                  (*out)[slot][entry.first] = static_cast<float>(level);
                }
              })) {
        return false;
      }
    }
  }
  return true;
}

bool sceneWholeHandGrip(Airpack& airpack, const std::vector<Target>& targets,
                        const Options& options) {
  const double ramp_s = 1.5 * options.speed;
  const double hold_s = 1.0 * options.speed;
  const double release_s = 1.2 * options.speed;
  const double total_s = ramp_s + hold_s + release_s;

  for (const Target& target : targets) {
    std::vector<int> ids;
    for (const Tactor& tactor : target.peripheral->tactors) {
      if (tactor.isEnabled()) {
        ids.push_back(tactor.getId());
      }
    }
    if (ids.empty()) {
      continue;
    }
    printf("    all %d tactors: %.1fs squeeze, %.1fs hold, %.1fs release\n",
           static_cast<int>(ids.size()), ramp_s, hold_s, release_s);
    std::flush(std::cout);

    const int slot = target.slot;
    if (!runDirectScene(airpack, targets, options, total_s,
                        [&ids, slot, ramp_s, hold_s, release_s](double elapsed_s, double,
                                                                SlotLevels* out) {
                          double level = 0.0;
                          if (elapsed_s < ramp_s) {
                            level = elapsed_s / ramp_s;
                          } else if (elapsed_s < ramp_s + hold_s) {
                            level = 1.0;
                          } else {
                            level = std::max(0.0, 1.0 - (elapsed_s - ramp_s - hold_s) / release_s);
                          }
                          for (const int id : ids) {
                            (*out)[slot][id] = static_cast<float>(level);
                          }
                        })) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Scenes 6-8: vibrotactile
// ---------------------------------------------------------------------------

// Everything needed to play vibro clips, torn down in the right order.
struct VibroStack {
  std::shared_ptr<VibroClipManager> clip_manager;
  std::shared_ptr<VibroAirpackWrapper> wrapper;
  std::unique_ptr<VibroTimingManager> timing_manager;
  bool running = false;

  bool start(const std::shared_ptr<Airpack>& airpack) {
    clip_manager = std::make_shared<VibroClipManager>();
    wrapper = std::make_shared<VibroAirpackWrapper>(airpack, clip_manager);
    timing_manager = std::make_unique<VibroTimingManager>();

    if (!timing_manager->registerClipManager(clip_manager)) {
      printf("VibroTimingManager::registerClipManager() failed.\n");
      return false;
    }
    if (!timing_manager->registerAirpackWrapper(wrapper)) {
      printf("VibroTimingManager::registerAirpackWrapper() failed.\n");
      return false;
    }
    if (!timing_manager->startVibroThread()) {
      printf("VibroTimingManager::startVibroThread() failed.\n");
      return false;
    }
    running = true;
    return true;
  }

  void stop() {
    if (timing_manager != nullptr) {
      if (running) {
        const uint64_t dropped = timing_manager->getDroppedFrameCount();
        if (dropped > 0) {
          printf("    (vibro thread dropped %llu frames)\n",
                 static_cast<unsigned long long>(dropped));
        }
        timing_manager->stopVibroThread();
        running = false;
      }
      if (wrapper != nullptr) {
        timing_manager->unregisterAirpackWrapper(wrapper);
      }
      if (clip_manager != nullptr) {
        timing_manager->unregisterClipManager(clip_manager);
      }
    }
    if (clip_manager != nullptr) {
      clip_manager->stopAllClips();
    }
    timing_manager.reset();
    wrapper.reset();
    clip_manager.reset();
  }
};

// Plays one clip on a set of tactors for a while, then stops it.
bool playClip(VibroStack& vibro, const std::vector<Target>& targets, const Options& options,
              const SlotVibro& weights_template, const VibroClip& clip, VibroClipHandle handle,
              double duration_s) {
  vibro.clip_manager->startClip(handle, clip);
  publishVibro(*vibro.wrapper, targets, SlotLevels{}, weights_template, options.scale);

  const bool completed = sleepInterruptible(duration_s);

  vibro.clip_manager->stopClip(handle);
  publishVibro(*vibro.wrapper, targets, SlotLevels{}, SlotVibro{}, options.scale);

  if (const ReturnCode error = vibro.wrapper->getLatestAirpackError(); error != ReturnCode::SUCCESS) {
    printf("      VibroAirpackWrapper reported %s\n", toString(error).c_str());
    vibro.wrapper->clearLatestAirpackError();
  }
  return completed;
}

// Builds "every fingertip tactor gets this one clip at full weight".
SlotVibro fingertipWeights(const std::vector<Target>& targets, VibroClipHandle handle) {
  SlotVibro weights;
  for (const Target& target : targets) {
    for (const CoverageRegion region : fingertipsForHand(target.handedness)) {
      for (const Tactor* tactor : tactorsInRegion(*target.peripheral, region)) {
        weights[target.slot][tactor->getId()] = {{handle, 1.0f}};
      }
    }
  }
  return weights;
}

bool sceneFrequencySweep(VibroStack& vibro, const std::vector<Target>& targets,
                         const Options& options) {
  const double step_s = 2.0 * options.speed;
  const float frequencies[] = {10.0f, 20.0f, 40.0f, 80.0f};

  for (const float frequency_hz : frequencies) {
    if (g_interrupted) {
      return false;
    }
    if (frequency_hz > VIBRO_MAX_WAVE_HZ) {
      printf("      %.0f Hz is above the %.0f Hz Nyquist limit -- skipped\n", frequency_hz,
             VIBRO_MAX_WAVE_HZ);
      continue;
    }
    printf("      %.0f Hz sine\n", frequency_hz);
    std::flush(std::cout);

    const VibroClipHandle handle = getUniqueVibroHandle();
    // Offset equal to the amplitude keeps the wave non-negative.
    const VibroClip clip = VibroClip::createLoopingSinClip(frequency_hz, options.vibro_amp_m,
                                                           options.vibro_amp_m);
    if (!playClip(vibro, targets, options, fingertipWeights(targets, handle), clip, handle,
                  step_s)) {
      return false;
    }
  }
  return true;
}

bool sceneWaveformTextures(VibroStack& vibro, const std::vector<Target>& targets,
                           const Options& options) {
  const double step_s = 2.0 * options.speed;
  const float frequency_hz = 30.0f;

  struct Texture {
    const char* name;
    VibroClip clip;
  };
  const std::vector<Texture> textures = {
      {"square", VibroClip::createLoopingSquareClip(frequency_hz, options.vibro_amp_m, false)},
      {"sawtooth", VibroClip::createLoopingSawtoothClip(frequency_hz, options.vibro_amp_m, false)},
      {"sawtooth inverted",
       VibroClip::createLoopingSawtoothClip(frequency_hz, options.vibro_amp_m, true)},
      {"uniform noise",
       VibroClip::createUniformNoiseClip(options.vibro_amp_m, static_cast<float>(step_s))},
  };

  for (const Texture& texture : textures) {
    if (g_interrupted) {
      return false;
    }
    printf("      %s\n", texture.name);
    std::flush(std::cout);

    const VibroClipHandle handle = getUniqueVibroHandle();
    if (!playClip(vibro, targets, options, fingertipWeights(targets, handle), texture.clip, handle,
                  step_s)) {
      return false;
    }
  }
  return true;
}

bool sceneTravellingBuzz(VibroStack& vibro, const std::vector<Target>& targets,
                         const Options& options) {
  const double step_s = 0.7 * options.speed;
  const VibroClipHandle handle = getUniqueVibroHandle();
  const VibroClip clip =
      VibroClip::createLoopingSinClip(40.0f, options.vibro_amp_m, options.vibro_amp_m);

  vibro.clip_manager->startClip(handle, clip);

  bool completed = true;
  for (const Target& target : targets) {
    for (const CoverageRegion region : regionsForHand(target.handedness)) {
      if (g_interrupted) {
        completed = false;
        break;
      }
      const std::vector<const Tactor*> tactors = tactorsInRegion(*target.peripheral, region);
      if (tactors.empty()) {
        continue;
      }
      printf("      %s\n", getName(region).getText().c_str());
      std::flush(std::cout);

      SlotVibro weights;
      for (const Tactor* tactor : tactors) {
        weights[target.slot][tactor->getId()] = {{handle, 1.0f}};
      }
      publishVibro(*vibro.wrapper, targets, SlotLevels{}, weights, options.scale);

      if (!sleepInterruptible(step_s)) {
        completed = false;
        break;
      }
    }
    if (!completed) {
      break;
    }
  }

  vibro.clip_manager->stopClip(handle);
  publishVibro(*vibro.wrapper, targets, SlotLevels{}, SlotVibro{}, options.scale);
  return completed;
}

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Hand view: watch the fingers close
// ---------------------------------------------------------------------------
//
// This is the one part of the program that needs motion capture, because
// finger angles are what mocap produces. The tactile scenes never touch it.
//
// The chain is: MocapSystem::update() reads the magnetic sensors,
// addToMocapFrame() puts fingertip poses into a MocapFrame relative to the
// MCP3 tracking origin, and DefaultHandIk::addToAnimFrame() solves those into
// per-finger joint angles. AnimFrame::angles carries MCP, PIP and DIP flexion
// in radians, which is exactly "how closed is this finger".

// Full flexion used to scale the closure bar: roughly 90 + 100 + 80 degrees
// across the three joints. Only used for display.
constexpr double FULL_FLEXION_DEG = 270.0;

const char* const FINGER_NAMES[F_LAST] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};

// One glove's mocap pipeline, opened only for --hand-view.
struct HandViewSource {
  Target target;
  std::shared_ptr<MocapSystem> mocap_system;
  std::shared_ptr<Glove> glove;
  int tolerated_errors = 0;
  int hard_errors = 0;
  int frame_failures = 0;   // addToMocapFrame() said no
  int ik_failures = 0;      // addToAnimFrame() said no
  int last_transforms = 0;  // how much mocap data reached the frame
  ReturnCode last_frame_error = ReturnCode::SUCCESS;
};

// How far each joint of one finger is bent.
struct FingerFlexion {
  bool valid = false;
  float joint_rad[FJ_LAST] = {0.0f, 0.0f, 0.0f};
};

// Pulls a finger's bend out of an AnimFrame.
//
// DefaultHandIk::addToAnimFrame() documents itself as returning true when
// "joint orientations for each finger joint are inserted" -- those go into
// AnimFrame::l_orientations. AnimFrame::angles is a separate field the solver
// does not always populate, so prefer it when it is there and fall back to the
// orientations, whose rotation magnitude about the joint axis is the bend.
FingerFlexion flexionFor(const AnimFrame& anim_frame, RelativeDirection handedness,
                         Finger finger) {
  FingerFlexion out;

  const auto angle_it = anim_frame.angles.find(finger);
  if (angle_it != anim_frame.angles.end() && angle_it->second.flexion.isValid()) {
    out.valid = true;
    out.joint_rad[FJ_JOINT1] = angle_it->second.flexion.theta_mcp_rad;
    out.joint_rad[FJ_JOINT2] = angle_it->second.flexion.theta_pip_rad;
    out.joint_rad[FJ_JOINT3] = angle_it->second.flexion.theta_dip_rad;
    return out;
  }

  for (unsigned int joint = 0; joint < FJ_LAST; joint++) {
    const BodyPartJoint key =
        getBodyPartJoint(handedness, finger, static_cast<FingerJoint>(joint));
    const auto it = anim_frame.l_orientations.find(key);
    if (it == anim_frame.l_orientations.end()) {
      continue;
    }
    Vector3D axis;
    float theta_rad = 0.0f;
    it->second.toAxisAngle(axis, theta_rad);
    out.joint_rad[joint] = std::fabs(theta_rad);
    out.valid = true;
  }
  return out;
}

// Draws one finger's row: joint angles plus a closure bar.
void drawFingerRow(const char* name, const FingerFlexion& flexion) {
  if (!flexion.valid) {
    printf("  %-7s    --     --     --   %-24s\n", name, "(no data)");
    return;
  }

  constexpr double kRadToDeg = 180.0 / PI;
  const double mcp_deg = flexion.joint_rad[FJ_JOINT1] * kRadToDeg;
  const double pip_deg = flexion.joint_rad[FJ_JOINT2] * kRadToDeg;
  const double dip_deg = flexion.joint_rad[FJ_JOINT3] * kRadToDeg;
  const double closure =
      std::max(0.0, std::min(1.0, (mcp_deg + pip_deg + dip_deg) / FULL_FLEXION_DEG));

  constexpr int kBarWidth = 22;
  const int filled = static_cast<int>(closure * kBarWidth + 0.5);
  std::string bar(kBarWidth, '.');
  for (int i = 0; i < filled && i < kBarWidth; i++) {
    bar[i] = '#';
  }

  printf("  %-7s %5.0f  %5.0f  %5.0f   |%s| %3.0f%%\n", name, mcp_deg, pip_deg, dip_deg,
         bar.c_str(), closure * 100.0);
}

// A hand's worth of flexion, ready to draw.
struct HandPose {
  std::string label;
  RelativeDirection handedness = RD_UNKNOWN;
  bool valid = false;
  FingerFlexion fingers[F_LAST];
};

#ifdef _WIN32
// ---------------------------------------------------------------------------
// A window with a hand in it
// ---------------------------------------------------------------------------
//
// Plain Win32 and GDI on purpose. The SDK ships HyleasVisualizer.exe, but it is
// a UE4 application and its D3D11 backend asserts under Wine before it ever
// opens a window ("CanFormatBeDisplayed"). GDI goes straight through Wine to
// X11 with no GPU path involved, so it just works.

// The hand is drawn in profile, fingers pointing right, because that is the
// only 2D view where closing reads correctly. Seen from the back, a curling
// finger folds into the page and no amount of rotation on screen shows that
// honestly; from the side, flexion is just rotation and the pose is unambiguous.
struct FingerLayout {
  double base_x;    // where the finger leaves the palm, in units of palm width
  double base_y;    // and at what height, in units of palm height
  double angle_deg; // which way it points before any bend, 0 = right
  double length;    // finger length, in units of the drawing size
  double curl;      // +1 curls down toward the palm, -1 curls up
};

// Four fingers leave the front edge of the palm at slightly different heights
// and angles so they stay tellable apart; the thumb leaves the bottom and
// closes upward against them, the way an opposed thumb does.
constexpr FingerLayout FINGER_LAYOUTS[F_LAST] = {
    {0.72, 1.00, 38.0, 0.26, -1.0},  // F_THUMB
    {1.00, 0.16, -9.0, 0.36, 1.0},   // F_INDEX
    {1.00, 0.37, -3.0, 0.40, 1.0},   // F_MIDDLE
    {1.00, 0.58, 3.0, 0.36, 1.0},    // F_RING
    {1.00, 0.78, 9.0, 0.29, 1.0},    // F_PINKY
};

// Proximal, medial and distal shares of a finger's length.
constexpr double SEGMENT_FRACTIONS[FJ_LAST] = {0.42, 0.33, 0.25};

COLORREF fingerColor(double closure) {
  // Cool when open, warm when closed.
  const int r = static_cast<int>(70 + 185 * closure);
  const int g = static_cast<int>(200 - 110 * closure);
  const int b = static_cast<int>(245 - 175 * closure);
  return RGB(r, g, b);
}

LRESULT CALLBACK handWindowProc(HWND hwnd, UINT message, WPARAM w_param, LPARAM l_param) {
  switch (message) {
    case WM_CLOSE:
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;
    case WM_ERASEBKGND:
      return 1;  // Everything is painted into a back buffer; never flash.
    default:
      break;
  }
  return DefWindowProcA(hwnd, message, w_param, l_param);
}

class HandWindow {
 public:
  bool create() {
    WNDCLASSA window_class = {};
    window_class.lpfnWndProc = handWindowProc;
    window_class.hInstance = GetModuleHandleA(nullptr);
    window_class.hCursor = LoadCursorA(nullptr, IDC_ARROW);
    window_class.lpszClassName = "HaptxHandView";
    if (RegisterClassA(&window_class) == 0) {
      printf("RegisterClassA() failed (%lu).\n", GetLastError());
      return false;
    }

    hwnd_ = CreateWindowExA(0, "HaptxHandView", "HaptX hand view", WS_OVERLAPPEDWINDOW,
                            CW_USEDEFAULT, CW_USEDEFAULT, 820, 620, nullptr, nullptr,
                            window_class.hInstance, nullptr);
    if (hwnd_ == nullptr) {
      printf("CreateWindowExA() failed (%lu).\n", GetLastError());
      return false;
    }
    ShowWindow(hwnd_, SW_SHOW);
    UpdateWindow(hwnd_);

    font_ = CreateFontA(-14, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                        FF_DONTCARE, "Consolas");
    return true;
  }

  // Returns false once the user closes the window.
  bool pump() {
    MSG message;
    while (PeekMessageA(&message, nullptr, 0, 0, PM_REMOVE)) {
      if (message.message == WM_QUIT) {
        return false;
      }
      TranslateMessage(&message);
      DispatchMessageA(&message);
    }
    return true;
  }

  void draw(const std::vector<HandPose>& hands) {
    if (hwnd_ == nullptr) {
      return;
    }
    RECT client;
    GetClientRect(hwnd_, &client);
    const int width = client.right - client.left;
    const int height = client.bottom - client.top;
    if (width <= 0 || height <= 0) {
      return;
    }

    const HDC window_dc = GetDC(hwnd_);
    const HDC dc = CreateCompatibleDC(window_dc);
    const HBITMAP buffer = CreateCompatibleBitmap(window_dc, width, height);
    const HGDIOBJ old_bitmap = SelectObject(dc, buffer);

    const HBRUSH background = CreateSolidBrush(RGB(18, 20, 26));
    FillRect(dc, &client, background);
    DeleteObject(background);

    SetBkMode(dc, TRANSPARENT);
    SelectObject(dc, font_);

    // Split the width between however many hands are being watched.
    const int panel_width = hands.empty() ? width : width / static_cast<int>(hands.size());
    for (size_t i = 0; i < hands.size(); i++) {
      RECT panel = client;
      panel.left = static_cast<LONG>(i) * panel_width;
      panel.right = panel.left + panel_width;
      drawHand(dc, panel, hands[i]);
    }

    BitBlt(window_dc, 0, 0, width, height, dc, 0, 0, SRCCOPY);

    SelectObject(dc, old_bitmap);
    DeleteObject(buffer);
    DeleteDC(dc);
    ReleaseDC(hwnd_, window_dc);
  }

  void destroy() {
    if (font_ != nullptr) {
      DeleteObject(font_);
      font_ = nullptr;
    }
    if (hwnd_ != nullptr) {
      DestroyWindow(hwnd_);
      hwnd_ = nullptr;
    }
  }

 private:
  void drawHand(HDC dc, const RECT& panel, const HandPose& pose) {
    const int width = panel.right - panel.left;
    const int height = panel.bottom - panel.top;
    const double size = std::min(width, height) * 0.92;

    // A left hand is the same profile seen from the other side, so mirror x
    // about the panel centre and let the fingers point the other way.
    const double mirror = pose.handedness == RD_LEFT ? -1.0 : 1.0;
    const double centre_x = panel.left + width / 2.0;
    const double centre_y = panel.top + height * 0.56;
    const double palm_width = size * 0.34;
    const double palm_height = size * 0.44;
    const double palm_left_x = centre_x - mirror * size * 0.30;
    const double palm_top_y = centre_y - palm_height / 2.0;

    SetTextColor(dc, RGB(150, 158, 175));
    TextOutA(dc, panel.left + 14, panel.top + 10, pose.label.c_str(),
             static_cast<int>(pose.label.size()));

    if (!pose.valid) {
      SetTextColor(dc, RGB(210, 120, 110));
      const char* message = "waiting for motion capture...";
      TextOutA(dc, panel.left + 14, panel.top + 30, message,
               static_cast<int>(strlen(message)));
    }

    // Palm.
    const HBRUSH palm_brush = CreateSolidBrush(RGB(58, 63, 78));
    const HPEN palm_pen = CreatePen(PS_SOLID, 2, RGB(96, 104, 124));
    const HGDIOBJ old_brush = SelectObject(dc, palm_brush);
    const HGDIOBJ old_pen = SelectObject(dc, palm_pen);
    const double palm_far_x = palm_left_x + mirror * palm_width;
    RoundRect(dc, static_cast<int>(std::min(palm_left_x, palm_far_x)),
              static_cast<int>(palm_top_y), static_cast<int>(std::max(palm_left_x, palm_far_x)),
              static_cast<int>(palm_top_y + palm_height), static_cast<int>(size * 0.07),
              static_cast<int>(size * 0.07));
    SelectObject(dc, old_pen);
    SelectObject(dc, old_brush);
    DeleteObject(palm_pen);
    DeleteObject(palm_brush);

    constexpr double kDegToRad = PI / 180.0;
    const int thickness = std::max(3, static_cast<int>(size * 0.035));

    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      const FingerLayout& layout = FINGER_LAYOUTS[finger];
      const FingerFlexion& flexion = pose.fingers[finger];

      double x = palm_left_x + mirror * layout.base_x * palm_width;
      double y = palm_top_y + layout.base_y * palm_height;
      // Mirroring x flips which way "forward" is, so flip the heading too.
      double angle_deg = mirror > 0.0 ? layout.angle_deg : 180.0 - layout.angle_deg;

      double total_deg = 0.0;
      for (unsigned int joint = 0; joint < FJ_LAST; joint++) {
        total_deg += flexion.joint_rad[joint] * 180.0 / PI;
      }
      const double closure = std::max(0.0, std::min(1.0, total_deg / FULL_FLEXION_DEG));
      const COLORREF colour = flexion.valid ? fingerColor(closure) : RGB(80, 84, 96);

      const HPEN pen = CreatePen(PS_SOLID, thickness, colour);
      const HGDIOBJ previous_pen = SelectObject(dc, pen);
      const HBRUSH joint_brush = CreateSolidBrush(colour);
      const HGDIOBJ previous_brush = SelectObject(dc, joint_brush);

      for (unsigned int joint = 0; joint < FJ_LAST; joint++) {
        // Each joint turns the finger further toward the palm, so the chain
        // curls exactly the way a closing finger does.
        angle_deg += mirror * layout.curl * flexion.joint_rad[joint] * 180.0 / PI;
        const double radians = angle_deg * kDegToRad;
        const double length = layout.length * size * SEGMENT_FRACTIONS[joint];
        const double next_x = x + std::cos(radians) * length;
        const double next_y = y + std::sin(radians) * length;

        MoveToEx(dc, static_cast<int>(x), static_cast<int>(y), nullptr);
        LineTo(dc, static_cast<int>(next_x), static_cast<int>(next_y));

        const int knuckle = thickness / 2 + 1;
        Ellipse(dc, static_cast<int>(x) - knuckle, static_cast<int>(y) - knuckle,
                static_cast<int>(x) + knuckle, static_cast<int>(y) + knuckle);

        x = next_x;
        y = next_y;
      }

      SelectObject(dc, previous_brush);
      SelectObject(dc, previous_pen);
      DeleteObject(joint_brush);
      DeleteObject(pen);

      if (flexion.valid) {
        char readout[64];
        snprintf(readout, sizeof(readout), "%-6s %3.0f%%", FINGER_NAMES[finger], closure * 100.0);
        SetTextColor(dc, colour);
        TextOutA(dc, panel.left + 14, static_cast<int>(panel.top + 56 + finger * 18), readout,
                 static_cast<int>(strlen(readout)));
      }
    }
  }

  HWND hwnd_ = nullptr;
  HFONT font_ = nullptr;
};
#endif  // _WIN32

// Opens mocap on the selected gloves and draws their fingers until the time
// runs out or Ctrl-C. Returns false only on a setup failure.
bool runHandView(const std::shared_ptr<Airpack>& airpack, const std::vector<Target>& targets,
                 const Options& options) {
  auto mocap_systems = airpack->getMocapSystems();
  if (!mocap_systems) {
    printf("Airpack::getMocapSystems() returned error code %d: %s.\n",
           static_cast<int>(mocap_systems.error()), toString(mocap_systems.error()).c_str());
    return false;
  }

  // The IK needs a user profile: hand geometry is what turns sensor poses into
  // joint angles. Fall back to a default profile rather than refusing to run,
  // but say so, because the angles will be off for your hand.
  UserProfile profile;
  if (auto username = UserProfileDatabase::getActiveUsername(); username) {
    if (auto stored = UserProfileDatabase::getUserProfile(*username); stored) {
      profile = *stored;
      printf("Using the active user profile.\n");
    } else {
      printf("UserProfileDatabase::getUserProfile() failed with %s -- using a default\n",
             toString(stored.error()).c_str());
      printf("profile, so the angles will not match your hand.\n");
    }
  } else {
    printf("No active user profile (%s) -- using a default profile, so the angles\n",
           toString(username.error()).c_str());
    printf("will not match your hand. Create one in the Dashboard for real numbers.\n");
  }

  std::vector<HandViewSource> sources;
  for (const Target& target : targets) {
    const auto it = mocap_systems->find(target.slot);
    if (it == mocap_systems->end() || it->second == nullptr) {
      printf("No MocapSystem on %s.\n", target.label.c_str());
      continue;
    }

    HandViewSource source;
    source.target = target;
    source.mocap_system = it->second;
    if (auto ret = source.mocap_system->init(); !ret) {
      printf("MocapSystem::init() on %s returned %s.\n", target.label.c_str(),
             toString(ret.error()).c_str());
      continue;
    }
    source.glove = source.mocap_system->getGlove();
    if (source.glove == nullptr) {
      printf("MocapSystem::getGlove() returned nullptr on %s.\n", target.label.c_str());
      continue;
    }
    sources.push_back(source);
  }

  if (sources.empty()) {
    printf("No mocap system to watch.\n");
    return false;
  }

#ifdef _WIN32
  HandWindow window;
  if (options.window && !window.create()) {
    return false;
  }
  if (options.window) {
    printf("\nMove your fingers. Close the window, or Ctrl-C here, to stop.\n\n");
  } else {
    printf("\nMove your fingers. Ctrl-C to stop.\n\n");
  }
#else
  if (options.window) {
    printf("--window needs a Windows build; falling back to the terminal view.\n");
  }
  printf("\nMove your fingers. Ctrl-C to stop.\n\n");
#endif
  std::flush(std::cout);

  const auto start = std::chrono::steady_clock::now();
  // The window can afford a faster refresh than a scrolling terminal.
  const auto period = std::chrono::duration<double>(1.0 / (options.window ? 30.0 : 20.0));
  int lines_drawn = 0;

  for (;;) {
    if (g_interrupted) {
      break;
    }
    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (options.view_time_s > 0.0 && elapsed_s >= options.view_time_s) {
      break;
    }

#ifdef _WIN32
    if (options.window && !window.pump()) {
      break;  // The user closed the window.
    }
#endif

    // Redraw over the previous block rather than scrolling.
    if (!options.window && !options.plain && lines_drawn > 0) {
      printf("\033[%dA", lines_drawn);
    }
    lines_drawn = 0;
    std::vector<HandPose> poses;

    for (HandViewSource& source : sources) {
      auto update_ret = source.mocap_system->update();
      if (!update_ret) {
        const ReturnCode error = update_ret.error();
        // MOCAP_SOLVER_ERROR is documented as intermittent and innocuous, and
        // MOCAP_BAD_SENSOR_DATA shows up on unusual finger positions. Neither
        // is worth aborting a live view for.
        if (error == ReturnCode::MOCAP_SOLVER_ERROR ||
            error == ReturnCode::MOCAP_BAD_SENSOR_DATA || error == ReturnCode::NOT_READY) {
          source.tolerated_errors++;
        } else {
          source.hard_errors++;
        }
      }

      MocapFrame mocap_frame;
      AnimFrame anim_frame;
      if (auto ret = source.mocap_system->addToMocapFrame(&mocap_frame); ret) {
        source.last_transforms = static_cast<int>(mocap_frame.transforms.size());
        if (!DefaultHandIk::addToAnimFrame(source.glove, mocap_frame, profile, &anim_frame)) {
          source.ik_failures++;
        }
      } else {
        source.frame_failures++;
        source.last_frame_error = ret.error();
      }

      HandPose pose;
      pose.label = source.target.label;
      pose.handedness = source.target.handedness;
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        pose.fingers[finger] =
            flexionFor(anim_frame, source.target.handedness, static_cast<Finger>(finger));
        pose.valid = pose.valid || pose.fingers[finger].valid;
      }
      poses.push_back(pose);

      if (options.window) {
        continue;  // The window is the display; keep the terminal quiet.
      }

      printf("  %s\n", source.target.label.c_str());
      printf("  finger    MCP    PIP    DIP   closing\n");
      lines_drawn += 2;
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        drawFingerRow(FINGER_NAMES[finger], pose.fingers[finger]);
        lines_drawn++;
      }
      printf("  hiccups %-4d hard %-4d  mocap transforms %-3d  frame fail %-4d (%s)  ik fail %-4d \n",
             source.tolerated_errors, source.hard_errors, source.last_transforms,
             source.frame_failures, toString(source.last_frame_error).c_str(),
             source.ik_failures);
      lines_drawn++;
    }

#ifdef _WIN32
    if (options.window) {
      window.draw(poses);
    }
#endif
    std::flush(std::cout);
    std::this_thread::sleep_for(period);
  }

#ifdef _WIN32
  window.destroy();
#endif
  printf("\n");
  for (HandViewSource& source : sources) {
    source.mocap_system->shutdown();
  }
  return true;
}

void printInventory(const std::vector<Target>& targets) {
  for (const Target& target : targets) {
    printf("%s\n", target.label.c_str());
    printf("  tactors         : %d\n", static_cast<int>(target.peripheral->tactors.size()));
    printf("  force actuators : %d (held DISENGAGED throughout)\n",
           static_cast<int>(target.peripheral->force_actuators.size()));

    for (const CoverageRegion region : regionsForHand(target.handedness)) {
      const std::vector<const Tactor*> tactors = tactorsInRegion(*target.peripheral, region);
      if (tactors.empty()) {
        continue;
      }
      const Tactor& first = *tactors.front();
      printf("  %-26s %2d tactors  %.2f-%.2f mm  %.1f-%.1f kPa  frame %s\n",
             getName(region).getText().c_str(), static_cast<int>(tactors.size()),
             first.getMinHeightM() * 1000.0f, first.getMaxHeightM() * 1000.0f,
             first.pressure_min_pa / 1000.0f, first.pressure_max_pa / 1000.0f,
             first.getParent().getText().c_str());
    }
    printf("\n");
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }
  if (options.list_scenes) {
    printScenes();
    return 0;
  }

  std::signal(SIGINT, onSigint);

  // Send SDK errors to stderr so a hardware fault is visible rather than silent.
  // The one exception is a redrawing hand view: the log writes to the same
  // terminal and would shred the in-place redraw, so --hand-view without
  // --plain runs quiet.
  if (!options.hand_view || options.plain) {
    Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());
  }

  printf("\nHAPTX TACTILE DEMO\n");
  printf("==================\n");
  printf("Tactors press into the skin -- this is touch, not resistance. No motion\n");
  printf("capture, no tracker, no VR: everything here is commanded straight from\n");
  printf("the hardware definition.\n\n");
  std::flush(std::cout);

  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  const std::shared_ptr<Airpack> airpack =
      haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  if (airpack == nullptr) {
    printf("No Airpack found. Is it powered, connected over USB, and are the USB\n");
    printf("devices bound to WinUSB/USBCCGP in this Wine prefix?\n");
    return 1;
  }

  auto attached_peripherals = airpack->getAttachedPeripherals();
  if (!attached_peripherals) {
    printf("Airpack::getAttachedPeripherals() returned error code %d: %s.\n",
           static_cast<int>(attached_peripherals.error()),
           toString(attached_peripherals.error()).c_str());
    airpack->shutdown();
    return 1;
  }

  // Pick the hand by handedness, not by slot, so it follows the glove.
  std::vector<Target> targets;
  for (const auto& peripheral_it : *attached_peripherals) {
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    if (peripheral == nullptr) {
      continue;
    }
    const auto* glove = dynamic_cast<const Glove*>(peripheral.get());
    const RelativeDirection handedness = glove != nullptr ? glove->handedness : RD_UNKNOWN;
    const std::string label = describePeripheral(peripheral_it.first, *peripheral);

    if (!handWanted(handedness, options.hand)) {
      printf("skipping %s\n", label.c_str());
      continue;
    }
    if (peripheral->is_simulated) {
      printf("skipping %s -- SIMULATED, no hardware behind it\n", label.c_str());
      continue;
    }

    Target target;
    target.slot = peripheral_it.first;
    target.peripheral = peripheral;
    target.handedness = handedness;
    target.label = label;
    targets.push_back(target);
  }
  printf("\n");

  if (targets.empty()) {
    printf("No matching glove to drive. Try --hand both, or check that the glove is\n");
    printf("seated in its connector and not in simulation mode.\n");
    airpack->shutdown();
    return 1;
  }

  printInventory(targets);
  if (options.list_only) {
    airpack->shutdown();
    return 0;
  }

  // --hand-view is a mode of its own: it watches, it does not actuate.
  if (options.hand_view) {
    const bool view_ok = runHandView(airpack, targets, options);
    airpack->shutdown();
    printf("%s\n", view_ok ? "Finished." : "Finished with errors.");
    return view_ok ? 0 : 1;
  }

  // Start from a known state: everything flat.
  if (!renderDirect(*airpack, targets, SlotLevels{}, options.scale)) {
    airpack->shutdown();
    return 1;
  }

  bool ok = true;
  const bool run_vibro = !options.no_vibro &&
                         (options.scene == 0 || options.scene >= FIRST_VIBRO_SCENE);
  const bool run_direct = options.scene == 0 || options.scene < FIRST_VIBRO_SCENE;

  for (int loop = 0; ok && !g_interrupted && (options.loops == 0 || loop < options.loops); loop++) {
    if (options.loops != 1) {
      printf("\n=== run %d%s ===\n", loop + 1,
             options.loops == 0 ? "" : (" of " + std::to_string(options.loops)).c_str());
    }

    if (run_direct) {
      for (int scene = 1; ok && !g_interrupted && scene < FIRST_VIBRO_SCENE; scene++) {
        if (options.scene != 0 && options.scene != scene) {
          continue;
        }
        printf("\n[%d] %s\n", scene, SCENE_NAMES[scene - 1]);
        std::flush(std::cout);

        switch (scene) {
          case 1: ok = sceneRegionTour(*airpack, targets, options); break;
          case 2: ok = sceneIntensityLadder(*airpack, targets, options); break;
          case 3: ok = sceneFingertipWave(*airpack, targets, options); break;
          case 4: ok = sceneFingertipStroke(*airpack, targets, options); break;
          case 5: ok = sceneWholeHandGrip(*airpack, targets, options); break;
          default: break;
        }
        if (!ok && g_interrupted) {
          ok = true;  // Ctrl-C is not a failure.
          break;
        }
      }
      // Leave the inflation path with the hand flat before anything else
      // takes ownership of the Airpack.
      renderDirect(*airpack, targets, SlotLevels{}, options.scale);
    }

    if (ok && !g_interrupted && run_vibro) {
      printf("\n--- handing the Airpack to the vibro thread (%d Hz) ---\n", VIBRO_SAMPLE_RATE_HZ);
      std::flush(std::cout);

      VibroStack vibro;
      if (!vibro.start(airpack)) {
        vibro.stop();
        ok = false;
      } else {
        for (int scene = FIRST_VIBRO_SCENE; ok && !g_interrupted && scene <= SCENE_COUNT;
             scene++) {
          if (options.scene != 0 && options.scene != scene) {
            continue;
          }
          printf("\n[%d] %s\n", scene, SCENE_NAMES[scene - 1]);
          std::flush(std::cout);

          switch (scene) {
            case 6: ok = sceneFrequencySweep(vibro, targets, options); break;
            case 7: ok = sceneWaveformTextures(vibro, targets, options); break;
            case 8: ok = sceneTravellingBuzz(vibro, targets, options); break;
            default: break;
          }
          if (!ok && g_interrupted) {
            ok = true;  // Ctrl-C is not a failure.
            break;
          }
        }
        vibro.stop();
      }
    }
  }

  // Always leave the hand flat, including after Ctrl-C or a failure.
  printf("\nDeflating.\n");
  g_interrupted = 0;  // The final deflate must be allowed to render.
  renderDirect(*airpack, targets, SlotLevels{}, options.scale);
  airpack->shutdown();

  printf("%s\n", ok ? "Finished." : "Finished with errors.");
  std::flush(std::cout);
  return ok ? 0 : 1;
}
