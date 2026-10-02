// fingertip_test.cpp
//
// Bench test for the G1 fingertip tactors -- the microfluidic actuators that
// press into the skin. This is the counterpart to force_actuator_test.cpp:
// that one drives the exoskeleton brakes, which you feel by pushing against
// them; this one drives the thing you feel as touch.
//
// Like the force actuator test it uses nothing but the Airpack: no motion
// capture, no tracker, no OpenVR, and no ContactInterpreter. Tactor heights are
// commanded straight from the hardware definition the Airpack reports, so the
// only thing that has to work is USB -> Airpack -> peripheral connector.
//
// A "fingertip" here is the distal segment of a finger, so the ten regions
// picked by default are the *_THUMB_DISTAL ... *_PINKY_DISTAL CoverageRegions
// on both hands. Each region holds several tactors, and by default all of a
// region's tactors are driven together -- that is what reads as "this
// fingertip". Use --each to walk them one at a time inside the region, and
// --all-regions to include the other finger segments and the palm.
//
// Usage: fingertip_test [options]
//   --list             enumerate tactors by region and exit; actuate nothing
//   --mode <m>         pulse | static | ramp                  (default pulse)
//   --hold  <seconds>  how long each region is driven          (default 2.0)
//   --gap   <seconds>  pause between regions                   (default 0.75)
//   --scale <0..1>     fraction of full inflation height       (default 1.0)
//   --freq  <hz>       pulse rate in --mode pulse              (default 2.0)
//   --loops <n>        how many sweeps to run, 0 = forever     (default 1)
//   --slot  <n>        only test the peripheral on this connector slot
//   --each             drive one tactor at a time, not a whole region
//   --all-regions      every coverage region, not just fingertips
//   --region <text>    only regions whose name contains <text> (e.g. palm)
//   --help
//
// Safety: heights are commanded as a fraction of each tactor's own
// getMaxHeightM(), and DirectPneumaticCalculator clamps to the tactor's height
// and pressure limits on top of that, so --scale 1.0 is full rated inflation
// and not an overpressure. Start lower if you want a gentler first run.

#include <HaptxApi/airpack.h>
#include <HaptxApi/direct_pneumatic_calculator.h>
#include <HaptxApi/enum.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/haptic_frame.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/names.h>
#include <HaptxApi/peripheral.h>
#include <HaptxApi/pneumatic_frame.h>
#include <HaptxApi/tactor.h>

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
#include <vector>

using namespace HaptxApi;

namespace {

// How often set points are pushed to the Airpack [Hz]. The starter project caps
// meaningful updates at 120 Hz; 90 is plenty for a pulse you can feel.
constexpr double RENDER_RATE_HZ = 90.0;

// Spelled out rather than taken from <cmath>: MSVC only defines M_PI when
// _USE_MATH_DEFINES is set before the include, and this file should compile
// without needing that flag on the command line.
constexpr double PI = 3.14159265358979323846;

// Set from the SIGINT handler so a Ctrl-C still deflates the tactors.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

// How a region's inflation varies over the hold.
enum class Mode {
  PULSE,   // Sine between deflated and --scale. Easiest to localize by feel.
  STATIC,  // Hold at --scale for the whole duration.
  RAMP     // Deflated to --scale linearly across the duration.
};

struct Options {
  bool list_only = false;
  bool each = false;
  bool all_regions = false;
  Mode mode = Mode::PULSE;
  double hold_s = 2.0;
  double gap_s = 0.75;
  double freq_hz = 2.0;
  float scale = 1.0f;
  int loops = 1;         // 0 means "until interrupted"
  int slot = -1;         // -1 means "every slot"
  std::string region;    // empty means "no substring filter"
};

void printUsage() {
  printf(
      "Usage: fingertip_test [options]\n"
      "  --list             enumerate tactors by region and exit; actuate nothing\n"
      "  --mode <m>         pulse | static | ramp                  (default pulse)\n"
      "  --hold  <seconds>  how long each region is driven          (default 2.0)\n"
      "  --gap   <seconds>  pause between regions                   (default 0.75)\n"
      "  --scale <0..1>     fraction of full inflation height       (default 1.0)\n"
      "  --freq  <hz>       pulse rate in --mode pulse              (default 2.0)\n"
      "  --loops <n>        how many sweeps to run, 0 = forever     (default 1)\n"
      "  --slot  <n>        only test the peripheral on this connector slot\n"
      "  --each             drive one tactor at a time, not a whole region\n"
      "  --all-regions      every coverage region, not just fingertips\n"
      "  --region <text>    only regions whose name contains <text> (e.g. palm)\n"
      "  --help\n");
}

std::string toLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

// Parses the command line. Returns false if the program should not run.
bool parseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; i++) {
    const char* arg = argv[i];
    const bool has_value = i + 1 < argc;

    if (strcmp(arg, "--list") == 0) {
      options->list_only = true;
    } else if (strcmp(arg, "--each") == 0) {
      options->each = true;
    } else if (strcmp(arg, "--all-regions") == 0) {
      options->all_regions = true;
    } else if (strcmp(arg, "--mode") == 0 && has_value) {
      const std::string mode = toLower(argv[++i]);
      if (mode == "pulse") {
        options->mode = Mode::PULSE;
      } else if (mode == "static") {
        options->mode = Mode::STATIC;
      } else if (mode == "ramp") {
        options->mode = Mode::RAMP;
      } else {
        printf("Unrecognized --mode: %s (want pulse, static or ramp)\n", mode.c_str());
        return false;
      }
    } else if (strcmp(arg, "--hold") == 0 && has_value) {
      options->hold_s = atof(argv[++i]);
    } else if (strcmp(arg, "--gap") == 0 && has_value) {
      options->gap_s = atof(argv[++i]);
    } else if (strcmp(arg, "--freq") == 0 && has_value) {
      options->freq_hz = atof(argv[++i]);
    } else if (strcmp(arg, "--scale") == 0 && has_value) {
      options->scale = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--loops") == 0 && has_value) {
      options->loops = atoi(argv[++i]);
    } else if (strcmp(arg, "--slot") == 0 && has_value) {
      options->slot = atoi(argv[++i]);
    } else if (strcmp(arg, "--region") == 0 && has_value) {
      options->region = toLower(argv[++i]);
    } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      printUsage();
      return false;
    } else {
      printf("Unrecognized argument: %s\n\n", arg);
      printUsage();
      return false;
    }
  }

  if (options->hold_s < 0.0 || options->gap_s < 0.0 || options->loops < 0) {
    printf("--hold, --gap and --loops must not be negative.\n");
    return false;
  }
  if (options->scale < 0.0f || options->scale > 1.0f) {
    printf("--scale must be between 0 and 1.\n");
    return false;
  }
  return true;
}

// The coverage regions to walk, in the order they should be tested. Fingertips
// are the distal segments; the SDK's own getName() supplies the text, so this
// matches whatever the hardware definition calls them.
std::vector<CoverageRegion> candidateRegions(const Options& options) {
  if (options.all_regions) {
    std::vector<CoverageRegion> regions;
    for (int i = 0; i < static_cast<int>(CoverageRegion::LAST); i++) {
      regions.push_back(static_cast<CoverageRegion>(i));
    }
    return regions;
  }
  return {CoverageRegion::RIGHT_THUMB_DISTAL,  CoverageRegion::RIGHT_INDEX_DISTAL,
          CoverageRegion::RIGHT_MIDDLE_DISTAL, CoverageRegion::RIGHT_RING_DISTAL,
          CoverageRegion::RIGHT_PINKY_DISTAL,  CoverageRegion::LEFT_THUMB_DISTAL,
          CoverageRegion::LEFT_INDEX_DISTAL,   CoverageRegion::LEFT_MIDDLE_DISTAL,
          CoverageRegion::LEFT_RING_DISTAL,    CoverageRegion::LEFT_PINKY_DISTAL};
}

// Names a peripheral for logging, e.g. "slot 2 (RD_RIGHT, G1 medium right Glove)".
std::string describePeripheral(int slot, const Peripheral& peripheral) {
  std::string handedness = "non-glove";
  if (const auto* glove = dynamic_cast<const Glove*>(&peripheral)) {
    handedness = toString(glove->handedness);
  }
  return "slot " + std::to_string(slot) + " (" + handedness + ", " + peripheral.casual_name + ")";
}

// True if a peripheral can actually be driven. A simulated peripheral has no
// hardware behind it, so rendering to it does nothing.
bool isDrivable(const std::shared_ptr<Peripheral>& peripheral, const Options& options, int slot) {
  return peripheral != nullptr && !peripheral->is_simulated &&
         (options.slot < 0 || options.slot == slot);
}

// One step of the sweep: a set of tactors on one peripheral, driven together.
struct Step {
  int slot = 0;
  std::vector<int> tactor_ids;
  std::string label;
};

// Decides, per tactor, whether it is the one being driven this frame.
using SelectPredicate = std::function<bool(int slot, const Tactor&)>;

// Builds one PneumaticFrame covering every drivable peripheral and renders it.
//
// Every tactor is named on every frame -- the ones that should be flat are
// commanded to their minimum height rather than left out -- so a frame fully
// describes the hardware state and nothing stays inflated by omission. Force
// actuators are explicitly held DISENGAGED for the same reason: this test is
// about the tactors, and the brakes should be out of the way.
//
// @param level How far to inflate the selected tactors, 0 (flat) to 1 (full).
bool renderLevel(Airpack& airpack, const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                 const Options& options, const SelectPredicate& is_selected, float level) {
  PneumaticFrame pneumatic_frame;

  for (const auto& peripheral_it : peripherals) {
    const int slot = peripheral_it.first;
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    if (!isDrivable(peripheral, options, slot)) {
      continue;
    }

    HapticFrame haptic_frame;
    for (const Tactor& tactor : peripheral->tactors) {
      const float min_m = tactor.getMinHeightM();
      const float max_m = tactor.getMaxHeightM();
      const float fraction =
          (tactor.isEnabled() && is_selected(slot, tactor)) ? level * options.scale : 0.0f;
      haptic_frame.tactor_heights_m[tactor.getId()] = min_m + fraction * (max_m - min_m);
    }
    for (const ForceActuator& force_actuator : peripheral->force_actuators) {
      haptic_frame.force_actuator_states[force_actuator.getId()] =
          PassiveForceActuator::State::DISENGAGED;
    }

    if (!DirectPneumaticCalculator::addToPneumaticFrame(*peripheral, haptic_frame, airpack,
                                                        &pneumatic_frame)) {
      printf("DirectPneumaticCalculator::addToPneumaticFrame() failed for %s.\n",
             describePeripheral(slot, *peripheral).c_str());
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

// Drives a selection for a while, shaping the inflation according to --mode.
// Returns false on a render failure or Ctrl-C.
bool driveSelection(Airpack& airpack,
                    const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                    const Options& options, const SelectPredicate& is_selected, double duration_s,
                    bool active) {
  const auto period = std::chrono::duration<double>(1.0 / RENDER_RATE_HZ);
  const auto start = std::chrono::steady_clock::now();

  for (;;) {
    if (g_interrupted) {
      return false;
    }

    const double elapsed_s = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
    float level = 0.0f;
    if (active) {
      switch (options.mode) {
        case Mode::PULSE:
          level = static_cast<float>(
              (1.0 - std::cos(2.0 * PI * options.freq_hz * elapsed_s)) / 2.0);
          break;
        case Mode::STATIC:
          level = 1.0f;
          break;
        case Mode::RAMP:
          level = duration_s > 0.0 ? static_cast<float>(
                                         std::min(1.0, elapsed_s / duration_s))
                                   : 1.0f;
          break;
      }
    }

    if (!renderLevel(airpack, peripherals, options, is_selected, level)) {
      return false;
    }
    if (elapsed_s >= duration_s) {
      return true;
    }
    std::this_thread::sleep_for(period);
  }
}

// Prints the tactor inventory and builds the sweep.
std::vector<Step> inventory(const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                            const Options& options) {
  const std::vector<CoverageRegion> regions = candidateRegions(options);
  std::vector<Step> steps;

  for (const auto& peripheral_it : peripherals) {
    const int slot = peripheral_it.first;
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    if (peripheral == nullptr) {
      printf("Slot %d: null peripheral.\n\n", slot);
      continue;
    }

    printf("%s\n", describePeripheral(slot, *peripheral).c_str());
    printf("  tactors         : %d\n", static_cast<int>(peripheral->tactors.size()));
    printf("  force actuators : %d (held DISENGAGED by this test)\n",
           static_cast<int>(peripheral->force_actuators.size()));

    if (peripheral->is_simulated) {
      printf("  SIMULATED -- no hardware behind this peripheral, so it will not be driven.\n\n");
      continue;
    }
    if (options.slot >= 0 && options.slot != slot) {
      printf("  skipped (--slot %d)\n\n", options.slot);
      continue;
    }

    int matched_regions = 0;
    for (const CoverageRegion region : regions) {
      const HaptxName region_name = getName(region);
      const std::string region_text = region_name.getText();
      if (!options.region.empty() &&
          toLower(region_text).find(options.region) == std::string::npos) {
        continue;
      }

      // Collect this region's drivable tactors, and their limits for the report.
      std::vector<int> ids;
      float min_height_m = 0.0f;
      float max_height_m = 0.0f;
      float min_pressure_pa = 0.0f;
      float max_pressure_pa = 0.0f;
      int disabled = 0;
      for (const Tactor& tactor : peripheral->tactors) {
        if (tactor.coverage_region != region_name) {
          continue;
        }
        if (!tactor.isEnabled()) {
          disabled++;
          continue;
        }
        if (ids.empty()) {
          min_height_m = tactor.getMinHeightM();
          max_height_m = tactor.getMaxHeightM();
          min_pressure_pa = tactor.pressure_min_pa;
          max_pressure_pa = tactor.pressure_max_pa;
        }
        ids.push_back(tactor.getId());
      }
      if (ids.empty()) {
        continue;  // This region is not present on this peripheral.
      }
      matched_regions++;

      printf("  %-24s %2d tactors  %.2f-%.2f mm  %.1f-%.1f kPa%s\n", region_text.c_str(),
             static_cast<int>(ids.size()), min_height_m * 1000.0f, max_height_m * 1000.0f,
             min_pressure_pa / 1000.0f, max_pressure_pa / 1000.0f,
             disabled > 0 ? "  (some disabled)" : "");

      const std::string where = describePeripheral(slot, *peripheral) + " " + region_text;
      if (options.each) {
        for (const int id : ids) {
          Step step;
          step.slot = slot;
          step.tactor_ids = {id};
          step.label = where + " tactor " + std::to_string(id);
          steps.push_back(step);
        }
      } else {
        Step step;
        step.slot = slot;
        step.tactor_ids = ids;
        step.label = where + " (" + std::to_string(ids.size()) + " tactors)";
        steps.push_back(step);
      }
    }

    if (matched_regions == 0) {
      printf("  no matching coverage regions found on this peripheral.\n");
      printf("  Run with --all-regions --list to see every region it does report.\n");
    }
    printf("\n");
  }

  return steps;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }

  std::signal(SIGINT, onSigint);

  // Send SDK errors to stderr so a hardware fault is visible rather than silent.
  Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());

  printf("\nHAPTX FINGERTIP TACTOR TEST\n");
  printf("===========================\n");
  printf("Tactors press into the skin -- this is touch, not resistance. Rest your\n");
  printf("fingers naturally and keep still; each fingertip is driven in turn and\n");
  printf("announced before it fires.\n\n");
  std::flush(std::cout);

  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  // No MocapSystem::init(), no TrackerLoader, no OpenvrWrapper, no
  // ContactInterpreter: driving tactors directly needs none of them.
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

  const std::map<int, std::shared_ptr<Peripheral>>& peripherals = *attached_peripherals;
  if (peripherals.empty()) {
    printf("The Airpack reports no attached peripherals. Check the glove connectors.\n");
    airpack->shutdown();
    return 1;
  }

  printf("Attached peripherals: %d\n\n", static_cast<int>(peripherals.size()));
  const std::vector<Step> steps = inventory(peripherals, options);

  if (steps.empty()) {
    printf("Nothing to test.\n");
    airpack->shutdown();
    return 1;
  }
  printf("%d steps to run, %.1fs each.\n", static_cast<int>(steps.size()), options.hold_s);
  std::flush(std::cout);

  if (options.list_only) {
    airpack->shutdown();
    return 0;
  }

  // Start from a known state: everything flat.
  const SelectPredicate none_selected = [](int, const Tactor&) { return false; };
  if (!renderLevel(*airpack, peripherals, options, none_selected, 0.0f)) {
    airpack->shutdown();
    return 1;
  }

  bool ok = true;
  for (int loop = 0; ok && !g_interrupted && (options.loops == 0 || loop < options.loops); loop++) {
    if (options.loops != 1) {
      printf("\n--- sweep %d%s ---\n", loop + 1,
             options.loops == 0 ? "" : (" of " + std::to_string(options.loops)).c_str());
    }

    for (const Step& step : steps) {
      if (g_interrupted) {
        break;
      }

      printf("DRIVE   %s\n", step.label.c_str());
      std::flush(std::cout);

      const SelectPredicate this_step = [&step](int slot, const Tactor& tactor) {
        return slot == step.slot && std::find(step.tactor_ids.begin(), step.tactor_ids.end(),
                                              tactor.getId()) != step.tactor_ids.end();
      };
      if (!driveSelection(*airpack, peripherals, options, this_step, options.hold_s, true)) {
        ok = !g_interrupted;
        break;
      }

      if (!driveSelection(*airpack, peripherals, options, none_selected, options.gap_s, false)) {
        ok = !g_interrupted;
        break;
      }
    }
  }

  // Always leave the hardware flat, including after Ctrl-C or a failure.
  printf("\nDeflating all tactors.\n");
  g_interrupted = 0;  // The final deflate must be allowed to render.
  renderLevel(*airpack, peripherals, options, none_selected, 0.0f);
  airpack->shutdown();

  printf("%s\n", ok ? "Finished." : "Finished with errors.");
  std::flush(std::cout);
  return ok ? 0 : 1;
}
