// force_actuator_test.cpp
//
// Bench test for the G1 force-feedback actuators (the pneumatic brakes in the
// exoskeleton) on the fingers and the palm.
//
// It deliberately uses nothing but the Airpack: no motion capture, no tracker
// loading, no OpenVR. Force actuators are driven straight off the hardware
// definition the Airpack reports for each attached peripheral, so the only
// thing that has to work is USB -> Airpack -> peripheral connector.
//
// What it does:
//   1. Detects the Airpack and lists every attached peripheral, every force
//      actuator on it, and the body parts each actuator restricts.
//   2. Sweeps the actuators one at a time: engage one, hold, release, next.
//   3. Optionally engages all of them together at the end.
//
// How to feel it: a ForceActuator is *passive* -- "resistive, meaning they can
// resist a user's motion but cannot move the user on their own". It cannot push
// your finger; it can only stop it from closing. So while an actuator is
// engaged, try to curl the finger it names -- it should lock. When it
// disengages the finger frees up again.
//
// Note that this bypasses the ContactInterpreter, so none of its force-feedback
// troubleshooting applies here: registerForceActuator(), addContact(),
// commit(), setForceFeedbackEnabled() and the force filter are all out of the
// picture. Nothing stands between this program and the valves except
// DirectPneumaticCalculator. If an actuator does not engage, the interesting
// numbers are the ones the inventory prints: `enabled` and the actuation
// pressure, which must be strictly positive.
//
// Usage: force_actuator_test [options]
//   --list             enumerate hardware and exit; actuate nothing
//   --hold  <seconds>  how long each actuator stays engaged   (default 2.0)
//   --gap   <seconds>  pause between actuators                (default 0.75)
//   --loops <n>        how many times to run the sweep, 0 = forever (default 1)
//   --slot  <n>        only test the peripheral on this connector slot
//   --sweep-only       skip the final "everything at once" phase
//   --help

#include <HaptxApi/airpack.h>
#include <HaptxApi/direct_pneumatic_calculator.h>
#include <HaptxApi/enum.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/haptic_frame.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/peripheral.h>
#include <HaptxApi/pneumatic_frame.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <csignal>
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

// How often set points are pushed to the Airpack while holding a state [Hz].
constexpr double RENDER_RATE_HZ = 60.0;

// Set from the SIGINT handler so a Ctrl-C still releases the actuators.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

// Options, as parsed from the command line.
struct Options {
  bool list_only = false;
  bool sweep_only = false;
  double hold_s = 2.0;
  double gap_s = 0.75;
  int loops = 1;   // 0 means "until interrupted"
  int slot = -1;   // -1 means "every slot"
};

void printUsage() {
  printf(
      "Usage: force_actuator_test [options]\n"
      "  --list             enumerate hardware and exit; actuate nothing\n"
      "  --hold  <seconds>  how long each actuator stays engaged   (default 2.0)\n"
      "  --gap   <seconds>  pause between actuators                (default 0.75)\n"
      "  --loops <n>        how many sweeps to run, 0 = forever    (default 1)\n"
      "  --slot  <n>        only test the peripheral on this connector slot\n"
      "  --sweep-only       skip the final \"everything at once\" phase\n"
      "  --help\n");
}

// Parses the command line. Returns false if the program should not run.
bool parseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; i++) {
    const char* arg = argv[i];
    const bool has_value = i + 1 < argc;

    if (strcmp(arg, "--list") == 0) {
      options->list_only = true;
    } else if (strcmp(arg, "--sweep-only") == 0) {
      options->sweep_only = true;
    } else if (strcmp(arg, "--hold") == 0 && has_value) {
      options->hold_s = atof(argv[++i]);
    } else if (strcmp(arg, "--gap") == 0 && has_value) {
      options->gap_s = atof(argv[++i]);
    } else if (strcmp(arg, "--loops") == 0 && has_value) {
      options->loops = atoi(argv[++i]);
    } else if (strcmp(arg, "--slot") == 0 && has_value) {
      options->slot = atoi(argv[++i]);
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
  return true;
}

// Names the body parts a force actuator resists, e.g. "RIGHT_INDEX_MEDIAL,
// RIGHT_INDEX_DISTAL". This is what tells you which finger to try to curl.
std::string describeRestrictions(const ForceActuator& force_actuator) {
  std::vector<std::string> body_parts;
  for (const auto& restriction : force_actuator.getRestrictions()) {
    body_parts.push_back(toString(restriction.first));
  }
  std::sort(body_parts.begin(), body_parts.end());

  std::string description;
  for (const std::string& body_part : body_parts) {
    if (!description.empty()) {
      description += ", ";
    }
    description += body_part;
  }
  return description.empty() ? "no declared restrictions" : description;
}

// Names a peripheral for logging, e.g. "slot 0 (RD_RIGHT, G1 medium right Glove)".
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

// Identifies one force actuator among all the attached peripherals.
struct ActuatorRef {
  int slot = 0;
  std::shared_ptr<Peripheral> peripheral;
  int actuator_id = 0;
  std::string label;
};

// Decides, per actuator, whether it should be engaged this frame.
using EngagePredicate = std::function<bool(int slot, const ForceActuator&)>;

// Builds one PneumaticFrame covering every drivable peripheral and renders it.
//
// Every force actuator is named explicitly on every frame -- the ones that
// should be off are sent DISENGAGED rather than left out -- so a frame fully
// describes the state of the hardware and nothing stays engaged by omission.
// Tactors are deliberately never addressed, so they keep whatever state they
// were in; this program does not touch them.
bool renderStates(Airpack& airpack, const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                  const Options& options, const EngagePredicate& should_engage) {
  PneumaticFrame pneumatic_frame;

  for (const auto& peripheral_it : peripherals) {
    const int slot = peripheral_it.first;
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    if (!isDrivable(peripheral, options, slot)) {
      continue;
    }

    HapticFrame haptic_frame;
    for (const ForceActuator& force_actuator : peripheral->force_actuators) {
      const bool engaged = force_actuator.isEnabled() && should_engage(slot, force_actuator);
      haptic_frame.force_actuator_states[force_actuator.getId()] =
          engaged ? PassiveForceActuator::State::ENGAGED
                  : PassiveForceActuator::State::DISENGAGED;
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

// Holds a state for a while, re-rendering it at RENDER_RATE_HZ so the Airpack
// keeps the set points fresh. Returns false on a render failure or Ctrl-C.
bool holdStates(Airpack& airpack, const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                const Options& options, const EngagePredicate& should_engage, double duration_s) {
  const auto period = std::chrono::duration<double>(1.0 / RENDER_RATE_HZ);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);

  do {
    if (g_interrupted) {
      return false;
    }
    if (!renderStates(airpack, peripherals, options, should_engage)) {
      return false;
    }
    std::this_thread::sleep_for(period);
  } while (std::chrono::steady_clock::now() < deadline);

  return true;
}

// Prints the hardware inventory and collects the actuators that will be tested.
std::vector<ActuatorRef> inventory(const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                                   const Options& options) {
  std::vector<ActuatorRef> actuators;

  for (const auto& peripheral_it : peripherals) {
    const int slot = peripheral_it.first;
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    if (peripheral == nullptr) {
      printf("Slot %d: null peripheral.\n\n", slot);
      continue;
    }

    printf("%s\n", describePeripheral(slot, *peripheral).c_str());
    printf("  mechanical id   : %s\n", peripheral->mechanical_id.c_str());
    printf("  force actuators : %d\n", static_cast<int>(peripheral->force_actuators.size()));
    printf("  tactors         : %d (not touched by this test)\n",
           static_cast<int>(peripheral->tactors.size()));

    if (peripheral->is_simulated) {
      printf("  SIMULATED -- no hardware behind this peripheral, so it will not be driven.\n");
      printf("  If you expected real hardware here, check the Dashboard's Hardware Simulation\n");
      printf("  setting and that the glove is seated in its connector.\n\n");
      continue;
    }
    if (options.slot >= 0 && options.slot != slot) {
      printf("  skipped (--slot %d)\n\n", options.slot);
      continue;
    }

    for (const ForceActuator& force_actuator : peripheral->force_actuators) {
      const std::string restrictions = describeRestrictions(force_actuator);
      printf("  actuator %2d : port %2d  valve %2d  %6.1f kPa  %s  -> %s\n", force_actuator.getId(),
             force_actuator.getPort(), force_actuator.getValveId(),
             force_actuator.getActuationPressurePa() / 1000.0f,
             force_actuator.isEnabled() ? "enabled " : "DISABLED", restrictions.c_str());

      if (!force_actuator.isEnabled()) {
        continue;  // Hardware says this one must never engage.
      }

      ActuatorRef actuator_ref;
      actuator_ref.slot = slot;
      actuator_ref.peripheral = peripheral;
      actuator_ref.actuator_id = force_actuator.getId();
      actuator_ref.label = describePeripheral(slot, *peripheral) + " actuator " +
                           std::to_string(force_actuator.getId()) + " [" + restrictions + "]";
      actuators.push_back(actuator_ref);
    }
    printf("\n");
  }

  return actuators;
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

  printf("\nHAPTX FORCE ACTUATOR TEST\n");
  printf("=========================\n");
  printf("Force actuators are passive brakes: they resist your fingers closing,\n");
  printf("they cannot move them. While one is engaged, try to curl the finger it\n");
  printf("names -- it should lock up, then free again when it releases.\n\n");
  std::flush(std::cout);

  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  // No MocapSystem::init(), no TrackerLoader, no OpenvrWrapper: driving force
  // actuators needs none of them.
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
  const std::vector<ActuatorRef> actuators = inventory(peripherals, options);

  if (actuators.empty()) {
    printf("No drivable force actuators found -- nothing to test.\n");
    airpack->shutdown();
    return 1;
  }
  printf("%d force actuators will be tested.\n", static_cast<int>(actuators.size()));
  std::flush(std::cout);

  if (options.list_only) {
    airpack->shutdown();
    return 0;
  }

  // Start from a known state: everything released.
  const EngagePredicate none_engaged = [](int, const ForceActuator&) { return false; };
  if (!renderStates(*airpack, peripherals, options, none_engaged)) {
    airpack->shutdown();
    return 1;
  }

  bool ok = true;
  for (int loop = 0; ok && !g_interrupted && (options.loops == 0 || loop < options.loops); loop++) {
    if (options.loops != 1) {
      printf("\n--- sweep %d%s ---\n", loop + 1,
             options.loops == 0 ? "" : (" of " + std::to_string(options.loops)).c_str());
    }

    // One at a time, so each actuator can be identified by feel on its own.
    for (const ActuatorRef& actuator : actuators) {
      if (g_interrupted) {
        break;
      }

      printf("ENGAGE  %s\n", actuator.label.c_str());
      std::flush(std::cout);

      const EngagePredicate this_one = [&actuator](int slot, const ForceActuator& force_actuator) {
        return slot == actuator.slot && force_actuator.getId() == actuator.actuator_id;
      };
      if (!holdStates(*airpack, peripherals, options, this_one, options.hold_s)) {
        ok = !g_interrupted;
        break;
      }

      printf("release\n");
      std::flush(std::cout);
      if (!holdStates(*airpack, peripherals, options, none_engaged, options.gap_s)) {
        ok = !g_interrupted;
        break;
      }
    }
  }

  // Everything at once: this is the "make a fist and it stops" test.
  if (ok && !g_interrupted && !options.sweep_only) {
    printf("\nALL ACTUATORS ENGAGED -- try to close your hand.\n");
    std::flush(std::cout);

    const EngagePredicate all_engaged = [](int, const ForceActuator&) { return true; };
    ok = holdStates(*airpack, peripherals, options, all_engaged, std::max(options.hold_s, 3.0));
    if (g_interrupted) {
      ok = true;
    }
  }

  // Always leave the hardware released, including after Ctrl-C or a failure.
  printf("\nReleasing all actuators.\n");
  g_interrupted = 0;  // The final release must be allowed to render.
  renderStates(*airpack, peripherals, options, none_engaged);
  airpack->shutdown();

  printf("%s\n", ok ? "Finished." : "Finished with errors.");
  std::flush(std::cout);
  return ok ? 0 : 1;
}
