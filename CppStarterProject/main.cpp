#include <HaptxApi/direct_pneumatic_calculator.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/names.h>
#include <HaptxApi/openvr_wrapper.h>
#include <HaptxApi/tracker_loader.h>
#include <HaptxApi/world_transform_calculator.h>

#include <chrono>
#include <iostream>
#include <limits>


using namespace HaptxApi;

// How long to run the simulation for [s].
const double PROGRAM_DURATION_S = 60.0;

// Period between simulation updates (like a tick/frame rate) [s]. 120 Hz is as fast as we can go.
const float UPDATE_PERIOD_S = 1.0f / 90.0f;

// The frequency that the tactors will oscillate at. The wave form is a sin wave between 0 and full
// inflation.
const float TACTOR_HEIGHT_OSCILLATION_FREQUENCY_1_S = 3.0f;

// The maximum distance from another locating feature at which tactile feedback kicks in.
const float MAX_PROXIMITY_M = 0.05f;

// Prompts and waits for user to quit by pressing Enter.
void waitForInput() {
  printf("Press Enter to quit.\n");
  std::flush(std::cout);
  std::cin.get();
}

// The wave function being rendered on tactors.
//
// @param time_s The independent variable of the function producing our wave [s].
// @returns The output of the wave function.
float getWaveOutput(float time_s) {
  return static_cast<float>(
      (std::sin(M_PI * TACTOR_HEIGHT_OSCILLATION_FREQUENCY_1_S * time_s) + 1.0f) / 2.0f);
}

// The haptic logic that determines how this application feels (per-frame).
//
// If real hardware is available this code scales the amplitude of a vibrational sensation on each
// of the fingers (and palm) by how close they are to another finger (or palm).
//
// If only simulated hardware is available this code does one of two things per-hand:
// - If a VR controller is available, it inflates tactors based on the controller's trigger
//   axis.
// - If a VR controller is not available, it plays a vibrational pattern on all tactors.

// @param airpack An initialized interface to a Airpack (can be null).
// @param time_s The amount of time the application has been running.
// @returns True if everything went well, false otherwise.
bool updateHaptics(const std::shared_ptr<Airpack>& airpack, float time_s) {
  // if (!OpenvrWrapper::isReady()) {
  //   printf("OpenvrWrapper::isReady() returned false.\n");
  //   return false;
  // }

  auto mocap_systems = airpack ? airpack->getMocapSystems() : Unexpected{ReturnCode::NULL_ARGUMENT};
  if (!mocap_systems) {
    printf("Airpack::getMocapSystems() failed with error code %d: %s",
           static_cast<int>(mocap_systems.error()), toString(mocap_systems.error()).c_str());
    return false;
  }
  MocapFrame l_mocap_frame;  // Contains mocap data relative to peripherals.
  MocapFrame w_mocap_frame;  // Contains mocap data in world space.
  for (const auto& mocap_system_it : *mocap_systems) {
    std::shared_ptr<MocapSystem> mocap_system = mocap_system_it.second;
    if (mocap_system == nullptr) {
      continue;
    }

    if (!mocap_system->isReady()) {
      printf("MocapSystem::isReady() returned false.\n");
      return false;
    }

    std::shared_ptr<Glove> glove = mocap_system->getGlove();
    if (glove == nullptr) {
      printf("MocapSystem::getGlove() returned nullptr.\n");
      return false;
    }

    auto mocap_update_ret = mocap_system->update();
    bool got_mocap_update = mocap_update_ret.has_value();
    // Add sensor transforms to mocap frame.
    if (!mocap_update_ret &&
        mocap_update_ret.error() !=
            ReturnCode::MOCAP_SOLVER_ERROR) {  // Ignore benign solver warnings.

      // We also want to ignore MOCAP_BAD_SENSOR_DATA, which can be emitted with
      // certain unusual finger positions.  This error is much more unusual than
      // MOCAP_SOLVER_ERROR, so it's worth noting it in the print statement, but
      // we don't want to completely abort on a single instance of it.
      if (mocap_update_ret.error() != ReturnCode::MOCAP_BAD_SENSOR_DATA &&
          (glove->is_simulated && mocap_update_ret.error() != ReturnCode::NOT_READY)) {
        printf("MocapSystem::update() returned error code %d: %s.\n",
               static_cast<int>(mocap_update_ret.error()),
               toString(mocap_update_ret.error()).c_str());
        return false;
      }
    }

    if (!glove->is_simulated || got_mocap_update) {
      if (auto ret = mocap_system->addToMocapFrame(&l_mocap_frame); !ret) {
        printf("MocapSystem::addToMocapFrame() returned error code %d: %s.\n",
               static_cast<int>(ret.error()), toString(ret.error()).c_str());
        return false;
      }

      const HaptxApi::HaptxName MCP3_NAME = HaptxApi::getName(
          HaptxApi::getBodyPartJoint(mocap_system->getGlove()->handedness,
                                     HaptxApi::Finger::F_MIDDLE, HaptxApi::FingerJoint::FJ_JOINT1));
      if (glove->is_simulated) {
        // if (auto controller_pos = glove ? OpenvrWrapper::getControllerTransform(glove->handedness)
        //                                 : Unexpected{ReturnCode::UNKNOWN_ERROR}) {
        //   w_mocap_frame.transforms.insert({TransformContext(WORLD, MCP3_NAME), *controller_pos});
        // } else {
          w_mocap_frame.transforms.insert(
              {TransformContext(WORLD, MCP3_NAME), Transform::identity()});
        // }
      } else {
        // Get list of serial numbers for trackers attached to this glove.
        std::map<int, std::string> tracker_serial_numbers{};
        if (auto tracker_loader_ret = HaptxApi::TrackerLoader::getAssociatedTrackerSerialNumbers(
                mocap_system->getSerialNumberAsString());
            !tracker_loader_ret) {
          // This isn't a critical error, we'll just use the default position for our tracking
          // origin (MCP3).
          printf(
              "updateHaptics(): TrackerLoader::getAssociatedTrackerSerialNumbers() "
              "returned error code %d: %s.\n",
              static_cast<int>(tracker_loader_ret.error()),
              HaptxApi::toString(tracker_loader_ret.error()).c_str());
        } else {
          tracker_serial_numbers = *tracker_loader_ret;
        }

        // Get the position of each tracker based on their serial numbers.
        std::vector<std::string> tracker_errors{};
        std::map<int, HaptxApi::Transform> tracker_transforms{};
        for (const auto& tracker : tracker_serial_numbers) {
          if (auto ret = HaptxApi::OpenvrWrapper::getTrackerTransform(tracker.second); !ret) {
            // Keep track of errors but don't log yet.
            tracker_errors.push_back(
                std::string("updateHaptics(): HaptxApi::OpenvrWrapper::getTrackerTransform() for "
                            "tracker with serno ") +
                tracker.second + " returned error code " +
                std::to_string(static_cast<int>(ret.error())) + ": " +
                HaptxApi::toString(ret.error()));
          } else {
            tracker_transforms.emplace(tracker.first, ret.value());
          }
        }

        // If this is our first time without any trackers, log our errors.
        static bool warned_about_trackers = false;
        if (tracker_transforms.empty() && !warned_about_trackers) {
          printf(
              "updateHaptics(): No trackers available for the %s hand. Lookup failed with the "
              "following errors:\n",
              mocap_system->getGlove()->casual_name.c_str());
          for (const auto& error : tracker_errors) {
            printf("%s\n", error.c_str());
          }
          warned_about_trackers = true;
        }
        // Calculate MCP3 position based on our trackers.
        if (auto w_mcp3_ret = HaptxApi::WorldTransformCalculator::calculateMcp3WorldTf(
                glove, l_mocap_frame, tracker_transforms);
            !w_mcp3_ret) {
          // Only log for the error of not having trackers connected once.
          static bool warned_about_trackers_in_wtc = false;
          if (!tracker_transforms.empty() ||
              (!warned_about_trackers_in_wtc && tracker_transforms.empty())) {
            printf(
                "updateHaptics(): WorldTransformCalculator::calculateMcp3WorldTf() failed with "
                "error code %d: %s\n",
                static_cast<int>(w_mcp3_ret.error()),
                HaptxApi::toString(w_mcp3_ret.error()).c_str());
          }
          // Log an error, but just use the default transform as our mcp3 position.
          w_mocap_frame.transforms.insert(
              {TransformContext(WORLD, MCP3_NAME), Transform::identity()});
        } else {
          w_mocap_frame.transforms.insert({TransformContext(WORLD, MCP3_NAME), *w_mcp3_ret});
        }
      }
    }

    // Convert all local transforms to world space.
    for (const auto& l_mocap_datum : l_mocap_frame.transforms) {
      const auto& w_parent_it =
          w_mocap_frame.transforms.find(TransformContext(WORLD, l_mocap_datum.first.parent));
      if (w_parent_it != w_mocap_frame.transforms.end()) {
        w_mocap_frame.transforms.insert({TransformContext(WORLD, l_mocap_datum.first.child),
                                         w_parent_it->second * l_mocap_datum.second});
      }
    }

    // Build a map of how close each world transform is to the next nearest world transform.
    std::unordered_map<HaptxName, float> proximity_m_from_name;
    for (std::unordered_map<TransformContext, Transform>::iterator a =
             w_mocap_frame.transforms.begin();
         a != w_mocap_frame.transforms.end(); a++) {
      float nearest_proximity_m = std::numeric_limits<float>::max();
      for (std::unordered_map<TransformContext, Transform>::iterator b =
               w_mocap_frame.transforms.begin();
           b != w_mocap_frame.transforms.end(); b++) {
        if (a == b) {
          continue;
        }

        float proximity_m = (a->second.getTranslation() - b->second.getTranslation()).length();
        if (proximity_m < nearest_proximity_m) {
          nearest_proximity_m = proximity_m;
        }
      }
      proximity_m_from_name[a->first.child] = nearest_proximity_m;
    }

    // Render haptics based on nearest world transforms.
    auto peripheral_from_slot = airpack->getAttachedPeripherals();
    if (!peripheral_from_slot) {
      printf("Airpack::getAttachedPeripherals() returned error code %d: %s.\n",
             static_cast<int>(peripheral_from_slot.error()),
             toString(peripheral_from_slot.error()).c_str());
      return false;
    }

    // One pneumatic frame per Airpack.
    PneumaticFrame pneumatic_frame;
    for (const auto& peripheral_it : *peripheral_from_slot) {
      if (peripheral_it.second == nullptr) {
        continue;
      }

      // One Haptic frame per Peripheral.
      HapticFrame haptic_frame;
      for (const auto& tactor : peripheral_it.second->tactors) {
        // The behavior when real hardware is detected.
        if (!peripheral_it.second->is_simulated) {
          // Scale the amplitude of the haptic response by proximity.
          const auto proximity_it = proximity_m_from_name.find(tactor.parent);
          if (proximity_it != proximity_m_from_name.end()) {
            float proximity_m = proximity_it->second;

            if (proximity_m < MAX_PROXIMITY_M) {
              const float proximity_scale_factor =
                  (MAX_PROXIMITY_M - proximity_m) / MAX_PROXIMITY_M;
              haptic_frame.tactor_heights_m.insert(
                  {tactor.id,
                   proximity_scale_factor * tactor.height_max_m * getWaveOutput(time_s)});
            }
          }
        } else {  // peripheral_it.second->is_simulated
          float tactor_height_m = 0.0f;
          std::shared_ptr<Glove> glove = std::dynamic_pointer_cast<Glove>(peripheral_it.second);
          if (auto trigger_value = glove ? OpenvrWrapper::getControllerAxisValue(
                                               glove->handedness, ControllerAxis::TRIGGER)
                                         : Unexpected{ReturnCode::UNKNOWN_ERROR}) {
            tactor_height_m = *trigger_value * tactor.height_max_m;
          } else {
            tactor_height_m = getWaveOutput(time_s) * tactor.height_max_m;
          }
          haptic_frame.tactor_heights_m.insert({tactor.id, tactor_height_m});
        }
      }

      if (!DirectPneumaticCalculator::addToPneumaticFrame(*peripheral_it.second, haptic_frame,
                                                          *airpack, &pneumatic_frame)) {
        printf("DirectPneumaticCalculator::addToPneumaticFrame() returned false.\n");
        return false;
      }
    }

    // Final rendering.
    if (auto ret = airpack->render(pneumatic_frame); !ret) {
      printf("Airpack::render() returned error code %d: %s.\n", static_cast<int>(ret.error()),
             toString(ret.error()).c_str());
      if (ret.error() == ReturnCode::LOST_CONNECTION) {
        return false;
      }
    }
  }

  return true;
}

// Executes when the application is run.
int main() {
  // Configure System Logger to print errors to stderr for easier debugging.
  Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());

  // Print messages to explain project functionality.
  printf("\n\nHAPTX C++ STARTER PROJECT\n");
  printf("=========================\n");
  printf("If you're using real hardware, try touching your fingers together.\n");
  printf("If you're using simulated hardware, grab a VR controller and pull the trigger.\n\n");
  std::flush(std::cout);

  // Get handles to connected hardware.
  HaptxSystem haptx_system;
  haptx_system.detectDevices(HaptxApi::PluginType::NATIVE_SDK);

  // Denotes whether this application is able to run properly and drive haptics.
  bool something_went_wrong = false;

  // Look for a Airpack.
  std::shared_ptr<Airpack> airpack =
      haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  if (airpack == nullptr) {
    printf("No valid Airpack found.\n\n");
    something_went_wrong = true;
  }

  for (const auto& mocap_system : haptx_system.getMocapSystems()) {
    if (mocap_system != nullptr) {
      if (auto ret = mocap_system->init(); !ret) {
        printf("MocapSystem::init() returned error code %d: %s.\n", static_cast<int>(ret.error()),
               toString(ret.error()).c_str());
        something_went_wrong = true;
      }
    }
  }
  // if (auto ret = OpenvrWrapper::init(OpenvrApplicationMode::SCENE); !ret) {
  //   printf("Openvr FAILURE \n ");
  //   printf("OpenvrWrapper::init() returned error code %d: %s.\n", static_cast<int>(ret.error()),
  //          toString(ret.error()).c_str());
  //   something_went_wrong = true;
  // }

  // Initialize variables for our main loop.
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point first_time_stamp = now;
  std::chrono::steady_clock::time_point last_time_stamp = now;
  std::chrono::duration<double> since_last_update{};
  std::chrono::duration<double> since_first_update{};

  while (!something_went_wrong && since_first_update.count() < PROGRAM_DURATION_S) {
    // Update all timers.
    now = std::chrono::steady_clock::now();
    since_last_update = now - last_time_stamp;
    since_first_update = now - first_time_stamp;

    // Don't act unless it's time to do so.
    if (since_last_update.count() < UPDATE_PERIOD_S) {
      continue;
    }
    last_time_stamp = now;

    // Run the logic that updates haptic set-points.
    something_went_wrong = !updateHaptics(airpack, static_cast<float>(since_first_update.count()));
  }

  OpenvrWrapper::shutdown();

  if (airpack) {
    airpack->shutdown();
  }

  printf("Finished!\n");
  std::flush(std::cout);
  waitForInput();
  return something_went_wrong ? -1 : 0;
}
