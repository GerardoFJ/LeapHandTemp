#ifdef _WIN32
// Ahead of everything: windows.h defines min/max macros that fight <algorithm>,
// and winsock2.h must come before windows.h.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <HaptxApi/airpack.h>
#include <HaptxApi/alternate_hand_ik.h>
#include <HaptxApi/anim_frame.h>
#include <HaptxApi/default_hand_ik.h>
#include <HaptxApi/direct_pneumatic_calculator.h>
#include <HaptxApi/enum.h>
#include <HaptxApi/force_actuator.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/haptic_frame.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/mocap_frame.h>
#include <HaptxApi/mocap_system.h>
#include <HaptxApi/names.h>
#include <HaptxApi/peripheral.h>
#include <HaptxApi/pneumatic_frame.h>
#include <HaptxApi/tactor.h>
#include <HaptxApi/user_profile.h>
#include <HaptxApi/user_profile_database.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>





int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }

  std::signal(SIGINT, onSigint);

  printf("\nHAPTX HAND STREAM\n");
  printf("=================\n");
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
  auto mocap_systems = airpack->getMocapSystems();
  if (!attached_peripherals || !mocap_systems) {
    printf("Airpack did not report its peripherals or mocap systems.\n");
    airpack->shutdown();
    return 1;
  }

  // Pick the glove by handedness, not by slot, so it follows the glove.
  std::shared_ptr<MocapSystem> mocap_system;
  for (const auto& peripheral_it : *attached_peripherals) {
    const auto* glove = dynamic_cast<const Glove*>(peripheral_it.second.get());
    if (glove == nullptr || glove->handedness != options.hand) {
      continue;
    }
    const auto it = mocap_systems->find(peripheral_it.first);
    if (it != mocap_systems->end() && it->second != nullptr) {
      mocap_system = it->second;
      printf("Glove: slot %d, %s\n", peripheral_it.first,
             peripheral_it.second->casual_name.c_str());
      break;
    }
  }
  if (mocap_system == nullptr) {
    printf("No %s glove with a mocap system. Try --hand %s.\n",
           options.hand == RD_LEFT ? "left" : "right", options.hand == RD_LEFT ? "right" : "left");
    airpack->shutdown();
    return 1;
  }

  if (auto ret = mocap_system->init(); !ret) {
    printf("MocapSystem::init() returned %s.\n", toString(ret.error()).c_str());
    airpack->shutdown();
    return 1;
  }
  const std::shared_ptr<Glove> glove = mocap_system->getGlove();
  if (glove == nullptr) {
    printf("MocapSystem::getGlove() returned nullptr.\n");
    mocap_system->shutdown();
    airpack->shutdown();
    return 1;
  }

  const std::vector<Brake> brakes = mapBrakes(*glove);
  if (options.list_brakes || options.brake_test) {
    listBrakes(*glove);
    if (options.brake_test) {
      brakeTest(*airpack, *glove, brakes);
    }
    mocap_system->shutdown();
    airpack->shutdown();
    return 0;
  }

  FeedbackReceiver feedback;
  if (options.feedback) {
    if (brakes.empty()) {
      printf("This glove reports no enabled force-feedback brakes -- --feedback is off.\n");
      options.feedback = false;
    } else if (!feedback.open(options.feedback_port)) {
      mocap_system->shutdown();
      airpack->shutdown();
      return 1;
    } else {
      printf("Brakes from leap_hand_node.py on UDP %d: engage at %.2f, release below %.2f.\n",
             options.feedback_port, options.engage, options.release);
    }
  }

  // The IK needs hand geometry; a default profile works but is less accurate.
  UserProfile profile;
  if (auto username = UserProfileDatabase::getActiveUsername(); username) {
    if (auto stored = UserProfileDatabase::getUserProfile(*username); stored) {
      profile = *stored;
      printf("Using the active user profile.\n");
    } else {
      printf("No stored profile for the active user -- using a default hand.\n");
    }
  } else {
    printf("No active user profile -- using a default hand. Create one in the Dashboard.\n");
  }

  UdpSender sender;
  if (!sender.open(options.host, options.port)) {
    mocap_system->shutdown();
    airpack->shutdown();
    return 1;
  }
  printf("Streaming to %s:%d at %.0f Hz. Ctrl-C to stop.\n\n", options.host.c_str(), options.port,
         options.rate_hz);
  std::flush(std::cout);

  // SDK log lines would shred the in-place table, so only register the log
  // writer when the table is off.
  if (options.quiet) {
    Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());
  }

  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
  const Clock::time_point start = Clock::now();
  Clock::time_point next = start;
  Clock::time_point last_table = start;
  unsigned long seq = 0;
  int ik_failures = 0;
  int frame_failures = 0;
  int lines_drawn = 0;
  const char hand_letter = options.hand == RD_LEFT ? 'L' : 'R';

  // The natural solver can use the previous frame's angles as hints.
  AlternateHandIk::Config ik_config;
  ik_config.legacy_blend_enabled = options.legacy_blend;
  std::unordered_map<Finger, FingerAngleHint> hints;
  const char* ik_name = !options.alternate_ik ? "DefaultHandIk"
                        : options.legacy_blend ? "AlternateHandIk"
                                               : "AlternateHandIk, no legacy blend";
  printf("IK: %s\n\n", ik_name);

  // Brake state per human finger. The LEAP's ring finger is driven by the
  // pinky, so its effort lands on the pinky (and the ring, which closes with it).
  std::array<float, 4> leap_effort = {};
  std::array<float, F_LAST> effort = {};
  std::array<bool, F_LAST> engaged = {};
  std::array<Clock::time_point, F_LAST> engaged_since = {};
  Clock::time_point last_feedback = start - std::chrono::hours(1);
  const auto feedback_timeout = std::chrono::milliseconds(options.feedback_timeout_ms);
  const auto min_hold = std::chrono::milliseconds(options.min_hold_ms);
  bool brakes_ok = !options.feedback || renderBrakes(*airpack, *glove, brakes, engaged);

  while (!g_interrupted) {
    // MOCAP_SOLVER_ERROR, MOCAP_BAD_SENSOR_DATA and NOT_READY are intermittent;
    // the frame below simply carries whatever the system still has.
    mocap_system->update();

    MocapFrame mocap_frame;
    AnimFrame anim_frame;
    anim_frame.hints = hints;
    if (mocap_system->addToMocapFrame(&mocap_frame)) {
      const bool solved =
          options.alternate_ik
              ? AlternateHandIk::addToAnimFrame(glove, ik_config, mocap_frame, profile, &anim_frame)
              : DefaultHandIk::addToAnimFrame(glove, mocap_frame, profile, &anim_frame);
      if (solved) {
        hints = anim_frame.hints;
      } else {
        ik_failures++;
      }
    } else {
      frame_failures++;
    }

    FingerData fingers[F_LAST];
    const long long t_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
    char buffer[2048];
    int length = snprintf(buffer, sizeof(buffer), "H2,%c,%lu,%lld", hand_letter, seq++, t_ms);
    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      fingers[finger] = fingerFrom(anim_frame, options.hand, static_cast<Finger>(finger));
      const FingerData& f = fingers[finger];
      length += snprintf(buffer + length, sizeof(buffer) - length, ",%d,%.4f,%.4f,%.4f,%.4f,%d",
                         f.has_angles ? 1 : 0, f.joint_rad[FJ_JOINT1], f.joint_rad[FJ_JOINT2],
                         f.joint_rad[FJ_JOINT3], f.adduction_rad, f.has_orientations ? 1 : 0);
      for (const Quaternion& q : f.orientations) {
        length += snprintf(buffer + length, sizeof(buffer) - length, ",%.5f,%.5f,%.5f,%.5f",
                           q.w_, q.x_, q.y_, q.z_);
      }
    }
    sender.send(std::string(buffer, static_cast<size_t>(length)));

    if (options.feedback) {
      const Clock::time_point t = Clock::now();
      if (feedback.poll(hand_letter, &leap_effort)) {
        last_feedback = t;
      }
      const bool fresh = t - last_feedback <= feedback_timeout;
      effort = {};
      if (fresh) {
        effort[F_THUMB] = leap_effort[0];
        effort[F_INDEX] = leap_effort[1];
        effort[F_MIDDLE] = leap_effort[2];
        effort[F_PINKY] = leap_effort[3];
        effort[F_RING] = options.ring_brake ? leap_effort[3] : 0.0f;
      }
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        if (!fresh) {
          engaged[finger] = false;  // no word from the hand: never leave a finger locked
        } else if (!engaged[finger] && effort[finger] >= options.engage) {
          engaged[finger] = true;
          engaged_since[finger] = t;
        } else if (engaged[finger] && effort[finger] < options.release &&
                   t - engaged_since[finger] >= min_hold) {
          engaged[finger] = false;
        }
      }
      brakes_ok = renderBrakes(*airpack, *glove, brakes, engaged);
    }

    const Clock::time_point now = Clock::now();
    if (!options.quiet && std::chrono::duration<double>(now - last_table).count() >= 0.1) {
      if (lines_drawn > 0) {
        printf("\033[%dA", lines_drawn);  // redraw in place
      }
      constexpr double kDeg = 180.0 / PI;
      printf("  finger    MCP    PIP    DIP    ADD   joint rotation [deg]\n");
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        const FingerData& f = fingers[finger];
        if (f.has_angles) {
          printf("  %-7s %5.0f  %5.0f  %5.0f  %5.0f ", FINGER_NAMES[finger],
                 f.joint_rad[FJ_JOINT1] * kDeg, f.joint_rad[FJ_JOINT2] * kDeg,
                 f.joint_rad[FJ_JOINT3] * kDeg, f.adduction_rad * kDeg);
        } else {
          printf("  %-7s    --     --     --     -- ", FINGER_NAMES[finger]);
        }
        if (f.has_orientations) {
          // Total rotation per joint, rest pose included: only a liveness cue.
          for (const Quaternion& q : f.orientations) {
            printf("  %4.0f", 2.0 * std::acos(std::min(1.0f, std::fabs(q.w_))) * kDeg);
          }
          printf("   \n");
        } else {
          printf("    --    --    --   \n");
        }
      }
      printf("  sent %-8lu  mocap frame fail %-5d  ik fail %-5d\n", seq, frame_failures,
             ik_failures);
      lines_drawn = F_LAST + 2;
      if (options.feedback) {
        const double age_ms =
            std::chrono::duration<double, std::milli>(now - last_feedback).count();
        printf("  brakes ");
        for (unsigned int finger = 0; finger < F_LAST; finger++) {
          printf(" %c %.2f%s", FINGER_NAMES[finger][0], effort[finger], engaged[finger] ? "#" : " ");
        }
        if (age_ms > options.feedback_timeout_ms) {
          printf("   no effort from leap_hand_node.py -- released%s     \n",
                 brakes_ok ? "" : ", RENDER FAILED");
        } else {
          printf("   (# = engaged)%s                \n", brakes_ok ? "" : "  RENDER FAILED");
        }
        lines_drawn++;
      }
      std::flush(std::cout);
      last_table = now;
    }

    next += std::chrono::duration_cast<Clock::duration>(period);
    if (next < now) {
      next = now;  // fell behind: don't try to catch up in a burst
    }
    std::this_thread::sleep_until(next);
  }

  printf("\nStopping.\n");
  if (options.feedback) {
    renderBrakes(*airpack, *glove, brakes, {});  // release every brake
  }
  mocap_system->shutdown();
  airpack->shutdown();
  return 0;
}

