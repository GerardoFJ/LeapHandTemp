// ik_stream_feedback.cpp
//
// ik_stream with the loop closed: streams one or both gloves' finger joint
// angles to the LEAP teleop over UDP, and renders what the LEAP Hands feel back
// on the gloves -- fingertip tactors in proportion to the force, and the brakes
// once a finger presses hard.
//
//   gloves -> MocapSystem -> hand IK -> H2,L / H2,R  --UDP 9871-->  leap_teleop_tactile.py
//   gloves <- tactors + brakes <- LFT,L / LFT,R      <--UDP 9873--  (force from the LEAP currents)
//
// Glove frames, one datagram per hand and frame (the same as ik_stream):
//   H2,<hand L|R>,<seq>,<t_ms>,
//     then per finger (thumb, index, middle, ring, pinky):
//     <ang_ok>,<mcp>,<pip>,<dip>,<add>,<ori_ok>,<w,x,y,z joint1>,<joint2>,<joint3>
// Feedback, from leap_teleop_tactile.py:
//   LFT,<hand L|R>,<seq>,<thumb>,<index>,<middle>,<ring>     fingertip force [N] per LEAP finger
// Your pinky drives the LEAP's ring finger, so that force goes to the pinky, and
// to the ring as well (it closes with the pinky) unless --no-ring.
//
// Per glove finger, from its force F:
//   tactors  the fingertip (*_DISTAL) tactors rise from their minimum height at
//            --touch to --tactor-scale x their maximum at --full, linearly
//   brake    engages at --engage, releases below --release (after --min-hold-ms)
// No feedback for --feedback-timeout-ms (the LEAP script stopped or crashed):
// every tactor deflates and every brake releases, so no finger stays locked.
//
// Usage: ik_stream_feedback [options]
//   --hands <h>       left | right | both                      (default both)
//   --ik <solver>     alternate | default                      (default alternate)
//   --no-legacy-blend AlternateHandIk: never blend in the legacy solver (fists)
//   --host <ip>       where leap_teleop_tactile.py runs        (default 127.0.0.1)
//   --port <n>        UDP port for the glove frames            (default 9871)
//   --rate <hz>       frames per second                        (default 90)
//   --quiet           no live table
//   --no-feedback     only stream, actuate nothing (like ik_stream)
//   --feedback-port <n>  UDP port for the forces               (default 9873)
//   --feedback-timeout-ms <ms>  release everything after this silence (default 300)
//   --no-tactors      brakes only
//   --no-brakes       tactors only
//   --touch <N>       force where the fingertip tactors start  (default 0.3)
//   --full <N>        force that inflates them fully           (default 2.5)
//   --tactor-scale <0-1>  height at --full, fraction of max    (default 1.0)
//   --engage <N>      force that engages a finger's brake      (default 1.5)
//   --release <N>     force below which it releases            (default 0.8)
//   --min-hold-ms <ms> shortest time a brake stays engaged     (default 120)
//   --no-ring         the glove's ring finger gets no feedback
//   --list            print each glove's fingertip tactors and brakes, exit
//   --help
//
// The IK uses the Dashboard's active user profile; without one it falls back to
// a default hand, and the angles will be off for yours. Stop it with Ctrl-C in
// an attached terminal so it releases the glove on the way out.

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
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace HaptxApi;

namespace {

using Clock = std::chrono::steady_clock;
constexpr double PI = 3.14159265358979323846;
constexpr int LEAP_FINGERS = 4;  // thumb, index, middle, ring

// Set from the SIGINT handler so a Ctrl-C still releases the glove.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

const char* const FINGER_NAMES[F_LAST] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};

// The SDK's shutdown (MocapSystem::shutdown(), Airpack::shutdown()) never returns
// under Wine (it waits on a USB pipe), so the process ends here instead, as
// ik_stream does. Whatever the glove must not keep doing is released before.
[[noreturn]] void exitNow(int code) {
  fflush(stdout);
  TerminateProcess(GetCurrentProcess(), static_cast<UINT>(code));
  std::_Exit(code);
}

struct Options {
  bool left = true;
  bool right = true;
  bool alternate_ik = true;
  bool legacy_blend = true;
  std::string host = "127.0.0.1";
  int port = 9871;
  double rate_hz = 90.0;
  bool quiet = false;
  bool feedback = true;
  int feedback_port = 9873;
  int feedback_timeout_ms = 300;
  bool tactors = true;
  bool brakes = true;
  float touch_n = 0.3f;
  float full_n = 2.5f;
  float tactor_scale = 1.0f;
  float engage_n = 1.5f;
  float release_n = 0.8f;
  int min_hold_ms = 120;
  bool ring = true;
  bool list = false;
};

void printUsage() {
  printf(
      "Usage: ik_stream_feedback [options]\n"
      "  --hands <h>       left | right | both                      (default both)\n"
      "  --ik <solver>     alternate | default                      (default alternate)\n"
      "  --no-legacy-blend AlternateHandIk: never blend in the legacy solver (fists)\n"
      "  --host <ip>       where leap_teleop_tactile.py runs        (default 127.0.0.1)\n"
      "  --port <n>        UDP port for the glove frames            (default 9871)\n"
      "  --rate <hz>       frames per second                        (default 90)\n"
      "  --quiet           no live table\n"
      "  --no-feedback     only stream, actuate nothing (like ik_stream)\n"
      "  --feedback-port <n>  UDP port for the forces               (default 9873)\n"
      "  --feedback-timeout-ms <ms>  release everything after this silence (default 300)\n"
      "  --no-tactors      brakes only\n"
      "  --no-brakes       tactors only\n"
      "  --touch <N>       force where the fingertip tactors start  (default 0.3)\n"
      "  --full <N>        force that inflates them fully           (default 2.5)\n"
      "  --tactor-scale <0-1>  height at --full, fraction of max    (default 1.0)\n"
      "  --engage <N>      force that engages a finger's brake      (default 1.5)\n"
      "  --release <N>     force below which it releases            (default 0.8)\n"
      "  --min-hold-ms <ms> shortest time a brake stays engaged     (default 120)\n"
      "  --no-ring         the glove's ring finger gets no feedback\n"
      "  --list            print each glove's fingertip tactors and brakes, exit\n"
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

    if (strcmp(arg, "--hands") == 0 && has_value) {
      const std::string hands = toLower(argv[++i]);
      if (hands == "left" || hands == "right" || hands == "both") {
        options->left = hands != "right";
        options->right = hands != "left";
      } else {
        printf("Unrecognized --hands: %s (want left, right or both)\n", hands.c_str());
        return false;
      }
    } else if (strcmp(arg, "--ik") == 0 && has_value) {
      const std::string ik = toLower(argv[++i]);
      if (ik == "alternate") {
        options->alternate_ik = true;
      } else if (ik == "default") {
        options->alternate_ik = false;
      } else {
        printf("Unrecognized --ik: %s (want alternate or default)\n", ik.c_str());
        return false;
      }
    } else if (strcmp(arg, "--no-legacy-blend") == 0) {
      options->legacy_blend = false;
    } else if (strcmp(arg, "--host") == 0 && has_value) {
      options->host = argv[++i];
    } else if (strcmp(arg, "--port") == 0 && has_value) {
      options->port = atoi(argv[++i]);
    } else if (strcmp(arg, "--rate") == 0 && has_value) {
      options->rate_hz = atof(argv[++i]);
    } else if (strcmp(arg, "--quiet") == 0) {
      options->quiet = true;
    } else if (strcmp(arg, "--no-feedback") == 0) {
      options->feedback = false;
    } else if (strcmp(arg, "--feedback-port") == 0 && has_value) {
      options->feedback_port = atoi(argv[++i]);
    } else if (strcmp(arg, "--feedback-timeout-ms") == 0 && has_value) {
      options->feedback_timeout_ms = atoi(argv[++i]);
    } else if (strcmp(arg, "--no-tactors") == 0) {
      options->tactors = false;
    } else if (strcmp(arg, "--no-brakes") == 0) {
      options->brakes = false;
    } else if (strcmp(arg, "--touch") == 0 && has_value) {
      options->touch_n = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--full") == 0 && has_value) {
      options->full_n = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--tactor-scale") == 0 && has_value) {
      options->tactor_scale = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--engage") == 0 && has_value) {
      options->engage_n = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--release") == 0 && has_value) {
      options->release_n = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--min-hold-ms") == 0 && has_value) {
      options->min_hold_ms = atoi(argv[++i]);
    } else if (strcmp(arg, "--no-ring") == 0) {
      options->ring = false;
    } else if (strcmp(arg, "--list") == 0) {
      options->list = true;
    } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      printUsage();
      return false;
    } else {
      printf("Unrecognized argument: %s\n\n", arg);
      printUsage();
      return false;
    }
  }

  if (options->port <= 0 || options->port > 65535 || options->feedback_port <= 0 ||
      options->feedback_port > 65535) {
    printf("--port and --feedback-port must be 1-65535.\n");
    return false;
  }
  if (options->rate_hz <= 0.0 || options->rate_hz > 500.0) {
    printf("--rate must be between 0 and 500.\n");
    return false;
  }
  if (options->touch_n < 0.0f || options->full_n <= options->touch_n) {
    printf("Want 0 <= --touch < --full.\n");
    return false;
  }
  if (options->tactor_scale <= 0.0f || options->tactor_scale > 1.0f) {
    printf("--tactor-scale must be in (0, 1].\n");
    return false;
  }
  if (options->release_n < 0.0f || options->engage_n <= options->release_n) {
    printf("Want 0 <= --release < --engage.\n");
    return false;
  }
  if (options->feedback_timeout_ms <= 0 || options->min_hold_ms < 0) {
    printf("--feedback-timeout-ms must be positive and --min-hold-ms not negative.\n");
    return false;
  }
  if (!options->tactors && !options->brakes) {
    options->feedback = false;  // nothing left to actuate
  }
  return true;
}

// ---------------------------------------------------------------------------
// Glove actuators
// ---------------------------------------------------------------------------

// One fingertip tactor and its height range.
struct TipTactor {
  int id = 0;
  float min_m = 0.0f;
  float max_m = 0.0f;
};

// One of the glove's ForceActuators and the fingers it resists, read from its
// restrictions (the body parts it holds back).
struct Brake {
  int id = 0;
  std::array<bool, F_LAST> fingers = {};
};

CoverageRegion fingertipRegion(RelativeDirection side, unsigned int finger) {
  static const CoverageRegion kRight[F_LAST] = {
      CoverageRegion::RIGHT_THUMB_DISTAL, CoverageRegion::RIGHT_INDEX_DISTAL,
      CoverageRegion::RIGHT_MIDDLE_DISTAL, CoverageRegion::RIGHT_RING_DISTAL,
      CoverageRegion::RIGHT_PINKY_DISTAL};
  static const CoverageRegion kLeft[F_LAST] = {
      CoverageRegion::LEFT_THUMB_DISTAL, CoverageRegion::LEFT_INDEX_DISTAL,
      CoverageRegion::LEFT_MIDDLE_DISTAL, CoverageRegion::LEFT_RING_DISTAL,
      CoverageRegion::LEFT_PINKY_DISTAL};
  return side == RD_RIGHT ? kRight[finger] : kLeft[finger];
}

std::array<std::vector<TipTactor>, F_LAST> mapFingertips(const Peripheral& peripheral,
                                                         RelativeDirection side) {
  std::array<std::vector<TipTactor>, F_LAST> tips;
  for (unsigned int finger = 0; finger < F_LAST; finger++) {
    const HaptxName region = getName(fingertipRegion(side, finger));
    for (const Tactor& tactor : peripheral.tactors) {
      if (tactor.isEnabled() && tactor.coverage_region == region) {
        tips[finger].push_back({tactor.getId(), tactor.getMinHeightM(), tactor.getMaxHeightM()});
      }
    }
  }
  return tips;
}

std::vector<Brake> mapBrakes(const Peripheral& peripheral) {
  std::vector<Brake> brakes;
  for (const ForceActuator& actuator : peripheral.force_actuators) {
    if (!actuator.isEnabled()) {
      continue;
    }
    Brake brake;
    brake.id = actuator.getId();
    for (const auto& restriction : actuator.getRestrictions()) {
      const Finger finger = getFinger(restriction.first);
      if (finger < F_LAST) {
        brake.fingers[finger] = true;
      }
    }
    brakes.push_back(brake);
  }
  return brakes;
}

void listActuators(const Peripheral& peripheral, RelativeDirection side) {
  const auto tips = mapFingertips(peripheral, side);
  printf("  fingertip tactors:");
  for (unsigned int finger = 0; finger < F_LAST; finger++) {
    printf("  %s %d", FINGER_NAMES[finger], static_cast<int>(tips[finger].size()));
  }
  for (const auto& tip : tips) {
    if (!tip.empty()) {
      printf("   (height %.1f-%.1f mm)", tip.front().min_m * 1000.0f, tip.front().max_m * 1000.0f);
      break;
    }
  }
  printf("\n  brakes:");
  const std::vector<Brake> brakes = mapBrakes(peripheral);
  if (brakes.empty()) {
    printf(" none enabled");
  }
  for (const Brake& brake : brakes) {
    printf("  [id %d:", brake.id);
    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      if (brake.fingers[finger]) {
        printf(" %s", FINGER_NAMES[finger]);
      }
    }
    printf("]");
  }
  printf("\n");
}

// ---------------------------------------------------------------------------
// One glove
// ---------------------------------------------------------------------------

// One finger, as sent.
struct FingerData {
  bool has_angles = false;
  float joint_rad[FJ_LAST] = {0.0f, 0.0f, 0.0f};
  float adduction_rad = 0.0f;
  bool has_orientations = false;
  Quaternion orientations[FJ_LAST] = {Quaternion::identity(), Quaternion::identity(),
                                      Quaternion::identity()};
};

struct Hand {
  RelativeDirection side = RD_LEFT;
  char letter = 'L';
  std::shared_ptr<MocapSystem> mocap;
  std::shared_ptr<Glove> glove;

  // Stream.
  std::unordered_map<Finger, FingerAngleHint> hints;  // the natural solver's last angles
  unsigned long seq = 0;
  int ik_failures = 0;
  int frame_failures = 0;
  FingerData fingers[F_LAST];

  // Feedback.
  std::array<std::vector<TipTactor>, F_LAST> tips;
  std::vector<Brake> brakes;
  std::array<float, LEAP_FINGERS> leap_force = {};  // N, as received
  Clock::time_point last_feedback = {};
  unsigned long feedback_count = 0;
  unsigned long feedback_count_then = 0;
  float feedback_hz = 0.0f;
  bool fresh = false;
  std::array<float, F_LAST> force = {};    // N per glove finger
  std::array<float, F_LAST> level = {};    // tactor height 0..1 (scale included)
  std::array<bool, F_LAST> engaged = {};
  std::array<Clock::time_point, F_LAST> engaged_since = {};
};

const char* sideName(RelativeDirection side) { return side == RD_LEFT ? "left" : "right"; }

// Pulls one finger out of an AnimFrame.
FingerData fingerFrom(const AnimFrame& anim_frame, RelativeDirection handedness, Finger finger) {
  FingerData out;

  const auto angle_it = anim_frame.angles.find(finger);
  if (angle_it != anim_frame.angles.end() && angle_it->second.flexion.isValid()) {
    const FingerAngleFrame& frame = angle_it->second;
    out.has_angles = true;
    out.joint_rad[FJ_JOINT1] = frame.flexion.theta_mcp_rad;
    out.joint_rad[FJ_JOINT2] = frame.flexion.theta_pip_rad;
    out.joint_rad[FJ_JOINT3] = frame.flexion.theta_dip_rad;
    out.adduction_rad = frame.adduction.isValid() ? frame.adduction.angle_rad : 0.0f;
  }

  out.has_orientations = true;
  for (unsigned int joint = 0; joint < FJ_LAST; joint++) {
    const BodyPartJoint key =
        getBodyPartJoint(handedness, finger, static_cast<FingerJoint>(joint));
    const auto it = anim_frame.l_orientations.find(key);
    if (it == anim_frame.l_orientations.end()) {
      out.has_orientations = false;
      break;
    }
    out.orientations[joint] = it->second;
  }
  return out;
}

// Renders one frame for every glove: each tactor at its height (the fingertips
// from the force, the rest at their minimum) and each brake ENGAGED or
// DISENGAGED, so the frame fully describes the hardware state and nothing stays
// inflated or locked by omission.
bool renderGloves(Airpack& airpack, const std::vector<Hand>& hands) {
  PneumaticFrame pneumatic_frame;
  for (const Hand& hand : hands) {
    HapticFrame haptic_frame;
    for (const Tactor& tactor : hand.glove->tactors) {
      haptic_frame.tactor_heights_m[tactor.getId()] = tactor.getMinHeightM();
    }
    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      for (const TipTactor& tip : hand.tips[finger]) {
        haptic_frame.tactor_heights_m[tip.id] =
            tip.min_m + hand.level[finger] * (tip.max_m - tip.min_m);
      }
    }
    for (const ForceActuator& actuator : hand.glove->force_actuators) {
      haptic_frame.force_actuator_states[actuator.getId()] = PassiveForceActuator::State::DISENGAGED;
    }
    for (const Brake& brake : hand.brakes) {
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        if (brake.fingers[finger] && hand.engaged[finger]) {
          haptic_frame.force_actuator_states[brake.id] = PassiveForceActuator::State::ENGAGED;
        }
      }
    }
    if (!DirectPneumaticCalculator::addToPneumaticFrame(*hand.glove, haptic_frame, airpack,
                                                        &pneumatic_frame)) {
      return false;
    }
  }
  if (auto ret = airpack.render(pneumatic_frame); !ret) {
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// UDP
// ---------------------------------------------------------------------------

class UdpSender {
 public:
  ~UdpSender() {
    if (socket_ != INVALID_SOCKET) {
      closesocket(socket_);
    }
    if (started_) {
      WSACleanup();
    }
  }

  bool open(const std::string& host, int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      printf("WSAStartup failed.\n");
      return false;
    }
    started_ = true;
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
      printf("socket() failed: %d\n", WSAGetLastError());
      return false;
    }
    destination_ = {};
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(static_cast<u_short>(port));
    if (inet_pton(AF_INET, host.c_str(), &destination_.sin_addr) != 1) {
      printf("--host must be an IPv4 address, got '%s'.\n", host.c_str());
      return false;
    }
    return true;
  }

  void send(const char* data, int length) {
    sendto(socket_, data, length, 0, reinterpret_cast<const sockaddr*>(&destination_),
           sizeof(destination_));
  }

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  sockaddr_in destination_ = {};
};

// Receives leap_teleop_tactile.py's forces, one datagram per reading and hand:
//   LFT,<hand L|R>,<seq>,<thumb>,<index>,<middle>,<ring>     [N]
class FeedbackReceiver {
 public:
  ~FeedbackReceiver() {
    if (socket_ != INVALID_SOCKET) {
      closesocket(socket_);
    }
    if (started_) {
      WSACleanup();
    }
  }

  bool open(int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      printf("WSAStartup failed.\n");
      return false;
    }
    started_ = true;
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
      printf("socket() failed: %d\n", WSAGetLastError());
      return false;
    }
    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<u_short>(port));
    if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      printf("bind() to UDP port %d failed: %d. Is another copy running?\n", port,
             WSAGetLastError());
      return false;
    }
    u_long non_blocking = 1;
    ioctlsocket(socket_, FIONBIO, &non_blocking);
    return true;
  }

  // Drains the queue; each reading goes to the hand with its letter.
  void poll(std::vector<Hand>* hands, Clock::time_point now) {
    for (;;) {
      const int n = recv(socket_, buffer_, sizeof(buffer_) - 1, 0);
      if (n <= 0) {
        break;  // WSAEWOULDBLOCK: nothing more queued.
      }
      buffer_[n] = '\0';
      char letter = 0;
      unsigned long seq = 0;
      std::array<float, LEAP_FINGERS> f = {};
      if (sscanf(buffer_, "LFT,%c,%lu,%f,%f,%f,%f", &letter, &seq, &f[0], &f[1], &f[2], &f[3]) !=
          6) {
        bad++;
        continue;
      }
      Hand* hand = nullptr;
      for (Hand& h : *hands) {
        if (h.letter == letter) {
          hand = &h;
        }
      }
      if (hand == nullptr) {
        other_hand++;
        continue;
      }
      for (float& v : f) {
        v = std::isfinite(v) ? std::max(0.0f, std::min(50.0f, v)) : 0.0f;
      }
      hand->leap_force = f;
      hand->last_feedback = now;
      hand->feedback_count++;
    }
  }

  int bad = 0;
  int other_hand = 0;

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  char buffer_[512];
};

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }

  std::signal(SIGINT, onSigint);

  printf("\nHAPTX IK STREAM + FEEDBACK\n");
  printf("==========================\n");
  fflush(stdout);

  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  const std::shared_ptr<Airpack> airpack =
      haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  if (airpack == nullptr) {
    printf("No Airpack found. Is it powered, connected over USB, and are the USB\n");
    printf("devices bound to WinUSB/USBCCGP in this Wine prefix?\n");
    exitNow(1);
  }

  auto attached_peripherals = airpack->getAttachedPeripherals();
  auto mocap_systems = airpack->getMocapSystems();
  if (!attached_peripherals || !mocap_systems) {
    printf("Airpack did not report its peripherals or mocap systems.\n");
    exitNow(1);
  }

  // Pick the gloves by handedness, not by slot, so each follows its glove.
  std::vector<Hand> hands;
  for (const RelativeDirection side : {RD_LEFT, RD_RIGHT}) {
    if ((side == RD_LEFT && !options.left) || (side == RD_RIGHT && !options.right)) {
      continue;
    }
    bool found = false;
    for (const auto& peripheral_it : *attached_peripherals) {
      const auto* glove = dynamic_cast<const Glove*>(peripheral_it.second.get());
      if (glove == nullptr || glove->handedness != side) {
        continue;
      }
      const auto it = mocap_systems->find(peripheral_it.first);
      if (it == mocap_systems->end() || it->second == nullptr) {
        continue;
      }
      printf("%s glove: slot %d, %s\n", side == RD_LEFT ? "Left" : "Right", peripheral_it.first,
             peripheral_it.second->casual_name.c_str());
      if (options.list) {
        listActuators(*peripheral_it.second, side);
      }
      Hand hand;
      hand.side = side;
      hand.letter = side == RD_LEFT ? 'L' : 'R';
      hand.mocap = it->second;
      hands.push_back(std::move(hand));
      found = true;
      break;
    }
    if (!found) {
      printf("No %s glove with a mocap system.%s\n", sideName(side),
             options.left && options.right ? " Use --hands left or --hands right for one." : "");
      exitNow(1);
    }
  }
  if (options.list) {
    exitNow(0);
  }

  for (Hand& hand : hands) {
    if (auto ret = hand.mocap->init(); !ret) {
      printf("MocapSystem::init() returned %s for the %s glove.\n", toString(ret.error()).c_str(),
             sideName(hand.side));
      exitNow(1);
    }
    hand.glove = hand.mocap->getGlove();
    if (hand.glove == nullptr) {
      printf("MocapSystem::getGlove() returned nullptr for the %s glove.\n", sideName(hand.side));
      exitNow(1);
    }
    hand.tips = mapFingertips(*hand.glove, hand.side);
    hand.brakes = mapBrakes(*hand.glove);
    int tips = 0;
    for (const auto& tip : hand.tips) {
      tips += static_cast<int>(tip.size());
    }
    if (options.feedback && options.tactors && tips == 0) {
      printf("The %s glove reports no fingertip tactors.\n", sideName(hand.side));
    }
    if (options.feedback && options.brakes && hand.brakes.empty()) {
      printf("The %s glove reports no enabled brakes.\n", sideName(hand.side));
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
    exitNow(1);
  }
  FeedbackReceiver receiver;
  if (options.feedback && !receiver.open(options.feedback_port)) {
    exitNow(1);
  }
  printf("Streaming %s to %s:%d at %.0f Hz.\n",
         hands.size() == 2 ? "both gloves" : (hands[0].side == RD_LEFT ? "the left glove"
                                                                       : "the right glove"),
         options.host.c_str(), options.port, options.rate_hz);
  if (options.feedback) {
    printf("Forces from leap_teleop_tactile.py on UDP %d.\n", options.feedback_port);
    if (options.tactors) {
      printf("  tactors: %.2f N -> nothing, %.2f N -> %.0f%% of full height\n", options.touch_n,
             options.full_n, options.tactor_scale * 100.0f);
    }
    if (options.brakes) {
      printf("  brakes:  engage at %.2f N, release below %.2f N, hold at least %d ms\n",
             options.engage_n, options.release_n, options.min_hold_ms);
    }
  } else {
    printf("No feedback: nothing on the gloves is actuated.\n");
  }
  printf("Ctrl-C to stop.\n\n");
  fflush(stdout);

  // SDK log lines would shred the in-place table, so only register the log
  // writer when the table is off.
  if (options.quiet) {
    Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());
  }

  const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
  const auto feedback_timeout = std::chrono::milliseconds(options.feedback_timeout_ms);
  const auto min_hold = std::chrono::milliseconds(options.min_hold_ms);
  const Clock::time_point start = Clock::now();
  Clock::time_point next = start;
  Clock::time_point last_table = start;
  Clock::time_point last_rate = start;
  int lines_drawn = 0;
  for (Hand& hand : hands) {
    hand.last_feedback = start - std::chrono::hours(1);
  }

  AlternateHandIk::Config ik_config;
  ik_config.legacy_blend_enabled = options.legacy_blend;
  const char* ik_name = !options.alternate_ik ? "DefaultHandIk"
                        : options.legacy_blend ? "AlternateHandIk"
                                               : "AlternateHandIk, no legacy blend";
  printf("IK: %s\n\n", ik_name);

  bool render_ok = !options.feedback || renderGloves(*airpack, hands);

  while (!g_interrupted) {
    // ----- stream: one H2 frame per glove
    for (Hand& hand : hands) {
      // MOCAP_SOLVER_ERROR, MOCAP_BAD_SENSOR_DATA and NOT_READY are intermittent;
      // the frame below simply carries whatever the system still has.
      hand.mocap->update();
      MocapFrame mocap_frame;
      AnimFrame anim_frame;
      anim_frame.hints = hand.hints;
      if (hand.mocap->addToMocapFrame(&mocap_frame)) {
        const bool solved =
            options.alternate_ik
                ? AlternateHandIk::addToAnimFrame(hand.glove, ik_config, mocap_frame, profile,
                                                  &anim_frame)
                : DefaultHandIk::addToAnimFrame(hand.glove, mocap_frame, profile, &anim_frame);
        if (solved) {
          hand.hints = anim_frame.hints;
        } else {
          hand.ik_failures++;
        }
      } else {
        hand.frame_failures++;
      }

      const long long t_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
      char buffer[2048];
      int length =
          snprintf(buffer, sizeof(buffer), "H2,%c,%lu,%lld", hand.letter, hand.seq++, t_ms);
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        hand.fingers[finger] = fingerFrom(anim_frame, hand.side, static_cast<Finger>(finger));
        const FingerData& f = hand.fingers[finger];
        length += snprintf(buffer + length, sizeof(buffer) - length, ",%d,%.4f,%.4f,%.4f,%.4f,%d",
                           f.has_angles ? 1 : 0, f.joint_rad[FJ_JOINT1], f.joint_rad[FJ_JOINT2],
                           f.joint_rad[FJ_JOINT3], f.adduction_rad, f.has_orientations ? 1 : 0);
        for (const Quaternion& q : f.orientations) {
          length += snprintf(buffer + length, sizeof(buffer) - length, ",%.5f,%.5f,%.5f,%.5f",
                             q.w_, q.x_, q.y_, q.z_);
        }
      }
      sender.send(buffer, length);
    }

    // ----- feedback: newest forces -> tactor heights and brake states
    if (options.feedback) {
      const Clock::time_point t = Clock::now();
      receiver.poll(&hands, t);
      for (Hand& hand : hands) {
        hand.fresh = t - hand.last_feedback <= feedback_timeout;
        hand.force = {};
        if (hand.fresh) {
          hand.force[F_THUMB] = hand.leap_force[0];
          hand.force[F_INDEX] = hand.leap_force[1];
          hand.force[F_MIDDLE] = hand.leap_force[2];
          hand.force[F_PINKY] = hand.leap_force[3];  // the pinky drives the LEAP's ring
          hand.force[F_RING] = options.ring ? hand.leap_force[3] : 0.0f;
        }
        for (unsigned int finger = 0; finger < F_LAST; finger++) {
          const float f = hand.force[finger];
          hand.level[finger] =
              options.tactors ? options.tactor_scale *
                                    std::max(0.0f, std::min(1.0f, (f - options.touch_n) /
                                                                      (options.full_n - options.touch_n)))
                              : 0.0f;
          if (!options.brakes || !hand.fresh) {
            hand.engaged[finger] = false;  // no word from the LEAP: never leave a finger locked
          } else if (!hand.engaged[finger] && f >= options.engage_n) {
            hand.engaged[finger] = true;
            hand.engaged_since[finger] = t;
          } else if (hand.engaged[finger] && f < options.release_n &&
                     t - hand.engaged_since[finger] >= min_hold) {
            hand.engaged[finger] = false;
          }
        }
      }
      render_ok = renderGloves(*airpack, hands);
    }

    // ----- live table
    const Clock::time_point now = Clock::now();
    if (std::chrono::duration<double>(now - last_rate).count() >= 1.0) {
      const double seconds = std::chrono::duration<double>(now - last_rate).count();
      for (Hand& hand : hands) {
        hand.feedback_hz =
            static_cast<float>((hand.feedback_count - hand.feedback_count_then) / seconds);
        hand.feedback_count_then = hand.feedback_count;
      }
      last_rate = now;
    }
    if (!options.quiet && std::chrono::duration<double>(now - last_table).count() >= 0.1) {
      if (lines_drawn > 0) {
        printf("\033[%dA", lines_drawn);  // redraw in place
      }
      constexpr double kDeg = 180.0 / PI;
      lines_drawn = 0;
      printf("  hand finger    MCP   PIP   DIP [deg]%s\n",
             options.feedback ? "   force   tactor  brake" : "");
      lines_drawn++;
      for (const Hand& hand : hands) {
        for (unsigned int finger = 0; finger < F_LAST; finger++) {
          const FingerData& f = hand.fingers[finger];
          if (f.has_angles) {
            printf("  %c    %-7s %5.0f %5.0f %5.0f      ", hand.letter, FINGER_NAMES[finger],
                   f.joint_rad[FJ_JOINT1] * kDeg, f.joint_rad[FJ_JOINT2] * kDeg,
                   f.joint_rad[FJ_JOINT3] * kDeg);
          } else {
            printf("  %c    %-7s    --    --    --      ", hand.letter, FINGER_NAMES[finger]);
          }
          if (options.feedback) {
            printf("%5.2f N   %3.0f %%   %s", hand.force[finger], hand.level[finger] * 100.0f,
                   hand.engaged[finger] ? "ON " : " - ");
          }
          printf("   \n");
          lines_drawn++;
        }
        printf("  %c    sent %-8lu mocap fail %-5d ik fail %-5d", hand.letter, hand.seq,
               hand.frame_failures, hand.ik_failures);
        if (options.feedback) {
          if (hand.fresh) {
            printf(" feedback %3.0f/s                    ", hand.feedback_hz);
          } else {
            printf(" NO FEEDBACK -- released             ");
          }
        }
        printf("\n");
        lines_drawn++;
      }
      if (options.feedback) {
        printf("  render %s   bad packets %d   other hand %d          \n",
               render_ok ? "ok" : "FAILED", receiver.bad, receiver.other_hand);
        lines_drawn++;
      }
      fflush(stdout);
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
    for (Hand& hand : hands) {
      hand.level = {};
      hand.engaged = {};
    }
    // A few release frames over ~0.3 s, so the Airpack surely has one before
    // the process ends.
    for (int i = 0; i < 15; i++) {
      renderGloves(*airpack, hands);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printf("Tactors deflated, brakes released.\n");
  }
  exitNow(0);
}
