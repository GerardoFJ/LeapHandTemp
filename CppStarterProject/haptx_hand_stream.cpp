// haptx_hand_stream.cpp
//
// Streams the glove's finger joint angles over UDP, for teleoperating a robot
// hand (leap_teleop.py drives a LEAP Hand in MuJoCo and on hardware).
//
//   glove -> MocapSystem -> DefaultHandIk -> AnimFrame -> UDP -> leap_teleop.py
//
// MocapSystem::addToMocapFrame() gives fingertip poses relative to the MCP3
// tracking origin, and a hand IK solves them into an AnimFrame:
//   * AlternateHandIk (default), the "natural angle" solver, fills both
//     AnimFrame::angles (signed MCP/PIP/DIP hinge angles plus an adduction
//     angle, flexion negative) and AnimFrame::l_orientations.
//   * DefaultHandIk (--ik default), what tactile_demo --hand-view uses, only
//     fills l_orientations.
// Both sets are sent: leap_calibrate.py fits the fingers on the angles and the
// thumb on the joint orientations.
//
// With --feedback it also closes the loop the other way: leap_hand_node.py
// sends back how hard each LEAP finger is pushing (its motor current while it
// is held short of its target), and the glove's force-feedback brakes engage
// on those fingers, so your finger stops where the robot's finger met the
// object. The brakes are on/off (PassiveForceActuator), so a finger engages at
// --engage and only releases below --release, after at least --min-hold-ms.
// Without --feedback nothing here actuates the glove.
//
// One datagram per frame, plain text so it is easy to read with netcat:
//   H2,<hand L|R>,<seq>,<t_ms>,
//     then per finger (thumb, index, middle, ring, pinky):
//     <ang_ok>,<mcp>,<pip>,<dip>,<add>,<ori_ok>,<w,x,y,z joint1>,<joint2>,<joint3>
// ang_ok is 1 when AnimFrame::angles had the finger (radians); ori_ok is 1
// when all three joint orientations are there. Each orientation is relative to
// its parent in the skeleton (joint1: the palm), as AnimFrame documents.
//
// Usage: haptx_hand_stream [options]
//   --hand <h>        left | right                             (default left)
//   --ik <solver>     alternate | default                      (default alternate)
//   --no-legacy-blend AlternateHandIk: never blend in the legacy solver (fists)
//   --host <ip>       where leap_teleop.py runs                (default 127.0.0.1)
//   --port <n>        UDP port                                 (default 9871)
//   --rate <hz>       frames per second                        (default 90)
//   --quiet           no live angle table
//   --feedback        engage the finger brakes from leap_hand_node.py's effort
//   --feedback-port <n>  UDP port for that effort                (default 9873)
//   --engage <0-1>    effort that engages a finger's brake       (default 0.30)
//   --release <0-1>   effort below which it releases             (default 0.15)
//   --min-hold-ms <ms> shortest time a brake stays engaged       (default 120)
//   --no-ring-brake   leave the ring brake alone (by default it follows the
//                     pinky, which drives the LEAP's ring finger)
//   --list-brakes     print the glove's brakes and the fingers they resist, exit
//   --brake-test      engage each finger's brake in turn, exit
//   --help
//
// The IK uses the Dashboard's active user profile; without one it falls back to
// a default hand, and the angles will be off for yours.

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

using namespace HaptxApi;

namespace {

constexpr double PI = 3.14159265358979323846;

// Set from the SIGINT handler so a Ctrl-C still shuts the mocap system down.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

const char* const FINGER_NAMES[F_LAST] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};

struct Options {
  RelativeDirection hand = RD_LEFT;
  bool alternate_ik = true;
  bool legacy_blend = true;
  std::string host = "127.0.0.1";
  int port = 9871;
  double rate_hz = 90.0;
  bool quiet = false;
  bool feedback = false;
  int feedback_port = 9873;
  float engage = 0.30f;
  float release = 0.15f;
  int min_hold_ms = 120;
  int feedback_timeout_ms = 300;
  bool ring_brake = true;
  bool list_brakes = false;
  bool brake_test = false;
};

void printUsage() {
  printf(
      "Usage: haptx_hand_stream [options]\n"
      "  --hand <h>        left | right                             (default left)\n"
      "  --ik <solver>     alternate | default                      (default alternate)\n"
      "  --no-legacy-blend AlternateHandIk: never blend in the legacy solver (fists)\n"
      "  --host <ip>       where leap_teleop.py runs                (default 127.0.0.1)\n"
      "  --port <n>        UDP port                                 (default 9871)\n"
      "  --rate <hz>       frames per second                        (default 90)\n"
      "  --quiet           no live angle table\n"
      "  --feedback        engage the finger brakes from leap_hand_node.py's effort\n"
      "  --feedback-port <n>  UDP port for that effort                (default 9873)\n"
      "  --engage <0-1>    effort that engages a finger's brake       (default 0.30)\n"
      "  --release <0-1>   effort below which it releases             (default 0.15)\n"
      "  --min-hold-ms <ms> shortest time a brake stays engaged       (default 120)\n"
      "  --no-ring-brake   leave the ring brake alone (default: it follows the pinky)\n"
      "  --list-brakes     print the glove's brakes and the fingers they resist, exit\n"
      "  --brake-test      engage each finger's brake in turn, exit\n"
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

    if (strcmp(arg, "--hand") == 0 && has_value) {
      const std::string hand = toLower(argv[++i]);
      if (hand == "left") {
        options->hand = RD_LEFT;
      } else if (hand == "right") {
        options->hand = RD_RIGHT;
      } else {
        printf("Unrecognized --hand: %s (want left or right)\n", hand.c_str());
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
    } else if (strcmp(arg, "--feedback") == 0) {
      options->feedback = true;
    } else if (strcmp(arg, "--feedback-port") == 0 && has_value) {
      options->feedback_port = atoi(argv[++i]);
    } else if (strcmp(arg, "--engage") == 0 && has_value) {
      options->engage = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--release") == 0 && has_value) {
      options->release = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--min-hold-ms") == 0 && has_value) {
      options->min_hold_ms = atoi(argv[++i]);
    } else if (strcmp(arg, "--no-ring-brake") == 0) {
      options->ring_brake = false;
    } else if (strcmp(arg, "--list-brakes") == 0) {
      options->list_brakes = true;
    } else if (strcmp(arg, "--brake-test") == 0) {
      options->brake_test = true;
    } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      printUsage();
      return false;
    } else {
      printf("Unrecognized argument: %s\n\n", arg);
      printUsage();
      return false;
    }
  }

  if (options->port <= 0 || options->port > 65535) {
    printf("--port must be 1-65535.\n");
    return false;
  }
  if (options->rate_hz <= 0.0 || options->rate_hz > 500.0) {
    printf("--rate must be between 0 and 500.\n");
    return false;
  }
  if (options->feedback_port <= 0 || options->feedback_port > 65535) {
    printf("--feedback-port must be 1-65535.\n");
    return false;
  }
  if (!(options->release >= 0.0f && options->release < options->engage &&
        options->engage <= 1.0f)) {
    printf("Need 0 <= --release < --engage <= 1.\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// UDP output
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

  void send(const std::string& message) {
    sendto(socket_, message.data(), static_cast<int>(message.size()), 0,
           reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
  }

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  sockaddr_in destination_ = {};
};

// Receives leap_hand_node.py's effort, one datagram per reading:
//   LFB,<hand L|R>,<seq>,<thumb>,<index>,<middle>,<ring>
// 0..1 per LEAP finger: how hard it pushes against something.
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

  // Drains the queue and keeps the newest reading for `hand_letter`. Returns
  // true if one arrived since the last call.
  bool poll(char hand_letter, std::array<float, 4>* effort) {
    bool got = false;
    for (;;) {
      const int n = recv(socket_, buffer_, sizeof(buffer_) - 1, 0);
      if (n <= 0) {
        break;  // WSAEWOULDBLOCK: nothing more queued.
      }
      buffer_[n] = '\0';
      char hand = 0;
      unsigned long seq = 0;
      std::array<float, 4> e = {};
      if (sscanf(buffer_, "LFB,%c,%lu,%f,%f,%f,%f", &hand, &seq, &e[0], &e[1], &e[2], &e[3]) != 6) {
        continue;
      }
      if (hand != hand_letter) {
        wrong_hand++;
        continue;
      }
      for (float& v : e) {
        v = std::isfinite(v) ? std::max(0.0f, std::min(1.0f, v)) : 0.0f;
      }
      *effort = e;
      got = true;
    }
    return got;
  }

  int wrong_hand = 0;

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  char buffer_[512];
};

// ---------------------------------------------------------------------------
// Force-feedback brakes
// ---------------------------------------------------------------------------

// One of the glove's ForceActuators and the fingers it resists, read from its
// restrictions (the body parts it holds back).
struct Brake {
  int id = 0;
  std::array<bool, F_LAST> fingers = {};
};

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

void listBrakes(const Peripheral& peripheral) {
  printf("\nForce-feedback brakes on this glove:\n");
  if (peripheral.force_actuators.empty()) {
    printf("  none\n");
    return;
  }
  printf("  id  enabled  port  valve  engage [kPa]  resists\n");
  for (const ForceActuator& actuator : peripheral.force_actuators) {
    std::string parts;
    for (const auto& restriction : actuator.getRestrictions()) {
      parts += (parts.empty() ? "" : ", ") + getName(restriction.first).getText();
    }
    printf("  %2d  %-7s  %4d  %5d  %12.1f  %s\n", actuator.getId(),
           actuator.isEnabled() ? "yes" : "no", actuator.getPort(), actuator.getValveId(),
           actuator.getActuationPressurePa() / 1000.0f, parts.empty() ? "-" : parts.c_str());
  }
}

// Renders one frame: every brake ENGAGED or DISENGAGED, and every tactor at
// its minimum height, so the frame fully describes the hardware state (as in
// fsr_fingertip.cpp -- don't run both at once, they would fight over it).
bool renderBrakes(Airpack& airpack, const Peripheral& peripheral, const std::vector<Brake>& brakes,
                  const std::array<bool, F_LAST>& engaged) {
  HapticFrame haptic_frame;
  for (const Tactor& tactor : peripheral.tactors) {
    haptic_frame.tactor_heights_m[tactor.getId()] = tactor.getMinHeightM();
  }
  for (const ForceActuator& actuator : peripheral.force_actuators) {
    haptic_frame.force_actuator_states[actuator.getId()] = PassiveForceActuator::State::DISENGAGED;
  }
  for (const Brake& brake : brakes) {
    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      if (brake.fingers[finger] && engaged[finger]) {
        haptic_frame.force_actuator_states[brake.id] = PassiveForceActuator::State::ENGAGED;
      }
    }
  }
  PneumaticFrame pneumatic_frame;
  if (!DirectPneumaticCalculator::addToPneumaticFrame(peripheral, haptic_frame, airpack,
                                                      &pneumatic_frame)) {
    printf("\nDirectPneumaticCalculator::addToPneumaticFrame() failed.\n");
    return false;
  }
  if (auto ret = airpack.render(pneumatic_frame); !ret) {
    printf("\nAirpack::render() returned %s.\n", toString(ret.error()).c_str());
    return false;
  }
  return true;
}

// Engages each finger's brake in turn so the mapping can be felt.
void brakeTest(Airpack& airpack, const Peripheral& peripheral, const std::vector<Brake>& brakes) {
  using Clock = std::chrono::steady_clock;
  printf("\nBrake test: each finger's brake engages for 2 s. Open your hand, then try to\n"
         "close the named finger while it is on -- it should stop.\n\n");
  const std::array<bool, F_LAST> none = {};
  for (unsigned int finger = 0; finger < F_LAST && !g_interrupted; finger++) {
    bool has_brake = false;
    for (const Brake& brake : brakes) {
      has_brake = has_brake || brake.fingers[finger];
    }
    if (!has_brake) {
      printf("  %-7s no brake\n", FINGER_NAMES[finger]);
      continue;
    }
    std::array<bool, F_LAST> engaged = {};
    engaged[finger] = true;
    printf("  %-7s ON ... ", FINGER_NAMES[finger]);
    std::flush(std::cout);
    const Clock::time_point end = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < end && !g_interrupted) {
      renderBrakes(airpack, peripheral, brakes, engaged);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    renderBrakes(airpack, peripheral, brakes, none);
    printf("off\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  renderBrakes(airpack, peripheral, brakes, none);
}

// ---------------------------------------------------------------------------
// Angles
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

}  // namespace

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
