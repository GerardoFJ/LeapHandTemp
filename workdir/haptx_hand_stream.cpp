// haptx_hand_stream.cpp
//
// Streams the glove's finger joint angles over UDP, for teleoperating a robot
// hand (leap_teleop.py drives a LEAP Hand in MuJoCo and on hardware).
//
//   glove -> MocapSystem -> DefaultHandIk -> AnimFrame -> UDP -> leap_teleop.py
//
// This is the --hand-view pipeline of tactile_demo.cpp without the drawing:
// MocapSystem::addToMocapFrame() gives fingertip poses relative to the MCP3
// tracking origin, DefaultHandIk::addToAnimFrame() solves them into per-finger
// angles, and AnimFrame::angles carries, per finger, MCP/PIP/DIP flexion and an
// adduction angle, all in radians. Nothing here actuates the glove.
//
// One datagram per frame, plain text so it is easy to read with netcat:
//   H,<hand L|R>,<seq>,<t_ms>,
//     <valid>,<mcp>,<pip>,<dip>,<add>,   x5 fingers: thumb, index, middle, ring, pinky
// valid is 0 when the IK produced nothing for that finger, 1 when the angles
// came from AnimFrame::angles, and 2 when they were recovered from the joint
// orientations (flexion only, adduction then reads 0).
//
// Usage: haptx_hand_stream [options]
//   --hand <h>        left | right                             (default left)
//   --host <ip>       where leap_teleop.py runs                (default 127.0.0.1)
//   --port <n>        UDP port                                 (default 9871)
//   --rate <hz>       frames per second                        (default 90)
//   --quiet           no live angle table
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
#include <HaptxApi/anim_frame.h>
#include <HaptxApi/default_hand_ik.h>
#include <HaptxApi/enum.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/haptx_system.h>
#include <HaptxApi/logging.h>
#include <HaptxApi/mocap_frame.h>
#include <HaptxApi/mocap_system.h>
#include <HaptxApi/names.h>
#include <HaptxApi/peripheral.h>
#include <HaptxApi/user_profile.h>
#include <HaptxApi/user_profile_database.h>

#include <algorithm>
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

using namespace HaptxApi;

namespace {

constexpr double PI = 3.14159265358979323846;

// Set from the SIGINT handler so a Ctrl-C still shuts the mocap system down.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

const char* const FINGER_NAMES[F_LAST] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};

struct Options {
  RelativeDirection hand = RD_LEFT;
  std::string host = "127.0.0.1";
  int port = 9871;
  double rate_hz = 90.0;
  bool quiet = false;
};

void printUsage() {
  printf(
      "Usage: haptx_hand_stream [options]\n"
      "  --hand <h>        left | right                             (default left)\n"
      "  --host <ip>       where leap_teleop.py runs                (default 127.0.0.1)\n"
      "  --port <n>        UDP port                                 (default 9871)\n"
      "  --rate <hz>       frames per second                        (default 90)\n"
      "  --quiet           no live angle table\n"
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
    } else if (strcmp(arg, "--host") == 0 && has_value) {
      options->host = argv[++i];
    } else if (strcmp(arg, "--port") == 0 && has_value) {
      options->port = atoi(argv[++i]);
    } else if (strcmp(arg, "--rate") == 0 && has_value) {
      options->rate_hz = atof(argv[++i]);
    } else if (strcmp(arg, "--quiet") == 0) {
      options->quiet = true;
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

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------

// One finger's angles, as sent.
struct FingerAngles {
  int valid = 0;  // 0 none, 1 from AnimFrame::angles, 2 recovered from orientations
  float joint_rad[FJ_LAST] = {0.0f, 0.0f, 0.0f};
  float adduction_rad = 0.0f;
};

// Pulls one finger out of an AnimFrame.
//
// DefaultHandIk::addToAnimFrame() documents itself as filling the joint
// orientations, AnimFrame::l_orientations. AnimFrame::angles is a separate
// field the solver does not always populate, so prefer it when it is there --
// it is the only one carrying adduction -- and fall back to the orientations,
// whose rotation magnitude about the joint axis is the bend.
FingerAngles anglesFor(const AnimFrame& anim_frame, RelativeDirection handedness, Finger finger) {
  FingerAngles out;

  const auto angle_it = anim_frame.angles.find(finger);
  if (angle_it != anim_frame.angles.end() && angle_it->second.flexion.isValid()) {
    const FingerAngleFrame& frame = angle_it->second;
    out.valid = 1;
    out.joint_rad[FJ_JOINT1] = frame.flexion.theta_mcp_rad;
    out.joint_rad[FJ_JOINT2] = frame.flexion.theta_pip_rad;
    out.joint_rad[FJ_JOINT3] = frame.flexion.theta_dip_rad;
    out.adduction_rad = frame.adduction.isValid() ? frame.adduction.angle_rad : 0.0f;
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
    out.valid = 2;
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

  while (!g_interrupted) {
    // MOCAP_SOLVER_ERROR, MOCAP_BAD_SENSOR_DATA and NOT_READY are intermittent;
    // the frame below simply carries whatever the system still has.
    mocap_system->update();

    MocapFrame mocap_frame;
    AnimFrame anim_frame;
    if (mocap_system->addToMocapFrame(&mocap_frame)) {
      if (!DefaultHandIk::addToAnimFrame(glove, mocap_frame, profile, &anim_frame)) {
        ik_failures++;
      }
    } else {
      frame_failures++;
    }

    FingerAngles fingers[F_LAST];
    const long long t_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
    char buffer[512];
    int length = snprintf(buffer, sizeof(buffer), "H,%c,%lu,%lld", hand_letter, seq++, t_ms);
    for (unsigned int finger = 0; finger < F_LAST; finger++) {
      fingers[finger] = anglesFor(anim_frame, options.hand, static_cast<Finger>(finger));
      const FingerAngles& f = fingers[finger];
      length += snprintf(buffer + length, sizeof(buffer) - length, ",%d,%.4f,%.4f,%.4f,%.4f",
                         f.valid, f.joint_rad[FJ_JOINT1], f.joint_rad[FJ_JOINT2],
                         f.joint_rad[FJ_JOINT3], f.adduction_rad);
    }
    sender.send(std::string(buffer, static_cast<size_t>(length)));

    const Clock::time_point now = Clock::now();
    if (!options.quiet && std::chrono::duration<double>(now - last_table).count() >= 0.1) {
      if (lines_drawn > 0) {
        printf("\033[%dA", lines_drawn);  // redraw in place
      }
      constexpr double kDeg = 180.0 / PI;
      printf("  finger    MCP    PIP    DIP    ADD   src\n");
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        const FingerAngles& f = fingers[finger];
        if (f.valid == 0) {
          printf("  %-7s    --     --     --     --   none \n", FINGER_NAMES[finger]);
        } else {
          printf("  %-7s %5.0f  %5.0f  %5.0f  %5.0f   %s\n", FINGER_NAMES[finger],
                 f.joint_rad[FJ_JOINT1] * kDeg, f.joint_rad[FJ_JOINT2] * kDeg,
                 f.joint_rad[FJ_JOINT3] * kDeg, f.adduction_rad * kDeg,
                 f.valid == 1 ? "ik   " : "quat ");
        }
      }
      printf("  sent %-8lu  mocap frame fail %-5d  ik fail %-5d\n", seq, frame_failures,
             ik_failures);
      lines_drawn = F_LAST + 2;
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
  mocap_system->shutdown();
  airpack->shutdown();
  return 0;
}
