// fsr_fingertip.cpp
//
// Feel the FSR matrix on a fingertip. The Teensy scans the sensor matrix
// (FSR_Development/Teensy_Project), fsr_bridge.py forwards each frame over UDP,
// and this program spreads the pressure image over the tactors of one G1
// fingertip, so a press on one side of the sensor is felt on that side of the
// finger.
//
//   Teensy --USB serial--> fsr_bridge.py (Linux) --UDP--> fsr_fingertip.exe (Wine)
//
// Like fingertip_test.cpp it drives the tactors straight from the hardware
// definition: no motion capture, no tracker, no OpenVR, no ContactInterpreter.
//
// Signal path, per cell:
//   raw ADC LSB -> minus a no-touch baseline taken at startup -> inverted (the
//   Teensy's ADC code goes negative under force) -> deadband -> divided by
//   --full-scale -> clamped to [0, 1].
//
// Spatial mapping: every tactor of the chosen fingertip region has a position.
// The pad's two widest axes become the matrix's row and column axes, both
// normalized to [0, 1], and each tactor takes the bilinearly interpolated
// pressure at its spot. If the felt pattern is mirrored or rotated, fix it with
// --transpose / --flip-rows / --flip-cols; --list prints the mapping.
//
// Usage: fsr_fingertip [options]
//   --list               print the tactor -> matrix mapping and exit
//   --hand <h>           left | right                              (default left)
//   --finger <f>         thumb | index | middle | ring | pinky     (default index)
//   --port <n>           UDP port to listen on                     (default 9870)
//   --full-scale <lsb>   pressure change that means full inflation (default 12000)
//   --deadband <lsb>     change ignored as noise                   (default 300)
//   --scale <0..1>       fraction of full inflation height         (default 1.0)
//   --baseline <s>       no-touch baseline averaging time          (default 1.0)
//   --timeout <ms>       deflate if no frame arrives for this long (default 250)
//   --no-invert          pressure raises the raw value instead of lowering it
//   --transpose          swap which pad axis the rows and columns run along
//   --flip-rows          mirror the row axis
//   --flip-cols          mirror the column axis
//   --help
//
// ~12000 LSB is roughly 10 N on the calibrated r1c3 cell (V = a*F^n); raise
// --full-scale for a firmer press to reach full height. Heights are fractions of
// each tactor's own getMaxHeightM(), and DirectPneumaticCalculator clamps on top
// of that, so --scale 1.0 is full rated inflation, not an overpressure.

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
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace HaptxApi;

namespace {

// How often set points are pushed to the Airpack [Hz]. The Teensy sends 50
// frames/s; rendering faster keeps latency down and the deflate-on-timeout prompt.
constexpr double RENDER_RATE_HZ = 90.0;

// The Teensy marks a cell it failed to convert with INT16_MIN.
constexpr int INVALID_CELL = -32768;

// Set from the SIGINT handler so a Ctrl-C still deflates the tactors.
volatile std::sig_atomic_t g_interrupted = 0;

void onSigint(int /*signal*/) { g_interrupted = 1; }

struct Options {
  bool list_only = false;
  RelativeDirection hand = RD_LEFT;
  int finger = 1;  // 0 thumb .. 4 pinky
  int port = 9870;
  float full_scale_lsb = 12000.0f;
  float deadband_lsb = 300.0f;
  float scale = 1.0f;
  double baseline_s = 1.0;
  double timeout_ms = 250.0;
  bool invert = true;
  bool transpose = false;
  bool flip_rows = false;
  bool flip_cols = false;
};

const char* const FINGER_NAMES[5] = {"thumb", "index", "middle", "ring", "pinky"};

void printUsage() {
  printf(
      "Usage: fsr_fingertip [options]\n"
      "  --list               print the tactor -> matrix mapping and exit\n"
      "  --hand <h>           left | right                              (default left)\n"
      "  --finger <f>         thumb | index | middle | ring | pinky     (default index)\n"
      "  --port <n>           UDP port to listen on                     (default 9870)\n"
      "  --full-scale <lsb>   pressure change that means full inflation (default 12000)\n"
      "  --deadband <lsb>     change ignored as noise                   (default 300)\n"
      "  --scale <0..1>       fraction of full inflation height         (default 1.0)\n"
      "  --baseline <s>       no-touch baseline averaging time          (default 1.0)\n"
      "  --timeout <ms>       deflate if no frame arrives for this long (default 250)\n"
      "  --no-invert          pressure raises the raw value instead of lowering it\n"
      "  --transpose          swap which pad axis the rows and columns run along\n"
      "  --flip-rows          mirror the row axis\n"
      "  --flip-cols          mirror the column axis\n"
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
    } else if (strcmp(arg, "--hand") == 0 && has_value) {
      const std::string hand = toLower(argv[++i]);
      if (hand == "left") {
        options->hand = RD_LEFT;
      } else if (hand == "right") {
        options->hand = RD_RIGHT;
      } else {
        printf("Unrecognized --hand: %s (want left or right)\n", hand.c_str());
        return false;
      }
    } else if (strcmp(arg, "--finger") == 0 && has_value) {
      const std::string finger = toLower(argv[++i]);
      const auto* end = FINGER_NAMES + 5;
      const auto* it = std::find(FINGER_NAMES, end, finger);
      if (it == end) {
        printf("Unrecognized --finger: %s (want thumb, index, middle, ring or pinky)\n",
               finger.c_str());
        return false;
      }
      options->finger = static_cast<int>(it - FINGER_NAMES);
    } else if (strcmp(arg, "--port") == 0 && has_value) {
      options->port = atoi(argv[++i]);
    } else if (strcmp(arg, "--full-scale") == 0 && has_value) {
      options->full_scale_lsb = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--deadband") == 0 && has_value) {
      options->deadband_lsb = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--scale") == 0 && has_value) {
      options->scale = static_cast<float>(atof(argv[++i]));
    } else if (strcmp(arg, "--baseline") == 0 && has_value) {
      options->baseline_s = atof(argv[++i]);
    } else if (strcmp(arg, "--timeout") == 0 && has_value) {
      options->timeout_ms = atof(argv[++i]);
    } else if (strcmp(arg, "--no-invert") == 0) {
      options->invert = false;
    } else if (strcmp(arg, "--transpose") == 0) {
      options->transpose = true;
    } else if (strcmp(arg, "--flip-rows") == 0) {
      options->flip_rows = true;
    } else if (strcmp(arg, "--flip-cols") == 0) {
      options->flip_cols = true;
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
  if (options->full_scale_lsb <= options->deadband_lsb || options->deadband_lsb < 0.0f) {
    printf("--full-scale must be larger than --deadband, and --deadband not negative.\n");
    return false;
  }
  if (options->port <= 0 || options->port > 65535) {
    printf("--port must be 1-65535.\n");
    return false;
  }
  if (options->baseline_s <= 0.0 || options->timeout_ms <= 0.0) {
    printf("--baseline and --timeout must be positive.\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Sensor input
// ---------------------------------------------------------------------------

// One matrix frame, row-major, raw ADC LSB.
struct SensorFrame {
  int rows = 0;
  int cols = 0;
  std::vector<int> cells;
};

// Parses "M,<rows>,<cols>,<micros>,<c0>,<c1>,..." from fsr_bridge.py.
bool parseFrame(const char* text, SensorFrame* frame) {
  if (text[0] != 'M' || text[1] != ',') {
    return false;
  }
  char* cursor = const_cast<char*>(text + 2);
  const long rows = strtol(cursor, &cursor, 10);
  if (*cursor != ',') return false;
  const long cols = strtol(cursor + 1, &cursor, 10);
  if (*cursor != ',') return false;
  strtoul(cursor + 1, &cursor, 10);  // Teensy micros, unused here.
  if (rows <= 0 || cols <= 0 || rows > 64 || cols > 64) {
    return false;
  }

  frame->rows = static_cast<int>(rows);
  frame->cols = static_cast<int>(cols);
  frame->cells.assign(static_cast<size_t>(rows * cols), 0);
  for (long i = 0; i < rows * cols; i++) {
    if (*cursor != ',') {
      return false;
    }
    frame->cells[static_cast<size_t>(i)] = static_cast<int>(strtol(cursor + 1, &cursor, 10));
  }
  return true;
}

// Non-blocking UDP socket that keeps only the newest frame.
class UdpReceiver {
 public:
  ~UdpReceiver() {
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

  // Drains everything queued and keeps the newest valid frame. Returns true if
  // at least one arrived since the last call.
  bool poll(SensorFrame* latest, int* frames_received) {
    bool got = false;
    for (;;) {
      const int n = recv(socket_, buffer_, sizeof(buffer_) - 1, 0);
      if (n <= 0) {
        break;  // WSAEWOULDBLOCK: nothing more queued.
      }
      buffer_[n] = '\0';
      if (parseFrame(buffer_, &scratch_)) {
        std::swap(*latest, scratch_);
        got = true;
        (*frames_received)++;
      }
    }
    return got;
  }

 private:
  SOCKET socket_ = INVALID_SOCKET;
  bool started_ = false;
  char buffer_[16384];
  SensorFrame scratch_;  // parse target, so a bad datagram never clobbers the last good frame
};

// Per-cell no-touch reference, averaged over the first --baseline seconds.
class Baseline {
 public:
  void reset(int rows, int cols) {
    rows_ = rows;
    cols_ = cols;
    sum_.assign(static_cast<size_t>(rows * cols), 0.0);
    count_.assign(static_cast<size_t>(rows * cols), 0);
    frames_ = 0;
  }

  bool matches(const SensorFrame& frame) const {
    return frame.rows == rows_ && frame.cols == cols_;
  }

  void add(const SensorFrame& frame) {
    for (size_t i = 0; i < frame.cells.size(); i++) {
      if (frame.cells[i] != INVALID_CELL) {
        sum_[i] += frame.cells[i];
        count_[i]++;
      }
    }
    frames_++;
  }

  int frames() const { return frames_; }

  float at(size_t i) const {
    return count_[i] > 0 ? static_cast<float>(sum_[i] / count_[i]) : 0.0f;
  }

 private:
  int rows_ = 0;
  int cols_ = 0;
  std::vector<double> sum_;
  std::vector<int> count_;
  int frames_ = 0;
};

// Raw frame -> pressure in [0, 1] per cell.
std::vector<float> toPressure(const SensorFrame& frame, const Baseline& baseline,
                              const Options& options) {
  std::vector<float> pressure(frame.cells.size(), 0.0f);
  for (size_t i = 0; i < frame.cells.size(); i++) {
    if (frame.cells[i] == INVALID_CELL) {
      continue;
    }
    float delta = static_cast<float>(frame.cells[i]) - baseline.at(i);
    if (options.invert) {
      delta = -delta;
    }
    const float level =
        (delta - options.deadband_lsb) / (options.full_scale_lsb - options.deadband_lsb);
    pressure[i] = std::max(0.0f, std::min(1.0f, level));
  }
  return pressure;
}

// Bilinear sample of the pressure image at (u, v) in [0, 1]^2, where u runs
// along the rows and v along the columns. Cell centres sit at (i + 0.5) / n.
float samplePressure(const std::vector<float>& pressure, int rows, int cols, float u, float v) {
  const float r = std::max(0.0f, std::min(static_cast<float>(rows - 1), u * rows - 0.5f));
  const float c = std::max(0.0f, std::min(static_cast<float>(cols - 1), v * cols - 0.5f));
  const int r0 = static_cast<int>(r);
  const int c0 = static_cast<int>(c);
  const int r1 = std::min(r0 + 1, rows - 1);
  const int c1 = std::min(c0 + 1, cols - 1);
  const float fr = r - r0;
  const float fc = c - c0;
  auto at = [&](int row, int col) { return pressure[static_cast<size_t>(row * cols + col)]; };
  const float top = at(r0, c0) * (1.0f - fc) + at(r0, c1) * fc;
  const float bottom = at(r1, c0) * (1.0f - fc) + at(r1, c1) * fc;
  return top * (1.0f - fr) + bottom * fr;
}

// ---------------------------------------------------------------------------
// Tactor mapping
// ---------------------------------------------------------------------------

// Where one tactor sits on the matrix, both coordinates in [0, 1].
struct TactorSpot {
  int id = 0;
  float u = 0.5f;  // along the matrix rows
  float v = 0.5f;  // along the matrix columns
};

CoverageRegion fingertipRegion(RelativeDirection hand, int finger) {
  static const CoverageRegion kRight[5] = {
      CoverageRegion::RIGHT_THUMB_DISTAL, CoverageRegion::RIGHT_INDEX_DISTAL,
      CoverageRegion::RIGHT_MIDDLE_DISTAL, CoverageRegion::RIGHT_RING_DISTAL,
      CoverageRegion::RIGHT_PINKY_DISTAL};
  static const CoverageRegion kLeft[5] = {
      CoverageRegion::LEFT_THUMB_DISTAL, CoverageRegion::LEFT_INDEX_DISTAL,
      CoverageRegion::LEFT_MIDDLE_DISTAL, CoverageRegion::LEFT_RING_DISTAL,
      CoverageRegion::LEFT_PINKY_DISTAL};
  return hand == RD_RIGHT ? kRight[finger] : kLeft[finger];
}

float axisOf(const Vector3D& p, int axis) { return axis == 0 ? p.x_ : (axis == 1 ? p.y_ : p.z_); }

// Lays the matrix over the pad: the widest axis carries the rows, the next
// widest the columns. Returns false only if the region has no tactors; a pad
// whose tactors share no common frame falls back to every tactor at the centre,
// which still renders the press, just without the sense of where.
bool mapTactors(const Peripheral& peripheral, CoverageRegion region, const Options& options,
                std::vector<TactorSpot>* spots) {
  const HaptxName region_name = getName(region);
  std::vector<const Tactor*> tactors;
  for (const Tactor& tactor : peripheral.tactors) {
    if (tactor.coverage_region == region_name && tactor.isEnabled()) {
      tactors.push_back(&tactor);
    }
  }
  if (tactors.empty()) {
    return false;
  }

  // Positions are only comparable when they share a parent frame.
  bool common_frame = true;
  const HaptxName parent = tactors.front()->getParent();
  for (const Tactor* tactor : tactors) {
    common_frame = common_frame && tactor->getParent() == parent;
  }

  float low[3] = {0, 0, 0};
  float span[3] = {0, 0, 0};
  int axis_u = 0;
  int axis_v = 1;
  if (common_frame) {
    float high[3];
    const Vector3D first = tactors.front()->getTransform().getTranslation();
    for (int a = 0; a < 3; a++) {
      low[a] = high[a] = axisOf(first, a);
    }
    for (const Tactor* tactor : tactors) {
      const Vector3D p = tactor->getTransform().getTranslation();
      for (int a = 0; a < 3; a++) {
        low[a] = std::min(low[a], axisOf(p, a));
        high[a] = std::max(high[a], axisOf(p, a));
      }
    }
    int order[3] = {0, 1, 2};
    for (int a = 0; a < 3; a++) {
      span[a] = high[a] - low[a];
    }
    std::sort(order, order + 3, [&span](int a, int b) { return span[a] > span[b]; });
    axis_u = order[0];
    axis_v = order[1];
  } else {
    printf("  %s spans more than one reference frame -- every tactor gets the\n"
           "  matrix centre, so you feel how hard but not where.\n",
           region_name.getText().c_str());
  }
  if (options.transpose) {
    std::swap(axis_u, axis_v);
  }

  spots->clear();
  for (const Tactor* tactor : tactors) {
    TactorSpot spot;
    spot.id = tactor->getId();
    if (common_frame) {
      const Vector3D p = tactor->getTransform().getTranslation();
      spot.u = span[axis_u] > 0.0f ? (axisOf(p, axis_u) - low[axis_u]) / span[axis_u] : 0.5f;
      spot.v = span[axis_v] > 0.0f ? (axisOf(p, axis_v) - low[axis_v]) / span[axis_v] : 0.5f;
    }
    if (options.flip_rows) spot.u = 1.0f - spot.u;
    if (options.flip_cols) spot.v = 1.0f - spot.v;
    spots->push_back(spot);
  }

  if (common_frame) {
    printf("  rows along the %c axis (%.1f mm), columns along the %c axis (%.1f mm)\n",
           "xyz"[axis_u], span[axis_u] * 1000.0f, "xyz"[axis_v], span[axis_v] * 1000.0f);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

// Builds one frame for the glove and renders it. Every tactor is named -- the
// ones that should be flat are commanded to their minimum height rather than
// left out -- and the force actuators are held DISENGAGED, so a frame fully
// describes the hardware state.
bool renderLevels(Airpack& airpack, const Peripheral& peripheral,
                  const std::unordered_map<int, float>& levels, float scale) {
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

  PneumaticFrame pneumatic_frame;
  if (!DirectPneumaticCalculator::addToPneumaticFrame(peripheral, haptic_frame, airpack,
                                                      &pneumatic_frame)) {
    printf("\nDirectPneumaticCalculator::addToPneumaticFrame() failed.\n");
    return false;
  }
  if (auto ret = airpack.render(pneumatic_frame); !ret) {
    printf("\nAirpack::render() returned error code %d: %s.\n", static_cast<int>(ret.error()),
           toString(ret.error()).c_str());
    return false;
  }
  return true;
}

// One-line view of the pressure image, rows separated by '|'.
std::string heatLine(const std::vector<float>& pressure, int rows, int cols) {
  static const char kRamp[] = " .:-=+*#%@";
  std::string line;
  for (int r = 0; r < rows; r++) {
    if (r > 0) line += '|';
    for (int c = 0; c < cols; c++) {
      const float p = pressure[static_cast<size_t>(r * cols + c)];
      line += kRamp[static_cast<int>(std::min(1.0f, std::max(0.0f, p)) * 9.0f + 0.5f)];
    }
  }
  return line;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }

  std::signal(SIGINT, onSigint);
  Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());

  printf("\nFSR MATRIX -> FINGERTIP\n");
  printf("=======================\n");
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

  // Pick the glove by handedness, not by slot, so it follows the glove.
  std::shared_ptr<Peripheral> glove;
  for (const auto& peripheral_it : *attached_peripherals) {
    const std::shared_ptr<Peripheral>& peripheral = peripheral_it.second;
    const auto* as_glove = dynamic_cast<const Glove*>(peripheral.get());
    if (as_glove != nullptr && as_glove->handedness == options.hand && !peripheral->is_simulated) {
      glove = peripheral;
      printf("Glove: slot %d, %s\n", peripheral_it.first, peripheral->casual_name.c_str());
      break;
    }
  }
  if (glove == nullptr) {
    printf("No %s glove attached (or it is simulated). Try --hand %s.\n",
           options.hand == RD_LEFT ? "left" : "right", options.hand == RD_LEFT ? "right" : "left");
    airpack->shutdown();
    return 1;
  }

  const CoverageRegion region = fingertipRegion(options.hand, options.finger);
  std::vector<TactorSpot> spots;
  printf("Region: %s\n", getName(region).getText().c_str());
  if (!mapTactors(*glove, region, options, &spots)) {
    printf("This glove reports no enabled tactors in %s.\n", getName(region).getText().c_str());
    airpack->shutdown();
    return 1;
  }
  printf("  %d tactors\n", static_cast<int>(spots.size()));

  if (options.list_only) {
    printf("\n  tactor   u (rows)   v (cols)   nearest cell of a 4x4\n");
    for (const TactorSpot& spot : spots) {
      const int r = std::min(3, static_cast<int>(spot.u * 4.0f));
      const int c = std::min(3, static_cast<int>(spot.v * 4.0f));
      printf("  %6d   %8.2f   %8.2f   r%dc%d\n", spot.id, spot.u, spot.v, r, c);
    }
    airpack->shutdown();
    return 0;
  }

  UdpReceiver receiver;
  if (!receiver.open(options.port)) {
    airpack->shutdown();
    return 1;
  }

  std::unordered_map<int, float> levels;
  bool ok = renderLevels(*airpack, *glove, levels, options.scale);  // start flat

  printf("\nListening on UDP %d. Start fsr_bridge.py on the host.\n", options.port);
  printf("Hands OFF the sensor -- taking a %.1f s baseline once frames arrive.\n",
         options.baseline_s);
  std::flush(std::cout);

  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / RENDER_RATE_HZ);
  SensorFrame frame;
  Baseline baseline;
  bool baselining = true;
  Clock::time_point baseline_start;
  Clock::time_point last_frame = Clock::now();
  Clock::time_point last_status = Clock::now();
  bool have_frame = false;
  int frames_received = 0;
  int frames_at_status = 0;
  std::vector<float> pressure;

  while (ok && !g_interrupted) {
    const Clock::time_point now = Clock::now();

    if (receiver.poll(&frame, &frames_received)) {
      last_frame = now;
      if (!have_frame || !baseline.matches(frame)) {
        // First frame, or the matrix size changed: start the baseline over.
        baseline.reset(frame.rows, frame.cols);
        baselining = true;
        baseline_start = now;
        printf("\nMatrix %dx%d. Baseline...", frame.rows, frame.cols);
        std::flush(std::cout);
      }
      have_frame = true;

      if (baselining) {
        baseline.add(frame);
        if (std::chrono::duration<double>(now - baseline_start).count() >= options.baseline_s) {
          baselining = false;
          printf(" done (%d frames). Press the sensor.\n", baseline.frames());
          std::flush(std::cout);
        }
      }
      if (!baselining) {
        pressure = toPressure(frame, baseline, options);
      }
    }

    // Stale data never leaves the finger inflated, and a pressure image is only
    // sampled when it matches the frame it is indexed by.
    const double age_ms = std::chrono::duration<double, std::milli>(now - last_frame).count();
    const bool live = have_frame && !baselining && age_ms <= options.timeout_ms &&
                      pressure.size() == static_cast<size_t>(frame.rows * frame.cols);

    float peak = 0.0f;
    for (const TactorSpot& spot : spots) {
      const float level =
          live ? samplePressure(pressure, frame.rows, frame.cols, spot.u, spot.v) : 0.0f;
      levels[spot.id] = level;
      peak = std::max(peak, level);
    }
    ok = renderLevels(*airpack, *glove, levels, options.scale);

    if (std::chrono::duration<double>(now - last_status).count() >= 0.2) {
      const double dt = std::chrono::duration<double>(now - last_status).count();
      const double rate = (frames_received - frames_at_status) / dt;
      if (live) {
        printf("\r  %5.1f frames/s  peak %3.0f%%  [%s]      ", rate, peak * 100.0f,
               heatLine(pressure, frame.rows, frame.cols).c_str());
      } else if (have_frame && !baselining) {
        printf("\r  NO DATA for %.0f ms -- deflated                         ", age_ms);
      }
      std::flush(std::cout);
      last_status = now;
      frames_at_status = frames_received;
    }

    std::this_thread::sleep_for(period);
  }

  // Always leave the hardware flat, including after Ctrl-C or a failure.
  printf("\nDeflating.\n");
  levels.clear();
  renderLevels(*airpack, *glove, levels, options.scale);
  airpack->shutdown();

  printf("%s\n", ok ? "Finished." : "Finished with errors.");
  std::flush(std::cout);
  return ok ? 0 : 1;
}
