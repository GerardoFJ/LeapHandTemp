
#include "socket_sender/socket_sender.hpp"

#include <HaptxApi/airpack.h>
#include <HaptxApi/alternate_hand_ik.h>
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
#include <atomic>
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

#include "haptx/haptx.hpp"
#include "iklib/iklib.hpp"

// Use HaptX library namespace
using namespace HaptxApi;
constexpr double PI = 3.14159265358979323846;


std::atomic<bool> running{true};

void on_sigint(int) { running = false; }

int main(int argc, char** argv) {
  // Options Declaration
  Options opts("ik_stream");

  HaptXConfig haptxconfig; 
  IKConfig ikconfig; 

  haptxconfig.register_options(opts);
  ikconfig.register_options(opts);

  opts.parse(argc, argv);
  if (haptxconfig.process_options() || ikconfig.process_options()) return 1;
  
  std::signal(SIGINT, on_sigint);


  printf("\nHAPTX IK STREAM\n");
  printf("===============\n");
  std::flush(std::cout);

  //////////////////////////////////////////////////////////// HAPTX migrate
  // HaptX initialization
  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);
 
  // Detect first airpack
  const std::shared_ptr<Airpack> airpack =
  haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  if (airpack == nullptr) {
    printf("No Airpack found. Is it powered, connected over USB, and are the USB\n");
    printf("devices bound to WinUSB/USBCCGP in this Wine prefix?\n");
    return 1;
  }

  // Search for peripherals and mocap_systems
  auto attached_peripherals = airpack->getAttachedPeripherals();
  auto mocap_systems = airpack->getMocapSystems();
  if (!attached_peripherals || !mocap_systems) {
    printf("Airpack did not report its peripherals or mocap systems.\n");
    airpack->shutdown();
    return 1;
  }

 //Save gloves and mocap system 
  for (const auto& peripheral_it : *attached_peripherals) {
    const auto* glove = dynamic_cast<const Glove*>(peripheral_it.second.get());
    if (glove == nullptr || haptxconfig.hand_selection.find(glove->handedness) == haptxconfig.hand_selection.end() ) {
      continue;
    }
    const auto it = mocap_systems->find(peripheral_it.first);
    if (it != mocap_systems->end() && it->second != nullptr) {
        std::pair<std::shared_ptr<MocapSystem>,std::shared_ptr<Glove>> glove_data; 
        glove_data.first = it->second;
        haptxconfig.gloves.push_back(glove_data);
        printf("Glove added: slot %d, %s\n", peripheral_it.first,
        peripheral_it.second->casual_name.c_str());
    }
  }

  //check the mockap and gloves system
  for(auto& g : haptxconfig.gloves){
    if (g.first == nullptr || haptxconfig.gloves.size() != haptxconfig.hand_selection.size()) {
      printf("Error fetching one of the required gloves.\n");
      airpack->shutdown();
      return 1;
    }
    if (auto ret = g.first->init(); !ret) {
      printf("MocapSystem::init() returned %s.\n", toString(ret.error()).c_str());
      airpack->shutdown();
      return 1;
    }
    g.second = g.first->getGlove();
    if (g.second == nullptr) {
      printf("MocapSystem::getGlove() returned nullptr.\n");
      g.first->shutdown();
      airpack->shutdown();
      return 1;
    }
  }

  // Get default profile data
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
 ////////////////////////////////////////////////////////////////////////////////
  ///
  // START UDP SENDER
  UdpSender sender;
  if (!sender.open(haptxconfig.host, haptxconfig.port)) {
    for(const auto& g : haptxconfig.gloves){
      g.first->shutdown();
    }
    airpack->shutdown();
    return 1;
  }
  printf("Streaming to %s:%d at %.0f Hz. Ctrl-C to stop.\n\n", haptxconfig.host.c_str(), haptxconfig.port,
        haptxconfig.rate_hz);
  std::flush(std::cout);

///////////////////////////////////////////////////////////////////////////////
  ///
  ///
  //PREPARING CLOCKS
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration<double>(1.0 / haptxconfig.rate_hz);
  const Clock::time_point start = Clock::now();
  Clock::time_point next = start;
  Clock::time_point last_table = start;
  std::unordered_map<RelativeDirection,unsigned long> seq = {
      {RD_LEFT, 0},
      {RD_RIGHT, 0}
  };
  std::unordered_map<RelativeDirection,int> ik_failures = {
      {RD_LEFT, 0},
      {RD_RIGHT, 0}
  };
  std::unordered_map<RelativeDirection,int> frame_failures = {
      {RD_LEFT, 0},
      {RD_RIGHT, 0}
  };
  //TO CHECK
  int lines_drawn = 0;

  AlternateHandIk::Config ik_setup;
  ik_setup.legacy_blend_enabled = ikconfig.legacy_blend;
  std::unordered_map<Finger,FingerAngleHint> i_hint;
  std::unordered_map<RelativeDirection,std::unordered_map<Finger,FingerAngleHint>> hints = {
    {RD_LEFT, i_hint},
    {RD_RIGHT, i_hint}
  };
  printf("IK: %s blend: %d\n\n", ikconfig.ik_method.c_str(), ikconfig.legacy_blend);


  while(running){
    for(auto& g: haptxconfig.gloves){
      RelativeDirection g_side = g.second->handedness;
      char hand_letter = g_side == RD_LEFT ? 'L' : 'R';
      g.first->update();
      MocapFrame mocap_frame;
      AnimFrame anim_frame;

      anim_frame.hints = hints[g_side];
      if (g.first->addToMocapFrame(&mocap_frame)) {
        const bool solved =
            ikconfig.ikmode
                ? AlternateHandIk::addToAnimFrame(g.second, ik_setup, mocap_frame, profile, &anim_frame)
                : DefaultHandIk::addToAnimFrame(g.second, mocap_frame, profile, &anim_frame);
        if (solved) {
          hints[g_side] = anim_frame.hints;
        } else {
          ik_failures[g_side]++;
        }
      } else {
        frame_failures[g_side]++;
      }


      FingerData fingers[F_LAST];
      const long long t_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
      char buffer[2048];
      int length = snprintf(buffer, sizeof(buffer), "H2,%c,%lu,%lld", hand_letter, seq[g_side]++, t_ms);
      for (unsigned int finger = 0; finger < F_LAST; finger++) {
        fingers[finger] = fingerFrom(anim_frame, g_side, static_cast<Finger>(finger));
        const FingerData& f = fingers[finger];
        length += snprintf(buffer + length, sizeof(buffer) - length, ",%d,%.4f,%.4f,%.4f,%.4f,%d",
                          f.has_angles ? 1 : 0, f.joint_rad[FJ_JOINT1], f.joint_rad[FJ_JOINT2],
                          f.joint_rad[FJ_JOINT3], f.adduction_rad, f.has_orientations ? 1 : 0);
        for (const Quaternion& q : f.orientations) {
          length += snprintf(buffer + length, sizeof(buffer) - length, ",%.5f,%.5f,%.5f,%.5f",
                            q.w_, q.x_, q.y_, q.z_);
        }
      }
      //Send the data to socket
      sender.send(std::string(buffer, static_cast<size_t>(length)));

      //Print data
      // printf("%s,,,%i,%i\n",buffer,ik_failures[g_side],frame_failures[g_side]);
      /// TEMPORAL TEST
      const Clock::time_point now = Clock::now();

      if (std::chrono::duration<double>(now - last_table).count() >= 0.1 && g_side == RD_LEFT) {
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
      printf("  sent %-8lu  mocap frame fail %-5d  ik fail %-5d\n", seq[g_side], frame_failures[g_side],
             ik_failures[g_side]);
      lines_drawn = F_LAST + 2;
      std::flush(std::cout);
      last_table = now;
    }

 //////////////////////////////////////////////////////////////////////////
      }

    //Update current time
    const Clock::time_point now = Clock::now();
    next += std::chrono::duration_cast<Clock::duration>(period);
    if (next < now) {
      next = now;  // fell behind: don't try to catch up in a burst
    }
    std::this_thread::sleep_until(next);


  }

  printf("Shuting down.\n");
  TerminateProcess(GetCurrentProcess(), 0);
  // Correct way to shutdwon system, but usb still have some work to do
  // for(auto& g: haptxconfig.gloves){
  // g.first->shutdown();
  // }
  // airpack->shutdown();
  return 0;

}
