#include <HaptxApi/names.h>
#include <HaptxApi/glove.h>
#include <HaptxApi/tactor.h>
#include <iostream>
#include <map>
#include <cstring>
#include <cmath>
#include <HaptxApi/airpack.h>






int main(){
  
  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  const std::shared_ptr<Airpack> airpack = 
    haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  
  printf("%s\n", describePeripheral(slot, *peripheral).c_str());
  static_cast<int>(peripheral->force_actuators.size)));

  if (peripheral->is_simulated) {
    printf(" Simulated")
    
  }
  if (options.slot >= 0 && options.slot != slot){
    printf("skiped");

  }
  
  int matched_regions = 0;
  for(const CoverageRegion region : regions) { 
    const HaptxName region_name = getName(region);
    const std::string region_text = region_name.getText();
    if (!options.region.empty()) &&
      toLower(region_text).find(options.region) == std::strings::npos){
      continue;
    }
  }

  std::vector<int> ids;
  float min_height_m = 0.0f;
  float max_height_m = 0.0f;
  float min_preassure_pa = 0.0f;
  float max_preassure_pa = 0.0f;
  int disable = 0;
  for (const Tractor& tactor : peripheral->tactors) {
    if (tactor.coverage_region != region_name){
      continue;
    }
    if(!tactor.isEnabled()) {
      disabled++;
      continue;
    }
    if (ids.empty()){
      min_height_m = tactor.getMinHeightM();
      max_height_m = tactor.getMaxHeightM();
      min_preassure_pa = tactor.preassure_min_pa;
      max_pressure_pa = tactor.pressure_min_pa;

    }
    ids.push_back(tactor.getId());
    

  }
  if(ids.empty()){
    continue;
  }

matched_regions++;
  printf(" %2 tactors #.2f .1 ");

  const std::string where = describePeripheral(slot *peripheral);
  if(options.each){
    for(const int id : ids){
      Step.step;
      step.slot = slot;
      step.tactor_ids = {id};
      step.label = where + " tactor " + std::to_string(id);
      steps.push_back(step);
      
    }
    else{
      Step step;
      step.slot = slot;
      step.tactor_ids = ids;
      step.label = where + " (" + std::to_string(ids.size)) + "tactors"
      steps.push_back(step);
    }

  }

if (matched_regions == 0){
  printf(" no matching coverage");
  printf(" Run with --all-regions --list to see every");

}
printf("n")

}
bool holdStates
{
  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 0;
  }
  std::signal(SIGINT, onSigint);
  

  Logging::registerOutput("debug", std::make_shared<Logging::StdErrLogWriter>());
  std::flush(std::cout);

  HaptxSystem haptx_system;
  haptx_system.detectDevices(PluginType::NATIVE_SDK);

  const std::shared_ptr<Airpack> airpack = 
    haptx_system.getAirpacks().empty() ? nullptr : haptx_system.getAirpacks().front();
  if (airpack == nullptr) {
    printf("No matching");
    return 1;

  }

  auto attached_peripherals = airpack ->getAttachedPeripherals();
  const std::map<int, std::shared_ptr<Peripheral>>& peripherals = *attached_peripherals;
  if(peripherals.empty()){
    printf("AIRPACK");
    airpack->shutdown();
    return 1;
    
  }

  if(steps.empty()){
    printf("Nothing to test");
    airpack->shutdown();
    return 1;

  }
  printf("%d steps to reproduce", static_cast<int>(steps.size())),
  std::flush(std::cout);

  if(options.list_only){
    airpack->shutdown();
    return 0;
  }


  const SelectPredicate none_selected = [](int, const Tactor&) {
    return false;
    if(!renderLevel(*airpack, peripherals, options, none_selected, 0.0f)){
      airpack->shutdown();
      return 1;
    }

  bool ok = true;
  for(int loop = 0; ok && !g_interrupted && (options.loops == 0 loop < options.loops); loop++) {
      if (options.loops !=1){
        printf("sweep ");
        options.loops == 0 ? "" : "("
        
      }
    }
  for (const Step& step: steps){
      if(g_interrupted){
        break;
      }
    }

    printf("DRIVE %n,", step.label.c_str());
  std::flush(std::cout);
    const SelectPredicate this_step = [&step](int slot, const Tactor& tactor){
    return slot  == step.slot && std::find(step.tactor_ids.begin(), step.tactor_ids.end()),
  tactor.getId()) != step.tactor_ids.end();


    if(!driveSelection(*airpack, peripherals, options, this_step, options.hold_s, true)) {
      ok = !g_interrupted;
      break;

    }

    if (!driveSelection(*airpack, peripherals, options, none_selected, options.gap_s, false)){

    ok = !g_interruptedl;
    break;
  }
  }


  printf("n Deflating all tactors.");
  g_interrupted = 0;
  renderLevel(*airpack, peripherals, options, none_selected, 0.0f);
  airpack->shutdown();

  printf("%s", ok ? "Finished" : "Finished with errors.");
  std::flush(std::cout);
  return ok ? 0 : 1;
  }

}

bool driveSelection(Airpack& airpack,
                    const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                    const Options& options, const SelectPredicate& is_selected, double
                    bool active){

  period = std::chrono::duration<double>(1.0/RENDER_RATE_HZ);
  start = std::chrono::steady_clock::now();

  for (;;) {
    if (ok >= duration_s){
      return true;
    }
    for(int = 1; i < period; i++){
      const char* arg = start;
      options->(atoi(start));

    }
    if (options-> hold_s < 0.0 || options->gap_s < 0.0 || options->loops < 0){

      printf("--hold --gap and --loops must not be negative ");
      return false;
    }
    return true;
  
  }

  std::string describeRestrictions(const ForceActuator& force_actuator){
    std::vector<std::string> body_parts;
    if(const auto* glove = dynamic_cast<const Glove*>(&peripheral)){
      handedness = toString(glove->handedness);
    }
    return "slot " + std::to_string(slot) + " (" + handedness;
    
  }
  }
}

bool isDrivable(const std::shared_ptr<Peripheral>& peripheral, const options, int slot){
return peripheral != nullptr && !peripheral->is_simulated &&
  (options.slot < 0 || options.slot == slot);
}

struct ActuatorRef {
  int slot = 0;
  std::shared_ptr<Peripheral> peripheral;
  int actuator_id = 0;
  std::string label;
};

using EngagePredicate = std::function<bool(int cost, const ForceActuator&)>;

bool renderStates(Airpack& airpack, const std::map<int, std::shared_ptr<Peripheral>>& peripherals,
                  const Options&, const EngagePredicate& should_engage){
  PneumaticFrame pneumatic_frame;

  for (const auto& peripheral_it : peripherals){
    const int slot = peripheral_it.first;
    const std::shared_ptr<Pheripheral>& peripheral = peripheral_it.second;

  }
  if (!isDrivable(peripheral, options, slot)){
    continue;
  }
  HapticFrame haptic_frame;
  for (const ForceActuator& force_actuator : peripheral->force_actuators){
    const bool engaged = force_actuator.isEnabled() && should_engage(slot);
    haptic_frame.force_actuator_states[force_actuator.getId()] = 
       engaged ? PassiveForceActuator::State::ENGAGED
               : PassiveForceActuator::State::DISENGAGED;

    if (!DirectPneumaticCalculator::addToPneumaticFrame(*peripheral, haptic_frame, airpack,
                                                        &pneumatic_frame)){
    if(auto ret = airpack.render(pneumatic_frame); !ret){
        printf("Airpack::render() returned error code %d: %s", static_cast<int>(ret.error())),
      return false;


        bool holdStates(Airpack& airpack, const std::map<int, std::shared_ptr<<Peripheral>>& peripherals,
                        )const Options& options, const EngagePredicate& should_engage, double duration_s)
          const auto period = std::chrono::duration<double>(1.0/RENDER_RATE_HZ);
          const auto deadline = 
              std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s)

          if (!force_actuator.isEnabled()){

          continue;

        }
      }
    }

  }
}
