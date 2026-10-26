#pragma once
#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>
#include <functional>
#include <sstream>

// Main class for parsing options
class Options {
public:
  // Initialize options CLI
  explicit Options(const std::string& name) : app_(name) {
    app_.add_option("-c,--config", config_path_, "YAML config file");
  }

  // One call = YAML key "teleop.rate" + CLI flag "--teleop.rate"
  template <typename T>
  void add(const std::string& key, T& field, const std::string& help = "") {
    auto* opt = app_.add_option("--" + key, field, help)->capture_default_str();
    loaders_.push_back([key, &field, opt](const YAML::Node& root) {
      if (opt->count() > 0) return; // CLI overrides
      auto n = lookup(root,key);
      if (!n.IsNull()) field = n.template as<T>(); // Save value as data type
    });
  }

  void parse(int argc, char** argv) {
    try { app_.parse(argc, argv); }
    catch (const CLI::ParseError& e) { std::exit(app_.exit(e)); }

    if (!config_path_.empty()) {
      YAML::Node root = YAML::LoadFile(config_path_);
      for (auto& load : loaders_) load(root);
    }
  }

private:
  static YAML::Node lookup(const YAML::Node& root, const std::string& key) {
    YAML::Node cur = YAML::Clone(root);
    std::stringstream ss(key);
    std::string part;
    while (std::getline(ss, part, '.')) {
      if (!cur.IsMap() || !cur[part]) return YAML::Node();
      cur.reset(cur[part]);   // reset, NOT '=' (yaml-cpp gotcha)
    }
    return cur;
  }

  CLI::App app_;
  std::string config_path_;
  std::vector<std::function<void(const YAML::Node&)>> loaders_;
};
