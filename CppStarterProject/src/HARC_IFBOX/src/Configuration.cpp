#include <HARC_IFBOX/Configuration.h>

#include <yaml-cpp/yaml.h>

namespace harc_ifbox
{

namespace
{

DataMatrix::Coord loadCoord(const YAML::Node & node)
{
  return {node["x"].as<size_t>(), node["y"].as<size_t>()};
}

DataMatrix::Mapping loadMapping(const YAML::Node & node)
{
  return {loadCoord(node["dst"]), loadCoord(node["src"])};
}

std::optional<DataMatrix::Mappings> loadMappings(const YAML::Node & node)
{
  if(!node || node.size() == 0) { return std::nullopt; }
  DataMatrix::Mappings out;
  out.reserve(node.size());
  for(const auto & n : node) { out.push_back(loadMapping(n)); }
  return out;
}

Configuration::Patch loadPatch(const YAML::Node & node, size_t default_rows, size_t default_cols)
{
  return {node["name"].as<std::string>(), node["rows"].as<size_t>(default_rows), node["cols"].as<size_t>(default_cols),
          loadMappings(node["mappings"])};
}

Configuration::Patches loadPatches(const YAML::Node & node, size_t default_rows, size_t default_cols)
{
  if(!node || node.size() == 0) { return {Configuration::Patch{std::string("default"), default_rows, default_cols}}; }
  Configuration::Patches out;
  out.reserve(node.size());
  for(const auto & n : node) { out.push_back(loadPatch(n, default_rows, default_cols)); }
  return out;
}

Configuration loadYAML(const YAML::Node & node)
{
  Configuration out;
  out.device = node["device"].as<std::string>();
  out.baudrate = node["baudrate"].as<uint32_t>(Configuration::DEFAULT_BAUDRATE);
  out.rows = node["rows"].as<size_t>(MAX_ROWS);
  out.cols = node["cols"].as<size_t>(MAX_COLS);
  out.patches = loadPatches(node["patches"], out.rows, out.cols);
  return out;
}

} // namespace

Configuration Configuration::loadFile(const std::string & f)
{
  return loadYAML(YAML::LoadFile(f));
}

Configuration Configuration::loadData(const char * data)
{
  return loadYAML(YAML::Load(data));
}

} // namespace harc_ifbox
