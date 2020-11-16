#ifndef CONDITION_PLUGINS_ACCELERATION_CONDITION_H_INCLUDED
#define CONDITION_PLUGINS_ACCELERATION_CONDITION_H_INCLUDED

#include <pluginlib/class_list_macros.hpp>

#include <scenario_api/scenario_api_core.h>
#include <scenario_conditions/condition_base.h>
#include <scenario_intersection/intersection_manager.h>
#include <scenario_logger/logger.h>
#include <scenario_utility/scenario_utility.h>

namespace condition_plugins
{

class AccelerationCondition
  : public scenario_conditions::ConditionBase
{
public:
  AccelerationCondition();

  bool update(const std::shared_ptr<scenario_intersection::IntersectionManager> &) override;
  bool configure(YAML::Node node, std::shared_ptr<ScenarioAPI> api_ptr) override;

private:
  std::string trigger_;
  float value_;
  Comparator<float> compare_;
};

}  // namespace condition_plugins

#endif  // CONDITION_PLUGINS_ACCELERATION_CONDITION_H_INCLUDED
