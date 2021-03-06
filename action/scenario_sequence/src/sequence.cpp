#include <scenario_sequence/sequence.h>

namespace scenario_sequence
{

Sequence::Sequence(
  const scenario_expression::Context& context,
  const YAML::Node& sequence_definition)
  : context_ { context }
  , name_ {sequence_definition["Name"].as<std::string>()}
  , ignited_ {false}
{
  event_manager_ =
    std::make_shared<scenario_sequence::EventManager>(
      context, sequence_definition["Events"]);

  if (const auto start_condition { sequence_definition["StartCondition"] })
  {
    start_condition_ = scenario_expression::read(context_, start_condition);
  }
  else // NOTE: If StartCondition unspecified, the sequence starts unconditionally.
  {
    ignited_ = true;
  }
}

simulation_is Sequence::update(
  const std::shared_ptr<scenario_intersection::IntersectionManager>&)
{
  if (ignited_ = start_condition_.evaluate(context_))
  {
    return (*event_manager_).update(context_.intersections_pointer());
  }
  else
  {
    return simulation_is::ongoing;
  }
}

} // namespace scenario_sequence

