#include <thread>
#include <boost/cstdlib.hpp>
#include <exception>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <scenario_logger/logger.h>
#include <scenario_runner/scenario_runner.h>
#include <scenario_runner/scenario_terminater.h>


static scenario_runner::ScenarioTerminator terminator { "0.0.0.0", 10000 };

static void failureCallback()
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Simulation failed unexpectedly.");
  scenario_logger::log.write();
}

int main(int argc, char * argv[]) try
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureFunction(&failureCallback);

  rclcpp::init(argc, argv);

  /*
   * setup scenario runner
   */
  auto runner = std::make_shared<scenario_runner::ScenarioRunner>();
  SCENARIO_INFO_STREAM(CATEGORY("simulation", "progress"), "ScenarioRunner instantiated.");

  scenario_logger::log.setStartDatetime(runner->now());
  SCENARIO_LOG_STREAM(CATEGORY("simulation", "progress"), "Logging started.");

  std::string scenario_id = runner->declare_parameter("scenario_id").get<std::string>();
  scenario_logger::log.setScenarioID(scenario_id);

  std::string log_output_path = runner->declare_parameter("log_output_path").get<std::string>();
  scenario_logger::log.setLogOutputPath(log_output_path);

  SCENARIO_INFO_STREAM(CATEGORY(), "Sleep for 10 seconds.");
  std::this_thread::sleep_for(std::chrono::seconds { 10 });
  SCENARIO_INFO_STREAM(CATEGORY(), "Wake-up.");


  /*
   * start simulation
   */
  for (runner->run(); rclcpp::ok(); rclcpp::spin_some(runner))
  {
    static auto previously{runner->currently};

    if (previously != runner->currently)
    {
      switch (previously = runner->currently)
      {
      case simulation_is::succeeded:
        SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation succeeded");
        scenario_logger::log.write();
        terminator.sendTerminateRequest(boost::exit_success);
        return boost::exit_success;

      case simulation_is::failed:
        SCENARIO_INFO_STREAM(CATEGORY("simulator", "endcondition"), "simulation failed");
        scenario_logger::log.write();
        terminator.sendTerminateRequest(boost::exit_test_failure);
        return boost::exit_test_failure;

      case simulation_is::ongoing:
        break;
      }
    }

    terminator.update_mileage(runner->current_mileage());

    terminator.update_duration(
      (runner->now() - scenario_logger::log.begin()).seconds());
  }

  if (runner->currently == simulation_is::ongoing)
  {
    SCENARIO_INFO_STREAM(CATEGORY(), "Simulation aborted.");
    scenario_logger::log.write();
    terminator.sendTerminateRequest(boost::exit_failure);
    return boost::exit_failure;
  }
  else
  {
    SCENARIO_INFO_STREAM(CATEGORY(), "Simulation unexpectedly failed.");
    scenario_logger::log.write();
    return boost::exit_exception_failure;
  }
}

catch (const std::exception& e)
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected standard exception thrown: " << e.what());
  scenario_logger::log.write();
  terminator.sendTerminateRequest(boost::exit_exception_failure);
  return boost::exit_exception_failure;

}

catch (...)
{
  SCENARIO_ERROR_STREAM(CATEGORY("simulator", "endcondition"), "Unexpected non-standard exception thrown.");
  scenario_logger::log.write();
  terminator.sendTerminateRequest(boost::exit_exception_failure);
  return boost::exit_exception_failure;
}
