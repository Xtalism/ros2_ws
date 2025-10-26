#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using namespace yasmin;

class FooState : public yasmin::State {
public:
  int counter;

  FooState()
      : yasmin::State({"outcome1", "outcome2", "outcome3"}), counter(0){};

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state FOO");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::string outcome;

    blackboard->set<std::string>("foo_str",
                                 "Counter: " + std::to_string(this->counter));

    if (this->counter < 3) {
      outcome = "outcome1";
    } else if (this->counter < 5) {
      outcome = "outcome2";
    } else {
      outcome = "outcome3";
    }

    YASMIN_LOG_INFO("Finishing state FOO");
    this->counter += 1;
    return outcome;
  };
};

class BarState : public yasmin::State {
public:
  BarState() : yasmin::State({"outcome3"}) {}

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) override {
    YASMIN_LOG_INFO("Executing state BAR");
    std::this_thread::sleep_for(std::chrono::seconds(4));

    if (blackboard->contains("foo_str")) {
      YASMIN_LOG_INFO(blackboard->get<std::string>("foo_str").c_str());
    } else {
      YASMIN_LOG_INFO("blackboard does not yet contains 'foo_str'");
    }

    YASMIN_LOG_INFO("Finishing state BAR");

    return "outcome3";
  }
};

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("yasmin_concurrence_demo");
  rclcpp::init(argc, argv);

  yasmin_ros::set_ros_loggers();

  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4"});

  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // Create states to run concurrently
  auto foo_state = std::make_shared<FooState>();
  auto bar_state = std::make_shared<BarState>();

  // Create concurrent state
  auto concurrent_state = std::make_shared<Concurrence>(
      std::map<std::string, std::shared_ptr<State>>{{"FOO", foo_state},
                                                    {"BAR", bar_state}},
      "defaulted",
      Concurrence::OutcomeMap{
          {"outcome1", Concurrence::StateOutcomeMap{{"FOO", "outcome1"},
                                                    {"BAR", "outcome3"}}},
          {"outcome2", Concurrence::StateOutcomeMap{{"FOO", "outcome2"},
                                                    {"BAR", "outcome3"}}}});

  // Add concurrent state to the state machine
  sm->add_state("CONCURRENCE", concurrent_state,
                {
                    {"outcome1", "CONCURRENCE"},
                    {"outcome2", "CONCURRENCE"},
                    {"defaulted", "outcome4"},
                });

  // Publish state machine updates
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_CONCURRENCE_DEMO", sm);

  // Execute the state machine
  try {
    std::string outcome = (*sm.get())();
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}