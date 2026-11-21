// Copyright 2026 RoboCup-FEI
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "fakes.hpp"
#include "manipulation_skill_server/busy_task_runner.hpp"

namespace manipulation_skill_server::test
{
namespace
{

struct SimpleResult
{
  Outcome outcome;
};

using Record = ResultRecord<SimpleResult>;

/// What a scripted task does when it runs.
enum class Behaviour
{
  Succeed,
  Throw,
};

/// A task that tells when it started and waits for the test to let it finish.
class GatedTask : public Task<SimpleResult>
{
public:
  GatedTask(
    std::shared_ptr<Record> record, std::shared_future<void> gate, Behaviour behaviour,
    std::promise<void> * started = nullptr)
  : Task<SimpleResult>(std::make_unique<FakeTaskContext<SimpleResult>>(std::move(record))),
    gate_(std::move(gate)),
    behaviour_(behaviour),
    started_(started)
  {
  }

protected:
  SimpleResult run() override
  {
    if (started_ != nullptr) {
      started_->set_value();
    }
    gate_.wait();
    if (behaviour_ == Behaviour::Throw) {
      throw std::runtime_error("arm state unknown");
    }
    return {{OutcomeCode::Success, "done"}};
  }

private:
  std::shared_future<void> gate_;
  Behaviour behaviour_;
  std::promise<void> * started_;
};

std::shared_future<void> open_gate()
{
  std::promise<void> gate;
  gate.set_value();
  return gate.get_future().share();
}

std::unique_ptr<GatedTask> instant_task(std::shared_ptr<Record> record)
{
  return std::make_unique<GatedTask>(std::move(record), open_gate(), Behaviour::Succeed);
}

TEST(BusyTaskRunner, RunsAcceptedTaskAndDeliversItsResult)
{
  BusyTaskRunner runner;
  auto record = std::make_shared<Record>();

  runner.submit(instant_task(record));

  ASSERT_TRUE(record->wait_for(1));
  ASSERT_EQ(record->results().size(), 1u);
  EXPECT_EQ(record->results()[0].outcome.code, OutcomeCode::Success);
}

TEST(BusyTaskRunner, RefusesTaskWithBusyWhileAnotherRuns)
{
  BusyTaskRunner runner;
  std::promise<void> gate;
  std::promise<void> started;
  auto running = std::make_shared<Record>();
  auto refused = std::make_shared<Record>();

  runner.submit(
    std::make_unique<GatedTask>(running, gate.get_future().share(), Behaviour::Succeed, &started));
  started.get_future().wait();

  runner.submit(instant_task(refused));

  ASSERT_EQ(refused->results().size(), 1u);  // delivered before submit() returned
  EXPECT_EQ(refused->results()[0].outcome.code, OutcomeCode::Busy);
  EXPECT_TRUE(running->results().empty());

  gate.set_value();
  ASSERT_TRUE(running->wait_for(1));
  EXPECT_EQ(running->results()[0].outcome.code, OutcomeCode::Success);
  EXPECT_EQ(refused->results().size(), 1u);
}

TEST(BusyTaskRunner, RefusesTaskWithBusyBeforeWorkerPicksUpAcceptedOne)
{
  // The worker may still be waiting for its first task: submit() must reserve it itself.
  BusyTaskRunner runner;
  std::promise<void> gate;
  auto running = std::make_shared<Record>();
  auto refused = std::make_shared<Record>();

  runner.submit(
    std::make_unique<GatedTask>(running, gate.get_future().share(), Behaviour::Succeed));
  runner.submit(instant_task(refused));

  ASSERT_EQ(refused->results().size(), 1u);
  EXPECT_EQ(refused->results()[0].outcome.code, OutcomeCode::Busy);
  gate.set_value();
  ASSERT_TRUE(running->wait_for(1));
}

TEST(BusyTaskRunner, WorkerIsFreeWhenCallerGetsResult)
{
  // The caller sends its next goal from inside the delivery of the previous result.
  BusyTaskRunner runner;
  auto first = std::make_shared<Record>();
  auto second = std::make_shared<Record>();
  first->on_result = [&runner, second]() {runner.submit(instant_task(second));};

  runner.submit(instant_task(first));

  ASSERT_TRUE(second->wait_for(1));
  EXPECT_EQ(second->results()[0].outcome.code, OutcomeCode::Success);
}

TEST(BusyTaskRunner, ThrowingTaskEndsWithInternalErrorAndFaultsRunner)
{
  BusyTaskRunner runner;
  auto failing = std::make_shared<Record>();
  auto later = std::make_shared<Record>();

  runner.submit(std::make_unique<GatedTask>(failing, open_gate(), Behaviour::Throw));
  ASSERT_TRUE(failing->wait_for(1));
  ASSERT_EQ(failing->results().size(), 1u);
  EXPECT_EQ(failing->results()[0].outcome.code, OutcomeCode::InternalError);
  EXPECT_NE(failing->results()[0].outcome.message.find("arm state unknown"), std::string::npos);

  // The fault is recorded before the failing task's result is delivered.
  runner.submit(instant_task(later));

  ASSERT_EQ(later->results().size(), 1u);
  EXPECT_EQ(later->results()[0].outcome.code, OutcomeCode::InternalError);
  EXPECT_NE(later->results()[0].outcome.message.find("arm state unknown"), std::string::npos);
}

TEST(BusyTaskRunner, RejectsNullTask)
{
  BusyTaskRunner runner;
  EXPECT_THROW(runner.submit(nullptr), std::invalid_argument);
}

TEST(BusyTaskRunner, DestructorWaitsForRunningTask)
{
  std::promise<void> gate;
  std::promise<void> started;
  auto record = std::make_shared<Record>();
  std::future<void> destroyed;
  {
    auto runner = std::make_unique<BusyTaskRunner>();
    runner->submit(
      std::make_unique<GatedTask>(record, gate.get_future().share(), Behaviour::Succeed, &started));
    started.get_future().wait();
    destroyed = std::async(std::launch::async, [&runner]() {runner.reset();});

    EXPECT_EQ(destroyed.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
    gate.set_value();
    destroyed.wait();
  }
  ASSERT_EQ(record->results().size(), 1u);
  EXPECT_EQ(record->results()[0].outcome.code, OutcomeCode::Success);
}

class InstantTask : public Task<SimpleResult>
{
public:
  using Task<SimpleResult>::Task;

protected:
  SimpleResult run() override {return {{OutcomeCode::Success, ""}};}
};

TEST(Task, RejectsNullContext) {
                                 EXPECT_THROW(InstantTask(nullptr), std::invalid_argument);
}

}  // namespace
}  // namespace manipulation_skill_server::test
