/**
 * freertos/task.h — Host stub for task creation, plus the deterministic
 *                   driver that lets a test run a FreeRTOS task loop.
 *
 * THE PROBLEM.  TR_LogToFlash's close path — the tail-page write, the
 * last_closed_session_bytes_ snapshot, the ring-cap restore — is reachable
 * ONLY from flushTaskLoop(), which is `while (!flush_task_stop_) { ... }`
 * with no public way to set that flag.  service() is the single-threaded
 * fallback and handles openLogSession / flushRingToNand, but not the
 * end-of-flight close.  So a harness for the close accounting has to run
 * the task loop, and then get back out of it.
 *
 * THE MECHANISM.  xTaskCreatePinnedToCore does not start anything: it
 * records (entry, param) so the test can decide when the loop runs.
 * runPendingTasks(n) then calls the entry ON THE TEST'S OWN THREAD with a
 * budget of n yields.  vTaskDelay() spends one unit of that budget per call
 * and throws TaskYieldBudgetExhausted when it runs out, which unwinds the
 * loop back to the runner.  Every path through flushTaskLoop ends in at
 * least one vTaskDelay, so the budget is always reachable.
 *
 * WHY A BUDGET RATHER THAN A FLAG.  A bounded budget makes a WEDGE fail as
 * a failed assertion instead of a hung CI job.  The #271 regression this
 * harness exists to guard is exactly that shape: if flushRingToNand stops
 * resetting page_buf_idx after a rejected page, the end-of-flight
 * `while (remaining > 0) { flushRingToNand(); vTaskDelay(1); }` never
 * drains and spins forever.  Under a budget it spins n times, unwinds, and
 * the test reports a ring that did not empty.
 *
 * WHAT THE UNWIND SKIPS.  flushTaskEntry sets flush_task_running_ = false
 * after the loop returns; an unwound loop never gets there, so
 * flush_task_running_ stays true and service() keeps no-op'ing afterwards.
 * That is correct for a test that has committed to driving the task loop —
 * do not mix the two drivers in one test.
 *
 * The task body holds no locks at a vTaskDelay (the push mutex is taken and
 * given inside the end-flight block, well before its drain loop yields), so
 * the unwind leaks nothing.
 */
#pragma once

#include <cstdint>
#include <vector>

#include "FreeRTOS.h"

typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

namespace _host_shim {

/// Thrown by vTaskDelay to unwind a task loop once its yield budget is spent.
/// Caught by runPendingTasks; never escapes to the test body.
struct TaskYieldBudgetExhausted {};

struct PendingTask {
    TaskFunction_t entry;
    void*          param;
};

inline std::vector<PendingTask>& pendingTasks()
{
    static std::vector<PendingTask> v;
    return v;
}

/// Remaining yields for the task currently running under runPendingTasks.
/// -1 means "no task is being driven", which makes vTaskDelay a plain no-op
/// for code that calls it outside the runner.
inline int& yieldBudget()
{
    static int budget = -1;
    return budget;
}

/// Forget every registered task.  Call from a fixture's SetUp/TearDown: the
/// registry is process-global, and a task left over from a destroyed object
/// would be re-entered with a dangling `this`.
inline void clearTasks() { pendingTasks().clear(); }

/// Run each registered task entry on the calling thread until it has yielded
/// `max_yields` times.  Returns after every task has unwound.
inline void runPendingTasks(int max_yields)
{
    for (const PendingTask& t : pendingTasks())
    {
        yieldBudget() = max_yields;
        try
        {
            t.entry(t.param);
        }
        catch (const TaskYieldBudgetExhausted&)
        {
        }
        yieldBudget() = -1;
    }
}

}  // namespace _host_shim

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t entry,
                                          const char* name,
                                          uint32_t stack_depth,
                                          void* param,
                                          UBaseType_t priority,
                                          TaskHandle_t* created,
                                          BaseType_t core)
{
    (void)name; (void)stack_depth; (void)priority; (void)core;
    _host_shim::pendingTasks().push_back({entry, param});
    // Non-null so the caller's "already started" guard latches, exactly as it
    // does on target.  The value is never dereferenced.
    if (created) *created = reinterpret_cast<TaskHandle_t>(_host_shim::pendingTasks().size());
    return pdPASS;
}

inline BaseType_t xTaskCreate(TaskFunction_t entry, const char* name,
                              uint32_t stack_depth, void* param,
                              UBaseType_t priority, TaskHandle_t* created)
{
    return xTaskCreatePinnedToCore(entry, name, stack_depth, param, priority,
                                   created, 0);
}

inline void vTaskDelete(TaskHandle_t) {}

inline void vTaskDelay(TickType_t)
{
    int& budget = _host_shim::yieldBudget();
    if (budget < 0) return;            // not under the runner
    if (budget == 0) throw _host_shim::TaskYieldBudgetExhausted{};
    --budget;
}

/// One fixed non-null handle: the component uses this only for identity
/// comparison (the #834 parkSpiBusForReset owner check), and a single-threaded
/// harness has exactly one identity.
inline TaskHandle_t xTaskGetCurrentTaskHandle()
{
    static int self = 0;
    return static_cast<TaskHandle_t>(&self);
}
