// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// This implementation is based on the implementation found here:
// https://github.com/progschj/ThreadPool

#include "theia/util/threadpool.h"

#include <glog/logging.h>

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "theia/util/util.h"

namespace theia
{

  ThreadPool::ThreadPool(const int num_threads) : stop(false)
  {
    CHECK_GE(num_threads, 1)
        << "The number of threads specified to the ThreadPool is insufficient.";
    for (size_t i = 0; i < num_threads; ++i)
    {
      workers.emplace_back([this]
                           {
      for (;;) {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(this->queue_mutex);
          this->condition.wait(lock, [this] {
            return this->stop || !this->tasks.empty();
          });
          if (this->stop && this->tasks.empty()) return;
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }

        task();
      } });
    }
  }

  // the destructor joins all threads
  ThreadPool::~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
    }
    condition.notify_all();
    for (std::thread &worker : workers)
      worker.join();
  }

} // namespace theia
