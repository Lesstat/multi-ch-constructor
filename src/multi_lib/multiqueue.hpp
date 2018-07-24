/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MULTIQUEUE_H
#define MULTIQUEUE_H

#include <condition_variable>
#include <deque>
#include <mutex>

template <class T> class MultiQueue {
  public:
  MultiQueue(size_t size = 5000000)
      : maxSize(size)
  {
  }
  MultiQueue(const MultiQueue& other) = default;
  MultiQueue(MultiQueue&& other) = default;
  virtual ~MultiQueue() noexcept = default;
  MultiQueue& operator=(const MultiQueue& other) = default;
  MultiQueue& operator=(MultiQueue&& other) = default;

  void send(const T& value)
  {
    std::unique_lock guard(key);
    non_full.wait(guard, [this] { return maxSize > fifo.size(); });
    fifo.push_back(value);
    non_empty.notify_one();
  }

  void send(std::vector<T>& values)
  {
    size_t valueSize = values.size();
    if (valueSize >= maxSize) {
      std::invalid_argument("Vector to big to add to queue");
    }
    std::unique_lock guard(key);
    non_full.wait(guard, [this, &valueSize] { return maxSize - valueSize > fifo.size(); });
    for (size_t i = 0; i < valueSize; ++i) {
      fifo.push_back(values[i]);
    }
    values.clear();
    non_empty.notify_all();
  }

  T receive()
  {
    std::unique_lock guard(key);
    non_empty.wait(guard, [this] { return !fifo.empty() || closed_; });
    if (fifo.empty() && closed_) {
      std::invalid_argument("Queue Closed and empty");
    }
    auto value = fifo.front();
    fifo.pop_front();
    non_full.notify_one();
    return value;
  }

  bool try_receive(T& value)
  {
    std::lock_guard guard(key);
    if (fifo.empty()) {
      return false;
    }
    value = fifo.front();
    fifo.pop_front();
    non_full.notify_one();
    return true;
  }

  size_t receive_some(std::vector<T>& container, size_t some)
  {
    std::unique_lock<std::mutex> guard(key);
    non_empty.wait(guard, [this] { return !fifo.empty() || closed_; });
    while (!fifo.empty() && container.size() < some) {
      container.push_back(fifo.front());
      fifo.pop_front();
    }
    non_full.notify_one();
    return container.size();
  }

  void close()
  {
    std::lock_guard guard(key);
    closed_ = true;
    non_empty.notify_all();
  }

  bool closed()
  {
    std::lock_guard guard(key);
    return closed_;
  }
  size_t size()
  {
    std::lock_guard guard(key);
    return fifo.size();
  }

  protected:
  private:
  std::mutex key;
  std::condition_variable_any non_empty;
  std::condition_variable_any non_full;
  std::deque<T> fifo;
  size_t maxSize;
  bool closed_ = false;
};

#endif /* MULTIQUEUE_H */
