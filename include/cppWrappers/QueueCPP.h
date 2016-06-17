/**
 * @file QueueCPP.h
 * @brief FreeRTOS Queue Wrapper
 *
 * This file contains a set of lightweight wrappers for queues using FreeRTOS
 * 
 * @copyright (c) 2007-2015 Richard Damon
 * @author Richard Damon <richard.damon@gmail.com>
 * @parblock
 * MIT License:
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * It is requested (but not required by license) that any bugs found or 
 * improvements made be shared, preferably to the author.
 * @endparblock
 * 
 * @ingroup FreeRTOSCpp
 */
#ifndef QUEUECPP_H
#define QUEUECPP_H

#include "FreeRTOS.h"
#include "queue.h"

//#include <type_traits>

/**
 * @brief Queue Wrapper.
 *
 * Note, is a template on the type of object to place on the queue, 
 * which makes the Queue more typesafe.
 *
 * @tparam T The type of object to be placed on the queue.
 * Note also, this type needs to be trivially copyable, and preferably a POD 
 * as the FreeRTOS queue code will copy it with memcpy().
 *
 * @todo add Overwrite operation
 * @todo add QueueSet Functionality
 * @ingroup FreeRTOSCpp
 */

template<class T> class Queue {
//	static_assert(::std::is_pod<T>, "Queues only work with PODs"); ///@todo need to get later version of C++ compile working
public:
	/**
	 * @brief Constructor.
	 * @param length How many object to make room for in the Queue.
	 * @param name The name to register the Queue with.
	 */
  Queue(unsigned portBASE_TYPE length, char const* name = 0){
    handle = xQueueCreate(length, sizeof(T));
#if configQUEUE_REGISTRY_SIZE > 0
    if(name)
      vQueueAddToRegistry(handle, name);
#endif

  };

  /**
   * @brief Destructor.
   */
  ~Queue() {
    vQueueDelete(handle);
  }

  /**
   * @brief Get number of items in the Queue.
   * @return The number of item in the Queue.
   */
  unsigned waiting() const {
    return uxQueueMessagesWaiting(handle);
  }

  /**
   * @brief Return number of spaces available in Queue
   * @return the number of spaces available in the Queue.
   */
  unsigned available() const {
	  return uxQueueSpacesAvailable(handle);
  }

  /**
   * @brief Reset the Queue.
   * Resets the Queue to an empty state.
   */
  void reset() {
	  xQueueReset(handle);
  }
  /**
   * @brief Push an item onto the Queue.
   * Puts an item onto the Queue so it will be the next item to remove.
   * @param item The item to put on the Queue.
   * @param time How long to wait for room if Queue is full.
   * @return True if successful
   */
  bool push(T const& item, TickType_t time = portMAX_DELAY){
    return xQueueSendToFront(handle, &item, time);
  }

  /**
   * @brief add an item at end of the Queue.
   * Puts an item onto the Queue so it will be the last item to remove.
   * @param item The item to put on the Queue.
   * @param time How long to wait for room if Queue is full.
   * @return True if successful
   */
  bool add(T const& item, TickType_t time = portMAX_DELAY){
    return xQueueSendToBack(handle, &item, time);
  }

  /**
   * @brief Get an item from the Queue.
   * Gets the first item from the Queue
   * @param var Variable to place the item
   * @param time How long to wait for an item to be available.
   * @return True if an item returned.
   */
  bool pop(T& var, TickType_t time = portMAX_DELAY) {
    return xQueueReceive(handle, &var, time);
  }

  /**
   * @brief Look at the first item in the Queue.
   * Gets the first item from the Queue leaving it there.
   * @param var Variable to place the item
   * @param time How long to wait for an item to be available.
   * @return True if an item returned.
   */
  bool peek(T& var, TickType_t time = portMAX_DELAY) {
    return xQueuePeek(handle, &var, time);
  }

  /**
   * @brief Check if Queue is Full.
   * @return True if Queue is Full.
   */
  bool full() {
    return 0 < uxQueueSpacesAvailable(handle);
  }

  /**
   * @brief Check if Queue is Empty
   * @return True if Queue is Empty.
   */
  bool empty() {
    return uxQueueMessagesWaiting(handle) == 0;
  }

  /**
   * @brief Push an item onto the Queue.
   * Puts an item onto the Queue so it will be the next item to remove.
   *
   * Note: Interrupt service routines should only call _ISR routines.
   * @param item The item to put on the Queue.
   * @param waswoken Flag variable to determine if context switch is needed.
   * @return True if successful
   */
  bool push_ISR(T const& item, portBASE_TYPE& waswoken){
    return xQueueSendToFrontFromISR(handle, &item, &waswoken);
  }

  /**
   * @brief add an item at end of the Queue.
   * Puts an item onto the Queue so it will be the last item to remove.
   *
   * Note: Interrupt service routines should only call _ISR routines.
   * @param item The item to put on the Queue.
   * @param waswoken Flag variable to determine if context switch is needed.
   * @return True if successful
   */
  bool add_ISR(T const& item, portBASE_TYPE& waswoken){
    return xQueueSendToBackFromISR(handle, &item, &waswoken);
  }

  /**
   * @brief Get an item from the Queue.
   * Gets the first item from the Queue
   *
   * Note: Interrupt service routines should only call _ISR routines.
   * @param var Variable to place the item
   * @param waswoken Flag variable to determine if context switch is needed.
   * @return True if an item returned.
   */
  bool pop_ISR(T& var, portBASE_TYPE& waswoken) {
    return xQueueReceiveFromISR(handle, &var, &waswoken);
  }

  /**
   * @brief Look at the first item in the Queue.
   * Gets the first item from the Queue leaving it there.
   *
   * Note: Interrupt service routines should only call _ISR routines.
   * @param var Variable to place the item
   * @param waswoken Flag variable to determine if context switch is needed.
   * @return True if an item returned.
   */
  bool peek_ISR(T& var, portBASE_TYPE& waswoken) {
    return xQueuePeekFromISR(handle, &var, &waswoken);
  }

  /**
   * @brief Is Queue Full.
   * Note: Interrupt service routines should only call _ISR routines.
   * @return True if Queue is Full.
   */
  bool full_ISR() {
    return xQueueIsQueueFullFromISR(handle);
  }

  /**
   * @brief Is Queue Empty.
   * Note: Interrupt service routines should only call _ISR routines.
   * @return True if Queue is Empty.
   */
  bool empty_ISR() {
    return xQueueIsQueueEmptyFromISR(handle);
  }

  /**
   * @brief Get number of message waiting.
   * Note: Interrupt service routines should only call _ISR routines.
   * @return The number of messages waiting.
   */
  unsigned waiting_ISR() {
    return uxQueueMessagesWaitingFromISR(handle);
  }

private:
    QueueHandle_t handle;

#if __cplusplus < 201101L
    Queue(Queue const&);      ///< We are not copyable.
    void operator =(Queue const&);  ///< We are not assignable.
#else
    Queue(Queue const&) = delete;      ///< We are not copyable.
    void operator =(Queue const&) = delete;  ///< We are not assignable.
#endif // __cplusplus
  
};

#endif
