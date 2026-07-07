/*
 * queue.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <iostream>
#include <cstdint>

template<std::size_t SIZE,typename T>
class ring_queue{
	private:
		T buff[SIZE];
		int16_t tail;
		int16_t head;
		int16_t length;
		const uint16_t cap = SIZE;
	public:
		ring_queue()
		{
			tail = -1;
			head = 0;
			length = 0;
		}
		T pop()
		{
			T pop_data = buff[head];
			head = (head + 1)%cap;
			length = length - 1;
			return pop_data;
		}
		void push(T push_data)
		{
			buff[(tail + 1)%cap] = push_data;
			tail = (tail + 1)%cap;
			length = length + 1;
		}
		int queue_length()
		{
			return length;
		}
		void queue_reset()
		{
			tail = -1;
			head = (tail + 1)%cap;
			length = 0;
		}
};



#endif /* INC_QUEUE_H_ */
