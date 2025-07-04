/*
 * priority_queue.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_PRIORITY_QUEUE_H_
#define INC_PRIORITY_QUEUE_H_


#include <iostream>


template<std::size_t SIZE,typename T>
class Priority_queue{
	private:
		int16_t tail;
		T buff[SIZE];
		uint16_t queue_length()
		{
			return tail + 1;
		}
		bool less_than(T a,T b)
		{
		     return ((a < b) ? true : false);
		}
		/*
		bool less_than(t_MapNode a,t_MapNode b)
		{
		     return ((a.cost < b.cost) ? true : false);
		}
		*/
		void min_heapify(uint16_t parent_pos)
		{
			uint16_t left_ch  = 2*parent_pos + 1;
			uint16_t right_ch = 2*parent_pos + 2;
			uint16_t smallest = parent_pos;

			while(1)
			{

				//if(left_ch <= tail && ((buff[smallest]) > (buff[left_ch])))
				if(left_ch <= tail && less_than(buff[left_ch],buff[smallest]))
					smallest = left_ch;
				//if(right_ch <= tail && ((buff[smallest]) > (buff[right_ch])))
				if(right_ch <= tail && less_than(buff[right_ch],buff[smallest]))
					smallest = right_ch;

				if(smallest != parent_pos)
				{
					swap(&buff[parent_pos],&buff[smallest]);
					left_ch = 2*smallest + 1;
					right_ch = 2*smallest + 2;
					parent_pos = smallest;
				}
				else
				{
					break;
				}
			}
		}
		void swap(T *a,T *b)
		{
			T temp;
			temp = *b;
			*b = *a;
			*a = temp;
		}
	public:
	    Priority_queue()
	    {
	    	tail = -1;
	    }
	    void queue_init()
	    {
	    	tail = -1;
	    }
	    void push(T push_data)
	    {
	    	buff[tail + 1] = push_data;
	    	tail = tail + 1;
	    }
		T heap_pop()
		{
			T pop_data = buff[0];
			buff[0] = buff[tail];
			tail = tail - 1;
			if(is_Empty_queue() == false)
				min_heapify(0);
			return pop_data;
		}
		void heap_push(T push_data)
		{
			buff[tail + 1] = push_data;
			tail = tail + 1;
			if(queue_length() > 1)
			    min_heapify((tail-1)/2);
		}
		void build_heap()
		{
		    if( queue_length() > 1)
		    {
		    	for(int i = (tail-1)/2; i >= 0; i--)
		    	{
		    		min_heapify(i);
		    	}
		    }
		}
		bool is_Empty_queue()
		{
			if(queue_length() == 0)
				return true;
			else
				return false;
		}

};



#endif /* INC_PRIORITY_QUEUE_H_ */
