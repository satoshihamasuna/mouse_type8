/*
 * priority_queue.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_PRIORITY_QUEUE_H_
#define INC_PRIORITY_QUEUE_H_


#include <iostream>

template<typename T>
struct Less {
    bool operator()(const T& a, const T& b) const {
        return a < b;     // min-heap
    }
};

template<typename T>
struct Greater {
    bool operator()(const T& a, const T& b) const {
        return a > b;     // max-heap
    }
};

template<std::size_t SIZE, typename T, typename Compare = Less<T>>
class Priority_queue{
private:
    int16_t tail;
    T buff[SIZE];
    Compare comp;

    uint16_t queue_length() const {
        return tail + 1;
    }

    void swap(T *a, T *b) {
        T temp = *b;
        *b = *a;
        *a = temp;
    }

    void heapify(uint16_t parent_pos) {
        while (1) {
            uint16_t left  = 2 * parent_pos + 1;
            uint16_t right = 2 * parent_pos + 2;
            uint16_t best = parent_pos;

            if (left <= tail && comp(buff[left], buff[best]))
                best = left;

            if (right <= tail && comp(buff[right], buff[best]))
                best = right;

            if (best != parent_pos) {
                swap(&buff[parent_pos], &buff[best]);
                parent_pos = best;
            } else {
                break;
            }
        }
    }

public:
    Priority_queue() : tail(-1) {}

    void queue_init() {
        tail = -1;
    }

    bool is_Empty_queue() const {
        return tail < 0;
    }

    uint16_t size() const {
        return queue_length();
    }

    // 通常push (上方向ヒープ化)
    void heap_push(const T& push_data) {
        if (tail + 1 >= SIZE) return;  // overflow guard（必要ならassert）

        buff[++tail] = push_data;
        uint16_t i = tail;

        while (i > 0) {
            uint16_t parent = (i - 1) / 2;
            if (comp(buff[i], buff[parent])) {
                swap(&buff[i], &buff[parent]);
                i = parent;
            } else {
                break;
            }
        }
    }

    // root取り出し
    T heap_pop() {
        T pop_data = buff[0];
        buff[0] = buff[tail--];
        if (!is_Empty_queue())
            heapify(0);
        return pop_data;
    }

    // ============================
    // build heap (O(N))
    // ============================
    void build_heap() {
        if (queue_length() <= 1) return;

        for (int i = (tail - 1) / 2; i >= 0; i--) {
            heapify(i);
        }
    }

    // 直接バッファに詰めた後でまとめてヒープ化したい場合用
    void push_raw(const T& data) {
        if (tail + 1 >= SIZE) return;
        buff[++tail] = data;
    }

    // top参照
    const T& top() const {
        return buff[0];
    }
};
/*
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
		void min_heapify(uint16_t parent_pos)
		{
			uint16_t left_ch  = 2*parent_pos + 1;
			uint16_t right_ch = 2*parent_pos + 2;
			uint16_t smallest = parent_pos;

			while(1)
			{

				if(left_ch <= tail && less_than(buff[left_ch],buff[smallest]))
					smallest = left_ch;
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
*/


#endif /* INC_PRIORITY_QUEUE_H_ */
