#ifndef MODULE_INC_LOG_BUFFER_H_
#define MODULE_INC_LOG_BUFFER_H_

#include "Component/Inc/half_float.h"
#include <string.h>

// Single producer. Callers exclude the producer while resetting or reading.
template<uint16_t Capacity, uint16_t Fields>
class LogBuffer
{
	static_assert(Capacity > 0 && Fields > 0, "Log buffer dimensions must be positive");
public:
	void reset() { next = count = 0; }
	uint16_t size() const { return count; }
	void push(const half_float *sample)
	{
		memcpy(data[next], sample, sizeof(data[next]));
		next = (next + 1) % Capacity;
		if(count < Capacity) ++count;
	}
	bool copy(uint16_t index, half_float *sample) const
	{
		if(index >= count) return false;
		const uint16_t oldest = count == Capacity ? next : 0;
		memcpy(sample, data[(oldest + index) % Capacity], sizeof(data[0]));
		return true;
	}
private:
	half_float data[Capacity][Fields];
	uint16_t next = 0;
	uint16_t count = 0;
};

#endif
