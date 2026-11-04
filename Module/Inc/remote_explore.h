#ifndef MODULE_INC_REMOTE_EXPLORE_H_
#define MODULE_INC_REMOTE_EXPLORE_H_

#include <stdint.h>

namespace RemoteExplore
{
enum StopReason { NONE, USER_STOP, LINK_TIMEOUT, TIME_LIMIT };
void arm();
void request_stop();
bool begin(uint32_t now_ms, uint32_t limit_ms);
void finish();
bool running();
bool should_stop(uint32_t now_ms);
StopReason stop_reason();
}

#endif
