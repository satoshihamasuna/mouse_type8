#include "Module/Inc/remote_explore.h"
#include "Module/Inc/log_stream.h"

namespace
{
volatile bool active;
volatile bool cancelled;
volatile bool armed;
volatile RemoteExplore::StopReason reason;
uint32_t started;
uint32_t duration;
}

void RemoteExplore::arm()
{
	if(!active) { cancelled = false; reason = NONE; armed = true; }
}

void RemoteExplore::request_stop() { cancelled = true; }

bool RemoteExplore::begin(uint32_t now_ms, uint32_t limit_ms)
{
	if(active || !armed || cancelled || limit_ms == 0 || !LogStream::active(now_ms)) return false;
	armed = false;
	started = now_ms;
	duration = limit_ms;
	reason = NONE;
	active = true;
	return true;
}

void RemoteExplore::finish() { active = false; }
bool RemoteExplore::running() { return active; }
RemoteExplore::StopReason RemoteExplore::stop_reason() { return reason; }

bool RemoteExplore::should_stop(uint32_t now_ms)
{
	if(!active) return false;
	if(reason != NONE) return true;
	if(cancelled) reason = USER_STOP;
	else if(!LogStream::active(now_ms)) reason = LINK_TIMEOUT;
	else if(now_ms - started >= duration) reason = TIME_LIMIT;
	return reason != NONE;
}
