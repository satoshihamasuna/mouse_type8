#ifndef SYSTEM_INC_CAMERA_RUN_PERMIT_H_
#define SYSTEM_INC_CAMERA_RUN_PERMIT_H_

#include <stdint.h>

// 撮影用コマンドだけの短時間・一回限りの許可。通常のexeには使用しない。
class CameraRunPermit
{
public:
	static constexpr uint32_t ttl_ms = 15000;
	static bool parse_token(const char *text, uint32_t *value)
	{
		if (text == nullptr || *text == '\0' || value == nullptr) return false;
		uint32_t parsed = 0;
		for (const char *p = text; *p != '\0'; ++p) {
			if (*p < '0' || *p > '9') return false;
			const uint32_t digit = static_cast<uint32_t>(*p - '0');
			if (parsed > (99999999u - digit) / 10u) return false;
			parsed = parsed * 10u + digit;
		}
		if (parsed == 0) return false;
		*value = parsed;
		return true;
	}
	void arm(uint32_t token, uint32_t now_ms) { token_ = token; armed_at_ = now_ms; }
	void clear() { token_ = 0; }
	bool consume(uint32_t token, uint32_t now_ms)
	{
		const bool valid = token_ != 0 && token == token_ && (now_ms - armed_at_) < ttl_ms;
		clear();
		return valid;
	}
private:
	uint32_t token_ = 0;
	uint32_t armed_at_ = 0;
};

#endif
