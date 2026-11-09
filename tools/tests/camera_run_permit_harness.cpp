#include "System/Inc/camera_run_permit.h"
#include <assert.h>

int main()
{
	CameraRunPermit permit;
	assert(!permit.consume(123, 0));
	permit.arm(123, 100);
	assert(permit.consume(123, 100));
	assert(!permit.consume(123, 101));
	permit.arm(123, 100);
	assert(!permit.consume(456, 101));
	assert(!permit.consume(123, 102));
	permit.arm(123, 100);
	assert(permit.consume(123, 15099));
	permit.arm(123, 100);
	assert(!permit.consume(123, 15100));
	permit.arm(123, 0xfffffff0u);
	assert(permit.consume(123, 0x10u));
	permit.arm(123, 0xfffffff0u);
	assert(!permit.consume(123, 0x4000u));
	permit.arm(123, 100);
	permit.arm(456, 101);
	assert(!permit.consume(123, 102));
	permit.arm(123, 100);
	permit.clear();
	assert(!permit.consume(123, 101));
	uint32_t token = 0;
	assert(CameraRunPermit::parse_token("123", &token) && token == 123);
	assert(CameraRunPermit::parse_token("99999999", &token) && token == 99999999);
	const char *invalid[] = {"", "0", "-1", "1x", "100000000", "4294967296"};
	for (const char *text : invalid) {
		assert(!CameraRunPermit::parse_token(text, &token));
	}
	assert(!CameraRunPermit::parse_token(nullptr, &token));
	return 0;
}
