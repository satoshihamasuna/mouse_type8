#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "firmware/hy0020/src/batch_policy.h"

static bool ready(struct uart_ble_batch_policy &policy, uint32_t session,
                  uint32_t available, int64_t now, int64_t &wait,
                  uint32_t capacity = 244)
{
    return uart_ble_batch_ready(&policy, session, available, capacity, now, 7, &wait);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    struct uart_ble_batch_policy policy = {};
    int64_t wait = -1;
    if(strcmp(argv[1], "ascii") == 0)
    {
        assert(!ready(policy, 1, 6, 100, wait) && wait == 7);
        assert(!ready(policy, 1, 10, 101, wait) && wait == 6);
        assert(!ready(policy, 1, 20, 106, wait) && wait == 1);
        assert(ready(policy, 1, 20, 107, wait));
        return 0;
    }
    if(strcmp(argv[1], "stream") == 0)
    {
        // Three 252-byte DMA arrivals: pack 3 full notifications and one tail.
        uint32_t available = 0, transmitted = 0;
        for(int64_t now = 0; now <= 12; now += 6)
        {
            available += 252;
            assert(ready(policy, 1, available, now, wait));
            available -= 244;
            transmitted += 244;
            assert(!ready(policy, 1, available, now, wait) && wait == 7);
        }
        assert(available == 24 && transmitted == 732);
        assert(!ready(policy, 1, available, 18, wait) && wait == 1);
        assert(ready(policy, 1, available, 19, wait));
        assert(transmitted + available == 756);
        return 0;
    }
    if(strcmp(argv[1], "reset") == 0)
    {
        assert(!ready(policy, 1, 126, 100, wait) && wait == 7);
        assert(!ready(policy, 1, 0, 103, wait) && wait == 20);
        assert(!ready(policy, 1, 8, 105, wait) && wait == 7);
        assert(!ready(policy, 2, 8, 108, wait) && wait == 7);
        assert(!ready(policy, 2, 100, 114, wait) && wait == 1);
        assert(ready(policy, 2, 100, 115, wait));
        return 0;
    }
    if(strcmp(argv[1], "mtu") == 0)
    {
        assert(!ready(policy, 1, 15, 0, wait, 20) && wait == 7);
        assert(ready(policy, 1, 20, 3, wait, 20));
        assert(!ready(policy, 1, 5, 4, wait) && wait == 7);
        assert(!ready(policy, 1, 40, 10, wait) && wait == 1);
        assert(ready(policy, 1, 40, 11, wait));
        return 0;
    }
    if(strcmp(argv[1], "uptime") == 0)
    {
        const int64_t now = INT64_C(0xffffffff) - 2;
        assert(!ready(policy, 0xffffffffU, 8, now, wait) && wait == 7);
        assert(!ready(policy, 0xffffffffU, 16, now + 6, wait) && wait == 1);
        assert(ready(policy, 0xffffffffU, 16, now + 7, wait));
        assert(!ready(policy, 0, 8, now + 8, wait) && wait == 7);
        return 0;
    }
    assert(!"Unknown case");
}
