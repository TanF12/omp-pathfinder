#pragma once
#include <atomic>
#include <thread>

#if defined(__i386__) || defined(__x86_64__)
    #include <immintrin.h>
    #define SPIN_HINT() _mm_pause()
#elif defined(__aarch64__)
    #define SPIN_HINT() __asm__ __volatile__("yield")
#else
    #define SPIN_HINT()
#endif

#if defined(__GNUC__) || defined(__clang__)
    #define OMP_LIKELY(x) __builtin_expect(!!(x), 1)
#else
    #define OMP_LIKELY(x) (x)
#endif

static constexpr size_t CACHE_LINE = 64;

class SpinLock {
    std::atomic_flag locked = ATOMIC_FLAG_INIT;
public:
    inline void lock() {
        int backoff = 1;
        while (locked.test_and_set(std::memory_order_acquire)) {
            if (OMP_LIKELY(backoff < 64)) {
                for (int i = 0; i < backoff; ++i) {
                    SPIN_HINT(); 
                }
                backoff *= 2;
            } else {
                std::this_thread::yield();
            }
        }
    }
    inline void unlock() {
        locked.clear(std::memory_order_release);
    }
};//SpinLock