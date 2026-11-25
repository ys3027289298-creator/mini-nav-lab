/* allocwatch.c: interpose on malloc family, count live heap.
 * Single-threaded by design (matches the modules under test). */
#include "allocwatch.h"

#include <stddef.h>
#include <stdint.h>

extern void* __libc_malloc(size_t);
extern void* __libc_calloc(size_t, size_t);
extern void* __libc_realloc(void*, size_t);
extern void* __libc_memalign(size_t, size_t);
extern void  __libc_free(void*);

/* glibc: malloc_usable_size gives the block size we actually hold. */
extern size_t malloc_usable_size(void*);

static unsigned long long live_bytes;
static unsigned long long live_blocks;
static unsigned long long peak_bytes;
static unsigned long long total_allocs;
static unsigned long long total_frees;

static void note_alloc(void* p) {
    if (!p) return;
    size_t sz = malloc_usable_size(p);
    live_bytes += sz;
    live_blocks += 1;
    total_allocs += 1;
    if (live_bytes > peak_bytes) peak_bytes = live_bytes;
}

static void note_free(void* p) {
    if (!p) return;
    size_t sz = malloc_usable_size(p);
    live_bytes -= sz;
    live_blocks -= 1;
    total_frees += 1;
}

void alloc_reset_peak(void) { peak_bytes = live_bytes; }
unsigned long long alloc_live_bytes(void) { return live_bytes; }
unsigned long long alloc_live_blocks(void) { return live_blocks; }
unsigned long long alloc_peak_bytes(void) { return peak_bytes; }
unsigned long long alloc_total_allocs(void) { return total_allocs; }
unsigned long long alloc_total_frees(void) { return total_frees; }

void* malloc(size_t n) { void* p = __libc_malloc(n); note_alloc(p); return p; }
void* calloc(size_t n, size_t m) { void* p = __libc_calloc(n, m); note_alloc(p); return p; }
void* realloc(void* old, size_t n) {
    void* p = __libc_realloc(old, n);
    if (old) note_free(old);
    note_alloc(p);
    return p;
}
void* memalign(size_t a, size_t n) { void* p = __libc_memalign(a, n); note_alloc(p); return p; }
void* aligned_alloc(size_t a, size_t n) { return memalign(a, n); }
void* valloc(size_t n) { return memalign(4096, n); }
void free(void* p) { if (p) note_free(p); __libc_free(p); }
