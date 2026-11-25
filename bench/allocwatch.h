#ifndef ALLOCWATCH_H
#define ALLOCWATCH_H

/* Live-heap accounting for a single-threaded process.
 * Counts net outstanding allocations by interposing on the public malloc
 * family. Sizes are glibc usable sizes, so they are directly comparable to
 * heap growth seen in RSS. */
void alloc_reset_peak(void);
unsigned long long alloc_live_bytes(void);
unsigned long long alloc_live_blocks(void);
unsigned long long alloc_peak_bytes(void);
unsigned long long alloc_total_allocs(void);
unsigned long long alloc_total_frees(void);

#endif
