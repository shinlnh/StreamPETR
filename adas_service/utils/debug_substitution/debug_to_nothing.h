#ifndef DEBUG_TO_NOTHING_H
#define DEBUG_TO_NOTHING_H

// Check if DEBUG is already defined before redefining it
#if LOG_LEVEL >= LOG_LEVEL_DEBUG

#undef DEBUG

#define DEBUG(...) do { /* No-op */ } while (0)

#endif // DEBUG

#endif // DEBUG_TO_NOTHING_H
