#ifndef PUPIL_PROGS_UTIL_HPP
#define PUPIL_PROGS_UTIL_HPP

#include <cstdlib>

/* print a byte buffer up to len */
void print_buffer(const unsigned char *buffer, size_t len);

/* print a byte buffer as unsigned integers */
void print_buffer_uint(const unsigned char *buffer, size_t len);

/* find first character in byte buffer that is not numeric */
int find_first_nonnumber(const char *buffer,size_t len);

#endif
