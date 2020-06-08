#include "util.hpp"

#include <cstdio>

void print_buffer(const unsigned char *buffer, size_t len){
	for(size_t i = 0; i < len; ++i)
		printf("%c",buffer[i]);
	printf("\n");
}

void print_buffer_uint(const unsigned char *buffer, size_t len){
	for(size_t i = 0; i < len; ++i)
		printf("%u ",buffer[i]);
	printf("\n");
}

int find_first_nonnumber(const char *buffer,size_t len){
	int index = -1;
	for(size_t i = 0; i < len; ++i)
		if(buffer[i] < '0' || buffer[i] > '9')
			return i;
	return index;
}
