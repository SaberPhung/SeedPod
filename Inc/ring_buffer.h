#include <string.h>

struct ring_buffer{
    unsigned char* buffer;
    unsigned char* tail;
    unsigned char* head;
    int size;
};

void init_buffer(struct ring_buffer* rb, unsigned char* buffer, int size);
void reset_buffer(struct ring_buffer* rb);
int get_buffer_size(struct ring_buffer* rb);
int append_char_to_buffer(struct ring_buffer* rb, unsigned char c);
unsigned char pop_char_from_buffer(struct ring_buffer* rb);
int append_string_to_buffer(struct ring_buffer* rb, unsigned char* s);
void pop_string_from_buffer(struct ring_buffer* rb, unsigned char* destination, int size);
