#include <ring_buffer.h>

void init_buffer(struct ring_buffer* rb, unsigned char* buffer, int size){
    rb->buffer = buffer;
    rb->tail = buffer;
    rb->head = buffer;
    rb->size = size;
}

void reset_buffer(struct ring_buffer* rb){
    rb->tail = rb->buffer;
    rb->head = rb->buffer;
}

int get_buffer_size(struct ring_buffer* rb){
    unsigned char* index = rb->tail;
    int counter = 0;

    while(1){
        if(index == rb->buffer + sizeof(unsigned char) * rb->size){ // If index points to end of reserved space, set index to beginning of reserved space
            index = rb->buffer;
        } else if(index == rb->head){ // If index points to the end of the buffer (the location where the head will store new values)
            return counter;
		} else { 
            counter++;
            index += sizeof(unsigned char);
        }
    }
}

int append_char_to_buffer(struct ring_buffer* rb, unsigned char c){
    if(rb->head + sizeof(unsigned char) == rb->tail || 
      (rb->head == rb->buffer + sizeof(unsigned char) * rb->size && rb->tail == rb->buffer)){ // If buffer is full
		return -1;
	} else if(rb->head >= rb->buffer + sizeof(unsigned char) * rb->size){ // If head points to end of, or after, reserved space
		*rb->buffer = c;
		rb->head = rb->buffer + sizeof(unsigned char);
	} else {
		*rb->head = c;
		rb->head += sizeof(unsigned char);
	}
    return 1;
}

unsigned char pop_char_from_buffer(struct ring_buffer* rb){
    if(rb->head == rb->tail){ // If buffer is empty
        return -1;
    } else if(rb->tail >= rb->buffer + sizeof(unsigned char) * rb->size){ 
        // If the tail is located outside (after) the reserved storage, loop to the start of the storage
        rb->tail = rb->buffer + sizeof(unsigned char);
        return *rb->buffer;
    }
    rb->tail += sizeof(unsigned char);
    return *(rb->tail - sizeof(unsigned char));
}

int append_string_to_buffer(struct ring_buffer* rb, unsigned char* s){
    int size = 0;

    if(rb->head > rb->tail){
        size = (((rb->buffer + rb->size * sizeof(unsigned char)) - rb->head) + (rb->tail - rb->buffer))/sizeof(unsigned char);
    } else {
        size = (rb->tail - rb->head)/sizeof(unsigned char);
    }

    if(size < strlen((char*)s)){ // If size is not enough
        return -1;
    } else {
        for(int i = 0; i < strlen((char*)s); i++){ // Add the given array char by char into the buffer
            append_char_to_buffer(rb, *(s + sizeof(char) * i));
        }
        return strlen((char*)s);
    }
}

void pop_string_from_buffer(struct ring_buffer* rb, unsigned char* destination, int size){
    for(int i = 0; i < size; i++){
        unsigned char temp_char = pop_char_from_buffer(rb);
        destination[i] = temp_char;
    }
}
