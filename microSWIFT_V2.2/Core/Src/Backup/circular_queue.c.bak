
#include "circular_queue.h"
#include "string.h"

// Helper functions
static inline bool requires_wraparound_enqueue(circular_queue_handle_t circ)
{
    return (circ->tail == circ->buffer_last_addr);
}
static inline bool requires_wraparound_dequeue(circular_queue_handle_t circ)
{
    return (circ->head == circ->buffer_last_addr);
}




void circular_queue_static_init(circular_queue_handle_t circ, uint8_t *input_buffer,
                                                     size_t data_element_size, size_t num_elements)
{
    circ->data_element_size = data_element_size;
    circ->max_num_elements = num_elements;
    circ->buffer = input_buffer;
    cq_reset(circ, true);
    circ->buffer_last_addr = &(circ->buffer[0]) + CIRCULAR_QUEUE_ALLOC_SIZE(data_element_size, num_elements);
}

void cq_reset(circular_queue_handle_t circ, bool clear_queue)
{
    circ->head = &(circ->buffer[0]);
    circ->tail = circ->head;
    if (clear_queue) {
        memset(&(circ->buffer[0]), 0, CIRCULAR_QUEUE_ALLOC_SIZE(circ->data_element_size, circ->max_num_elements));
    }
    circ->num_elements_written = 0;
}

bool cq_enqueue_element(circular_queue_handle_t circ, void* element_in)
{
    bool retval = false;

    if (!cq_is_full(circ)) {
        if (requires_wraparound_enqueue(circ)) {
            memcpy(&(circ->buffer[0]), element_in, circ->data_element_size);
            circ->tail = &(circ->buffer[0]) + circ->data_element_size;
        } else {
            memcpy(circ->tail, element_in, circ->data_element_size);
            circ->tail += circ->data_element_size;
        }
        circ->num_elements_written++;
        retval = true;
    }

    return retval;
}

bool cq_force_enqueue_element(circular_queue_handle_t circ, void* element_in)
{
    bool retval = false;

    if (requires_wraparound_enqueue(circ)) {
        memcpy(&(circ->buffer[0]), element_in, circ->data_element_size);
        circ->tail = &(circ->buffer[0]) + circ->data_element_size;
    } else {
        memcpy(circ->tail, element_in, circ->data_element_size);
        circ->tail += circ->data_element_size;
    }

    if (circ->num_elements_written < circ->max_num_elements) {
        circ->num_elements_written++;
        retval = true;
    }

    return retval;
}

bool cq_dequeue_element(circular_queue_handle_t circ, void* element_out)
{
    bool retval = false;

    if (!cq_is_empty(circ)) {
        if (requires_wraparound_dequeue(circ)) {
            memcpy(element_out, &(circ->buffer[0]), circ->data_element_size);
            circ->head = &(circ->buffer[0]) + circ->data_element_size;
        } else {
            memcpy(element_out, circ->head, circ->data_element_size);
            circ->head += circ->data_element_size;
        }

        circ->num_elements_written--;
        retval = true;
    }

    return retval;
}

bool cq_is_full(circular_queue_handle_t circ)
{
    return (circ->num_elements_written == circ->max_num_elements);
}

bool cq_is_empty(circular_queue_handle_t circ)
{
    return (circ->num_elements_written == 0);
}

size_t cq_get_space_remaining(circular_queue_handle_t circ)
{
    return circ->max_num_elements - circ->num_elements_written;
}

size_t cq_get_space_used(circular_queue_handle_t circ)
{
    return circ->num_elements_written;
}

