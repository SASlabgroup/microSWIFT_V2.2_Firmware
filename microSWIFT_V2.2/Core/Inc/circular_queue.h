
#ifndef C_UTILS_CIRCULAR_QUEUE_H
#define C_UTILS_CIRCULAR_QUEUE_H

#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"


/*! \brief Macro for finding the buffer size needed for a circular queue
 *  @param data_element_size Size of each element (i.e. sizeof(<type>))
 *  @param num_elements Number of elements the circular queue is to contain
 */
#define CIRCULAR_QUEUE_ALLOC_SIZE(data_element_size, num_elements) \
        (data_element_size * num_elements)

/*! \struct circular_queue_t
 *  \brief A simple circular queue implementation.
 */
typedef struct circular_queue_t {
    // Bookkeeping
    size_t data_element_size; /*!< Size of each element in the queue. */
    size_t max_num_elements; /*!< Maximum elements the queue can hold. */
    size_t num_elements_written; /*!< Number of elements written to the queue. */
    // The pointers
    uint8_t *buffer; /*!< Pointer to underlying byte array. */
    uint8_t *head; /*!< Head of queue, where oldest element is written. */
    uint8_t *tail; /*!< End of last element written to queue. */
    uint8_t *buffer_last_addr; /*!< Last address in the underlying array. */
} circular_queue_t;


/*!\typedef circular_queue_handle_t
 * \brief Pointer to circular_queue_t struct.
 */
typedef circular_queue_t* circular_queue_handle_t;


/*!\fn circular_queue_static_init
 * \brief Initialization function.
 *  @param circ Circular queue handle.
 *  @param input_buffer Pointer to input buffer.
 *  @param data_element_size Size of each element to be stored.
 *  @param num_elements Number of elements to store.
 *  @return Void
 */
void circular_queue_static_init(circular_queue_handle_t circ, uint8_t *input_queue,
                                                     size_t data_element_size, size_t num_elements);

/*!\fn cq_reset
 * \brief Reset the circular queue.
 *  @param circ Circular queue handle.
 *  @param clear_queue If true, memset underlying array to clear contents.
 *  @return Void
 */
void cq_reset(circular_queue_handle_t circ, bool clear_queue);


/*!\fn cq_push_element
 * \brief Push an element into the queue if there is room, otherwise discard.
 *  @param circ Circular queue handle.
 *  @param element_in Pointer to data element to be written.
 *  @return Bool indicating if the element was written -- queue was not full.
 */
bool cq_enqueue_element(circular_queue_handle_t circ, void* element_in);


/*!\fn cq_force_push_element
 * \brief Push an element into the queue, overwriting oldest element if queue is full.
 *  @param circ Circular queue handle.
 *  @param element_in Pointer to data element to be written.
 *  @return Bool indicating if the queue was not full -- false means data was overwritten.
 */
bool cq_force_enqueue_element(circular_queue_handle_t circ, void* element_in);


/*!\fn cq_pull_element
 * \brief Reset the circular queue.
 *  @param circ Circular queue handle.
 *  @param element_out Pointer to data destination.
 *  @return Bool indicating if the queue was not empty -- false indicated queue was empty and no data was returned.
 */
bool cq_dequeue_element(circular_queue_handle_t circ, void* element_out);


/*!\fn cq_is_full
 * \brief Check if the queue is full.
 *  @param circ Circular queue handle.
 *  @return Bool indicating if the queue is full or not.
 */
bool cq_is_full(circular_queue_handle_t circ);


/*!\fn cq_is_empty
 * \brief Check if the queue is empty.
 *  @param circ Circular queue handle.
 *  @return Bool indicating if the queue is empty or not.
 */
bool cq_is_empty(circular_queue_handle_t circ);


/*!\fn cq_get_space_remaining
 * \brief Check how many more elements can be stored in the queue.
 *  @param circ Circular queue handle.
 *  @return size_t indicating remaining space in number of elements (not in bytes).
 */
size_t cq_get_space_remaining(circular_queue_handle_t circ);


/*!\fn cq_get_space_used
 * \brief Check how many more elements are stored in the queue.
 *  @param circ Circular queue handle.
 *  @return size_t indicating how many elements are stored (not in bytes).
 */
size_t cq_get_space_used(circular_queue_handle_t circ);


#endif // C_UTILS_CIRCULAR_QUEUE_H