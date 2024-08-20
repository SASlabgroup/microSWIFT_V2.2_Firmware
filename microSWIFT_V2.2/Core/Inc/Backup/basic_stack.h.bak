#ifndef C_UTILS_BASIC_STACK_H
#define C_UTILS_BASIC_STACK_H

#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"

/*! \brief Macro for finding the buffer size needed for a basic stack
 *  @param data_element_size Size of each element (i.e. sizeof(<type>))
 *  @param num_elements Number of elements the stack is to contain
 */
#define STACK_ALLOC_SIZE(data_element_size, num_elements) \
        (data_element_size * num_elements)

/*! \struct basic_stack_t
 *  \brief A simple stack implementation.
 */
typedef struct basic_stack_t
{
  // Bookkeeping
  size_t data_element_size; /*!< Size of each element in the stack. */
  size_t max_num_elements; /*!< Maximum elements the stack can hold. */
  size_t num_elements_written; /*!< Number of elements written to the stack. */
  // The pointers
  uint8_t *buffer; /*!< Pointer to underlying byte array. */
  uint8_t *top; /*!< End of last element written to stack. */
} basic_stack_t;

/*!\typedef basic_stack_handle
 * \brief Pointer to basic_stack_t struct.
 */
typedef basic_stack_t *basic_stack_handle;

/*!\fn basic_stack_static_init
 * \brief Initialization function.
 *  @param stack Basic stack handle.
 *  @param input_buffer Pointer to input buffer.
 *  @param data_element_size Size of each element to be stored.
 *  @param num_elements Number of elements to store.
 *  @return Void
 */
void basic_stack_static_init ( basic_stack_handle stack, uint8_t *input_buffer,
                               size_t data_element_size, size_t num_elements, bool initialize_full );

/*!\fn bs_reset
 * \brief Reset the stack.
 *  @param stack basic stack handle.
 *  @param clear_stack If true, memset underlying array to clear contents.
 *  @return Void
 */
void bs_reset ( basic_stack_handle stack, bool clear_stack );

/*!\fn bs_push_element
 * \brief Push an element into the stack if there is room, otherwise discard.
 *  @param stack basic stack handle.
 *  @param element_in Pointer to data element to be written.
 *  @return Bool indicating if the element was written -- stack was not full.
 */
bool bs_push_element ( basic_stack_handle stack, void *element_in );

/*!\fn bs_force_push_element
 * \brief Push an element into the stack, overwriting oldest element if stack is full.
 *  @param stack basic stack handle.
 *  @param element_in Pointer to data element to be written.
 *  @return Bool indicating if the stack was not full -- false means data was overwritten.
 */
bool bs_force_push_element ( basic_stack_handle stack, void *element_in );

/*!\fn bs_pull_element
 * \brief Reset the stack.
 *  @param stack basic stack handle.
 *  @param element_out Pointer to data destination.
 *  @return Bool indicating if the stack was not empty -- false indicated stack was empty and no data was returned.
 */
bool bs_pop_element ( basic_stack_handle stack, void *element_out );

/*!\fn bs_is_full
 * \brief Check if the stack is full.
 *  @param stack basic stack handle.
 *  @return Bool indicating if the stack is full or not.
 */
bool bs_is_full ( basic_stack_handle stack );

/*!\fn bs_is_empty
 * \brief Check if the stack is empty.
 *  @param stack basic stack handle.
 *  @return Bool indicating if the stack is empty or not.
 */
bool bs_is_empty ( basic_stack_handle stack );

/*!\fn bs_get_space_remaining
 * \brief Check how many more elements can be stored in the stack.
 *  @param stack basic stack handle.
 *  @return size_t indicating remaining space in number of elements (not in bytes).
 */
size_t bs_get_space_remaining ( basic_stack_handle stack );

/*!\fn bs_get_space_used
 * \brief Check how many more elements are stored in the stack.
 *  @param stack basic stack handle.
 *  @return size_t indicating how many elements are stored (not in bytes).
 */
size_t bs_get_space_used ( basic_stack_handle stack );

#endif // C_UTILS_BASIC_STACK_H
