//
// Created by Phillip Bush on 3/18/24.
//

#include "basic_stack.h"
#include "string.h"

void basic_stack_static_init ( basic_stack_handle stack, uint8_t *input_stack,
                               size_t data_element_size, size_t num_elements, bool initialize_full )
{
  stack->data_element_size = data_element_size;
  stack->max_num_elements = num_elements;
  stack->buffer = input_stack;
  bs_reset (stack, true);

  if ( initialize_full )
  {
    stack->num_elements_written = stack->max_num_elements;
  }
}

void bs_reset ( basic_stack_handle stack, bool clear_stack )
{
  stack->top = &(stack->buffer[0]);

  if ( clear_stack )
  {
    memset (&(stack->buffer[0]), 0,
            STACK_ALLOC_SIZE(stack->data_element_size, stack->max_num_elements));
  }

  stack->num_elements_written = 0;
}

bool bs_push_element ( basic_stack_handle stack, void *element_in )
{
  bool retval = false;

  if ( !bs_is_full (stack) )
  {
    memcpy (stack->top, element_in, stack->data_element_size);
    stack->top += stack->data_element_size;

    stack->num_elements_written++;
    retval = true;
  }

  return retval;
}

bool bs_force_push_element ( basic_stack_handle stack, void *element_in )
{
  bool retval = false;

  if ( bs_is_full (stack) )
  {
    memcpy ((stack->top - stack->data_element_size), element_in, stack->data_element_size);
  }
  else
  {
    memcpy (stack->top, element_in, stack->data_element_size);
    stack->top += stack->data_element_size;
    stack->num_elements_written++;
    retval = true;
  }

  return retval;
}

bool bs_pop_element ( basic_stack_handle stack, void *element_out )
{
  bool retval = false;

  if ( !bs_is_empty (stack) )
  {
    stack->top -= stack->data_element_size;
    memcpy (element_out, stack->top, stack->data_element_size);

    stack->num_elements_written--;
    retval = true;
  }

  return retval;
}

bool bs_is_full ( basic_stack_handle stack )
{
  return (stack->num_elements_written == stack->max_num_elements);
}

bool bs_is_empty ( basic_stack_handle stack )
{
  return (stack->num_elements_written == 0);
}

size_t bs_get_space_remaining ( basic_stack_handle stack )
{
  return stack->max_num_elements - stack->num_elements_written;
}

size_t bs_get_space_used ( basic_stack_handle stack )
{
  return stack->num_elements_written;
}
