/*
 *  CORE Message Queue Handler
 *
 *  DESCRIPTION:
 *
 *  This package is the implementation of the CORE Message Queue Handler.
 *  This core object provides task synchronization and communication functions
 *  via messages passed to queue objects.
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include <rtems/system.h>
#include <rtems/score/chain.h>
#include <rtems/score/isr.h>
#include <rtems/score/object.h>
#include <rtems/score/coremsg.h>
#include <rtems/score/states.h>
#include <rtems/score/thread.h>
#include <rtems/score/wkspace.h>
#if defined(RTEMS_MULTIPROCESSING)
#include <rtems/score/mpci.h>
#endif

/*PAGE
 *
 *  _CORE_message_queue_Initialize
 *
 *  This routine initializes a newly created message queue based on the
 *  specified data.
 *
 *  Input parameters:
 *    the_message_queue            - the message queue to initialize
 *    the_class                    - the API specific object class
 *    the_message_queue_attributes - the message queue's attributes
 *    maximum_pending_messages     - maximum message and reserved buffer count
 *    maximum_message_size         - maximum size of each message
 *
 *  Output parameters:
 *    TRUE   - if the message queue is initialized
 *    FALSE  - if the message queue is NOT initialized
 */

boolean _CORE_message_queue_Initialize(
  CORE_message_queue_Control    *the_message_queue,
  CORE_message_queue_Attributes *the_message_queue_attributes,
  uint32_t                     maximum_pending_messages,
  uint32_t                     maximum_message_size
)
{
  uint32_t message_buffering_required;
  uint32_t allocated_message_size;

  the_message_queue->maximum_pending_messages   = maximum_pending_messages;
  the_message_queue->number_of_pending_messages = 0;
  the_message_queue->maximum_message_size       = maximum_message_size;
  _CORE_message_queue_Set_notify( the_message_queue, NULL, NULL );
 
  /*
   *  Round size up to multiple of a pointer for chain init and 
   *  check for overflow on adding overhead to each message.
   */
 
  allocated_message_size = maximum_message_size;
  if (allocated_message_size & (sizeof(uint32_t) - 1)) {
      allocated_message_size += sizeof(uint32_t);
      allocated_message_size &= ~(sizeof(uint32_t) - 1);
  }
   
  if (allocated_message_size < maximum_message_size)
    return FALSE;

  /*
   *  Calculate how much total memory is required for message buffering and
   *  check for overflow on the multiplication. 
   */
  message_buffering_required = maximum_pending_messages *
       (allocated_message_size + sizeof(CORE_message_queue_Buffer_control));
 
  if (message_buffering_required < allocated_message_size)
    return FALSE;

  /*
   *  Attempt to allocate the message memory
   */
  the_message_queue->message_buffers = (CORE_message_queue_Buffer *) 
     _Workspace_Allocate( message_buffering_required );
 
  if (the_message_queue->message_buffers == 0)
    return FALSE;
 
  /*
   *  Initialize the pool of inactive messages, pending messages,
   *  and set of waiting threads.
   */
  _Chain_Initialize (
    &the_message_queue->Inactive_messages,
    the_message_queue->message_buffers,
    maximum_pending_messages,
    allocated_message_size + sizeof( CORE_message_queue_Buffer_control )
  );
 
  _Chain_Initialize_empty( &the_message_queue->Pending_messages );
 
  _Thread_queue_Initialize(
    &the_message_queue->Wait_queue,
    _CORE_message_queue_Is_priority( the_message_queue_attributes ) ?
       THREAD_QUEUE_DISCIPLINE_PRIORITY : THREAD_QUEUE_DISCIPLINE_FIFO,
    STATES_WAITING_FOR_MESSAGE,
    CORE_MESSAGE_QUEUE_STATUS_TIMEOUT
  );

  return TRUE;
}
