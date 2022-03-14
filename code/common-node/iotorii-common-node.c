
 
//INICIA EL PROTOCOLO IOTORII PARA UN NODO COMÚN

#include "contiki.h"
#include "net/netstack.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/

PROCESS (start_iotorii_common_node, "Process to Start common node");
AUTOSTART_PROCESSES (&start_iotorii_common_node);


PROCESS_THREAD (start_iotorii_common_node, ev, data)
{
  PROCESS_BEGIN();
  
  NETSTACK_RADIO.on();
  //NETSTACK_MAC.init(); //MAC is initiated twice
  //netstack_init(); //MAC is initiated twice

  PROCESS_END();
}
