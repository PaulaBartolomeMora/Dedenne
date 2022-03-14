

//INICIA EL PROTOCOLO IOTORII PARA UN NODO ROOT

#include "contiki.h"
#include "net/netstack.h"
#include <stdio.h>

/*---------------------------------------------------------------------------*/

PROCESS (start_iotorii_root, "Process to Start IoTorii Root");
AUTOSTART_PROCESSES (&start_iotorii_root);


PROCESS_THREAD (start_iotorii_root, ev, data)
{
  PROCESS_BEGIN();

  NETSTACK_RADIO.on();
  //NETSTACK_MAC.init(); //MAC is initiated twice
  //netstack_init(); //MAC is initiated twice

  PROCESS_END();
}
