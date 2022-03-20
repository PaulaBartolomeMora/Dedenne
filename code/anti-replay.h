

#ifndef ANTI_REPLAY_H
#define ANTI_REPLAY_H

#include "contiki.h"

/*---------------------------------------------------------------------------*/

struct anti_replay_info 
{
	uint32_t last_broadcast_counter;
	uint32_t last_unicast_counter;
};

/*---------------------------------------------------------------------------*/

//SET THE FRAME COUNTER PACKETBUF ATRIBUTES  
void anti_replay_set_counter (void);

//GETS THE FRAME COUNTER FROM PACKETBUF
uint32_t anti_replay_get_counter (void);

//INICIALIZA LA INFORMACIÓN ANTI-REPLAY SOBRE EL EMISOR
void anti_replay_init_info (struct anti_replay_info *info);

//SE COMPRUEBA SI LA CABECERA RECIBIDA HA SIDO REPLAYED
//SE DEVUELVE 0 SI NO HA SIDO REPLAYED
int anti_replay_was_replayed (struct anti_replay_info *info);


#endif /* ANTI_REPLAY_H */

