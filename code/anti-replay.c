

#include "net/mac/csma/anti-replay.h"
#include "net/packetbuf.h"
#include "net/mac/llsec802154.h"

#if LLSEC802154_USES_FRAME_COUNTER

/*---------------------------------------------------------------------------*/

/* This node's current frame counter value */
static uint32_t counter; //CONTADOR DEL FRAME ACTUAL DEL NODO 

/*---------------------------------------------------------------------------*/

void anti_replay_set_counter (void)
{
	frame802154_frame_counter_t reordered_counter;

	++counter;
	reordered_counter.u32 = LLSEC802154_HTONL(counter);

	packetbuf_set_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1, reordered_counter.u16[0]);
	packetbuf_set_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3, reordered_counter.u16[1]);
}


uint32_t anti_replay_get_counter (void)
{
	frame802154_frame_counter_t disordered_counter;

	disordered_counter.u16[0] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1);
	disordered_counter.u16[1] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3);

	return LLSEC802154_HTONL(disordered_counter.u32); 
}


void anti_replay_init_info (struct anti_replay_info *info)
{
	info->last_broadcast_counter = info->last_unicast_counter = anti_replay_get_counter();
}


int anti_replay_was_replayed (struct anti_replay_info *info)
{
	uint32_t received_counter;
	received_counter = anti_replay_get_counter();

	if (packetbuf_holds_broadcast()) //BROADCAST
	{
		if (received_counter <= info->last_broadcast_counter) 
		{
			return 1;
		} 
		else 
		{
			info->last_broadcast_counter = received_counter;
			return 0;
		}
	} 
	else //UNICAST
	{
		if (received_counter <= info->last_unicast_counter) 
		{
			return 1;
		} 
		else 
		{
			info->last_unicast_counter = received_counter;
			return 0;
		}
	}
}


#endif /* LLSEC802154_USES_FRAME_COUNTER */


