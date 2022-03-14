

#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "net/mac/llsec802154.h"
#include <string.h>

#if LLSEC802154_USES_AUX_HEADER && LLSEC802154_USES_FRAME_COUNTER

/*---------------------------------------------------------------------------*/

static const uint8_t* get_extended_address (const linkaddr_t* addr)
#if LINKADDR_SIZE == 2
{
	/* workaround for short addresses: derive EUI64 as in RFC 6282 */
	static linkaddr_extended_t template = { { 0x00 , 0x00 , 0x00 ,
											0xFF , 0xFE , 0x00 , 0x00 , 0x00 } };
	template.u16[3] = LLSEC802154_HTONS(addr->u16);

	return template.u8;
}
#else /* LINKADDR_SIZE == 2 */
{
	return addr->u8;
}
#endif /* LINKADDR_SIZE == 2 */


void ccm_star_packetbuf_set_nonce (uint8_t* nonce, int forward)
{
	const linkaddr_t *source_addr;

	source_addr = forward ? &linkaddr_node_addr : packetbuf_addr(PACKETBUF_ADDR_SENDER);
	
	memcpy(nonce, get_extended_address(source_addr), 8);
	
	nonce[8] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3) >> 8;
	nonce[9] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3) & 0xff;
	nonce[10] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1) >> 8;
	nonce[11] = packetbuf_attr(PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1) & 0xff;
	nonce[12] = packetbuf_attr(PACKETBUF_ATTR_SECURITY_LEVEL);
}


#endif /* LLSEC802154_USES_AUX_HEADER && LLSEC802154_USES_FRAME_COUNTER */
