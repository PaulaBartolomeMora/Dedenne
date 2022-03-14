

//EXTRA BEGIN
// #include "net/mac/csma/csma.h"
// #include "net/mac/csma/csma-security.h"
  #include "iotorii-csma.h"
  #include "csma-security.h"
//EXTRA END
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "dev/watchdog.h"
#include "sys/ctimer.h"
#include "sys/clock.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/assert.h"

/* Log configuration */
#include "sys/log.h"

/*---------------------------------------------------------------------------*/

#define LOG_MODULE "CSMA"
#define LOG_LEVEL LOG_LEVEL_MAC

/* Constants of the IEEE 802.15.4 standard */

/* macMinBE: Initial backoff exponent. Range 0--CSMA_MAX_BE */
#ifdef CSMA_CONF_MIN_BE
#define CSMA_MIN_BE CSMA_CONF_MIN_BE
#else
#define CSMA_MIN_BE 3
#endif

/* macMaxBE: Maximum backoff exponent. Range 3--8 */
#ifdef CSMA_CONF_MAX_BE
#define CSMA_MAX_BE CSMA_CONF_MAX_BE
#else
#define CSMA_MAX_BE 5
#endif

/* macMaxCSMABackoffs: Maximum number of backoffs in case of channel busy/collision. Range 0--5 */
#ifdef CSMA_CONF_MAX_BACKOFF
#define CSMA_MAX_BACKOFF CSMA_CONF_MAX_BACKOFF
#else
#define CSMA_MAX_BACKOFF 5
#endif

/* macMaxFrameRetries: Maximum number of re-transmissions attampts. Range 0--7 */
#ifdef CSMA_CONF_MAX_FRAME_RETRIES
#define CSMA_MAX_FRAME_RETRIES CSMA_CONF_MAX_FRAME_RETRIES
#else
#define CSMA_MAX_FRAME_RETRIES 7
#endif

/*---------------------------------------------------------------------------*/

struct qbuf_metadata //PACKET METADATA
{
	mac_callback_t sent;
	void *cptr;
	uint8_t max_transmissions;
};


struct neighbor_queue //CARACTERÍSTICAS DE COLA DE PAQUETES DE VECINO
{
	struct neighbor_queue *next;
	linkaddr_t addr;
	struct ctimer transmit_timer;
	uint8_t transmissions;
	uint8_t collisions;
	LIST_STRUCT(packet_queue);
};


//MÁXIMO NÚMERO DE COLAS DE VECINOS COEXISTENTES
#ifdef CSMA_CONF_MAX_NEIGHBOR_QUEUES
#define CSMA_MAX_NEIGHBOR_QUEUES CSMA_CONF_MAX_NEIGHBOR_QUEUES
#else
#define CSMA_MAX_NEIGHBOR_QUEUES 2
#endif /* CSMA_CONF_MAX_NEIGHBOR_QUEUES */


//MÁXIMO NÚMERO DE PAQUETES PENDIENTES POR VECINO
#ifdef CSMA_CONF_MAX_PACKET_PER_NEIGHBOR
#define CSMA_MAX_PACKET_PER_NEIGHBOR CSMA_CONF_MAX_PACKET_PER_NEIGHBOR
#else
#define CSMA_MAX_PACKET_PER_NEIGHBOR MAX_QUEUED_PACKETS
#endif /* CSMA_CONF_MAX_PACKET_PER_NEIGHBOR */

#define MAX_QUEUED_PACKETS QUEUEBUF_NUM


struct packet_queue //COLA DE PAQUETES DEL VECINO
{
	struct packet_queue *next;
	struct queuebuf *buf;
	void *ptr;
};


MEMB(neighbor_memb, struct neighbor_queue, CSMA_MAX_NEIGHBOR_QUEUES);
MEMB(packet_memb, struct packet_queue, MAX_QUEUED_PACKETS);
MEMB(metadata_memb, struct qbuf_metadata, MAX_QUEUED_PACKETS);
LIST(neighbor_list);

/*---------------------------------------------------------------------------*/

static void packet_sent	(struct neighbor_queue *n, struct packet_queue *q, int status, int num_transmissions);	
static void transmit_from_queue (void *ptr);

/*---------------------------------------------------------------------------*/

static struct neighbor_queue* neighbor_queue_from_addr (const linkaddr_t *addr) //SE AÑADE LA COLA DE UN VECINO
{
	struct neighbor_queue *n = list_head(neighbor_list);
	
	while (n != NULL) 
	{
		if (linkaddr_cmp(&n->addr, addr)) 
			return n;
		
		n = list_item_next(n);
	}
	return NULL;
}


static clock_time_t backoff_period (void)
{
	#if CONTIKI_TARGET_COOJA
		/* Increase normal value by 20 to compensate for the coarse-grained
		radio medium with Cooja motes */
		return MAX(20 * CLOCK_SECOND / 3125, 1);
	#else /* CONTIKI_TARGET_COOJA */
		/* Use the default in IEEE 802.15.4: aUnitBackoffPeriod which is
		* 20 symbols i.e. 320 usec. That is, 1/3125 second. */
		return MAX(CLOCK_SECOND / 3125, 1);
	#endif /* CONTIKI_TARGET_COOJA */
}


static int send_one_packet (struct neighbor_queue *n, struct packet_queue *q) //SE EMPAQUETA Y SE ENVÍA
{
	int ret;
	int last_sent_ok = 0;

	packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
	packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);

	#if LLSEC802154_ENABLED
	#if LLSEC802154_USES_EXPLICIT_KEYS
	/* This should possibly be taken from upper layers in the future */
	packetbuf_set_attr(PACKETBUF_ATTR_KEY_ID_MODE, CSMA_LLSEC_KEY_ID_MODE);
	#endif /* LLSEC802154_USES_EXPLICIT_KEYS */
	#endif /* LLSEC802154_ENABLED */

	if (csma_security_create_frame() < 0) //ERROR DE CREACIÓN DE ESPACIO PARA CABECERAS
	{
		LOG_ERR("failed to create packet, seqno: %d\n", packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
		ret = MAC_TX_ERR_FATAL;
	} 
	
	else //SE HAN CREADO BIEN LAS CABECERAS
	{
		int is_broadcast;
		uint8_t dsn;
		dsn = ((uint8_t*)packetbuf_hdrptr())[2] & 0xff;

		NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

		is_broadcast = packetbuf_holds_broadcast(); //SE COMPRUEBA SI ES POR BROADCAST

		//SE ESTÁ RECIBIENDO UN PAQUETE O YA HA SIDO RECIBIDO Y ESTÁ PENDIENTE DE SER LEÍDO Y NO HAY ACK
		if (NETSTACK_RADIO.receiving_packet() || (!is_broadcast && NETSTACK_RADIO.pending_packet())) 
			ret = MAC_TX_COLLISION;
	
		else 
		{
			switch (NETSTACK_RADIO.transmit(packetbuf_totlen())) 
			{
				
				case RADIO_TX_OK: //TRANSMISIÓN OK	
				
					if (is_broadcast) //SE RECIBE POR DIFUSIÓN (RADIO)
						ret = MAC_TX_OK;
					
					else //NO SE RECIBE POR DIFUSIÓN
					{						
						RTIMER_BUSYWAIT_UNTIL(NETSTACK_RADIO.pending_packet(), CSMA_ACK_WAIT_TIME); //COMPRUEBA ACK Y ESPERA MAX CSMA_ACK_WAIT_TIME 

						ret = MAC_TX_NOACK;
						
						if (NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet() || NETSTACK_RADIO.channel_clear() == 0) 
						{
							int len;
							uint8_t ackbuf[CSMA_ACK_LEN];

							//ESPERA UN "CSMA_AFTER_ACK_DETECTED_WAIT_TIME" ADICIONAL PARA COMPLETAR LA RECEPCIÓN
							RTIMER_BUSYWAIT_UNTIL(NETSTACK_RADIO.pending_packet(), CSMA_AFTER_ACK_DETECTED_WAIT_TIME);

							if (NETSTACK_RADIO.pending_packet()) //SI EL PAQUETE HA LLEGADO Y ESTÁ PENDIENTE DE SER LEÍDO
							{
								len = NETSTACK_RADIO.read(ackbuf, CSMA_ACK_LEN); //SE COGE LA LONGITUD
								
							    if (len == CSMA_ACK_LEN && ackbuf[2] == dsn) //SE HA RECIBIDO ACK
									ret = MAC_TX_OK;
				
								else //NO HAY ACK ENTONCES COLISIÓN
									ret = MAC_TX_COLLISION;
							}
						}
					}
				break;
				
				case RADIO_TX_COLLISION: //CASO COLISIÓN SI NO HAY ACK
					ret = MAC_TX_COLLISION;
				break;
				
				default: //CASO ERROR
					ret = MAC_TX_ERR;
				break;
			}
		}
	}
	
	if (ret == MAC_TX_OK) //SI SE HA INDICADO QUE LA TX HA SIDO OK
		last_sent_ok = 1;

	packet_sent(n, q, ret, 1); //SE LLAMA A LA FUNCIÓN DE ENVÍO DE PAQUETE PASÁNDOLE EL ESTADO (RET)
	return last_sent_ok;
}


static void transmit_from_queue (void *ptr) //SE TRANSMITE DESDE LA COLA
{
	struct neighbor_queue *n = ptr;
	
	if (n) //SI HAY COLA EN EL VECINO
	{
		struct packet_queue *q = list_head(n->packet_queue); 
		
		if (q != NULL) 
		{
			LOG_INFO("preparing packet for ");
			LOG_INFO_LLADDR(&n->addr);
			LOG_INFO_(", seqno %u, tx %u, queue %d\n", queuebuf_attr(q->buf, PACKETBUF_ATTR_MAC_SEQNO), n->transmissions, list_length(n->packet_queue));
			
			//SE MANDA EL PRIMER PAQUETE DE LA COLA DEL VECINO
			queuebuf_to_packetbuf(q->buf);
			send_one_packet(n, q);
		}
	}
}


static void schedule_transmission (struct neighbor_queue *n) //ASIGNA DELAYS PARA PLANIFICAR EN EL TIEMPO LAS TX
{
	clock_time_t delay;
	int backoff_exponent; /* BE in IEEE 802.15.4 */

	backoff_exponent = MIN(n->collisions + CSMA_MIN_BE, CSMA_MAX_BE);

	delay = ((1 << backoff_exponent) - 1) * backoff_period(); //MAX DELAY POSIBLE PARA IEEE 802.15.4: 2^BE-1 backoff periods
	
	if (delay > 0) 
		delay = random_rand() % delay; //SE DETERMINA EL DELAY PARA LA SIGUIENTE TX

	LOG_DBG("scheduling transmission in %u ticks, NB=%u, BE=%u\n", (unsigned)delay, n->collisions, backoff_exponent);
	ctimer_set(&n->transmit_timer, delay, transmit_from_queue, n);
}


static void free_packet (struct neighbor_queue *n, struct packet_queue *p, int status) //BORRA TODOS LOS PAQUETES DE UNA COLA DE UN VECINO
{
	if (p != NULL) 
	{
		list_remove(n->packet_queue, p); //SE BORRA EL PAQUETE DE LA COLA Y SE DESASIGNA

		queuebuf_free(p->buf);
		memb_free(&metadata_memb, p->ptr);
		memb_free(&packet_memb, p);
		
		LOG_DBG("free_queued_packet, queue length %d, free packets %d\n", list_length(n->packet_queue), memb_numfree(&packet_memb));
			   
		if (list_head(n->packet_queue) != NULL) //SI HAY SIGUIENTES PAQUETES 
		{
		    n->transmissions = 0; //SE RESETEA LA INFO DE TX
		    n->collisions = 0;
		  
		    schedule_transmission(n); //SE PLANIFICAN LAS SIGUIENTES TRANSMISIONES
		} 
		
		else //SI EL PAQUETE ACTUAL ERA EL ÚLTIMO (YA NO HAY MÁS EN COLA)
		{
		    ctimer_stop(&n->transmit_timer);
			
		    list_remove(neighbor_list, n); //SE BORRA EL VECINO DE LA LISTA Y COMO MIEMBRO
		    memb_free(&neighbor_memb, n); 
		}
	}
}


static void tx_done (int status, struct packet_queue *q, struct neighbor_queue *n) //REALIZA LA TX Y NOTIFICA
{
	mac_callback_t sent;
	struct qbuf_metadata *metadata;
	void *cptr;
	uint8_t ntx;

	metadata = (struct qbuf_metadata *)q->ptr;
	sent = metadata->sent;
	cptr = metadata->cptr;
	ntx = n->transmissions;

	LOG_INFO("packet sent to ");
	LOG_INFO_LLADDR(&n->addr);
	LOG_INFO_(", seqno %u, status %u, tx %u, coll %u\n", packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO), status, n->transmissions, n->collisions);

	free_packet(n, q, status); //SE BORRA DE LA COLA
	mac_call_sent_callback(sent, cptr, status, ntx);
}


static void rexmit (struct packet_queue *q, struct neighbor_queue *n)
{
	schedule_transmission(n);
	queuebuf_update_attr_from_packetbuf(q->buf); //SE ATRIBUYE LA ENERGÍA GASTADA EN TX EL PAQUETE
}


static void collision (struct packet_queue *q, struct neighbor_queue *n, int num_transmissions)
{
	struct qbuf_metadata *metadata;
	metadata = (struct qbuf_metadata *)q->ptr;

	n->collisions += num_transmissions;

	if (n->collisions > CSMA_MAX_BACKOFF) 
	{
		n->collisions = 0;
		n->transmissions++; //INDICA UN NUEVO INTENTO
	}

	if (n->transmissions >= metadata->max_transmissions) 
		tx_done(MAC_TX_COLLISION, q, n); 	
	else 
		rexmit(q, n);
}


static void noack (struct packet_queue *q, struct neighbor_queue *n, int num_transmissions)
{
	struct qbuf_metadata *metadata;
	metadata = (struct qbuf_metadata *)q->ptr;

	n->collisions = 0;
	n->transmissions += num_transmissions;

	if (n->transmissions >= metadata->max_transmissions) 
		tx_done(MAC_TX_NOACK, q, n);
	else 
		rexmit(q, n);
}


static void tx_ok (struct packet_queue *q, struct neighbor_queue *n, int num_transmissions)
{
	n->collisions = 0;
	n->transmissions += num_transmissions;
	tx_done(MAC_TX_OK, q, n);
}


static void packet_sent (struct neighbor_queue *n, struct packet_queue *q, int status, int num_transmissions)
{
	assert(n != NULL); //SE COMPRUEBA QUE EL VECINO Y SU COLA NO SON NULOS
	assert(q != NULL);

	if (q->ptr == NULL) 
	{
		LOG_WARN("packet sent: no metadata\n");
		return;
	}

	LOG_INFO("tx to ");
	LOG_INFO_LLADDR(&n->addr);
	LOG_INFO_(", seqno %u, status %u, tx %u, coll %u\n", packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO), status, n->transmissions, n->collisions);

	switch (status) //SE COMPRUEBA EL ESTADO SOBRE LA TX DE UN PAQUETE DETERMINADO
	{
		case MAC_TX_OK:
			tx_ok(q, n, num_transmissions);
		break;
		
		case MAC_TX_NOACK:
			noack(q, n, num_transmissions);
		break;
		
		case MAC_TX_COLLISION:
			collision(q, n, num_transmissions);
		break;
		
		case MAC_TX_DEFERRED:
		break;
		
		default:
			tx_done(status, q, n); //SE REALIZA CORRECTAMENTE
		break;
	}
}


void csma_output_packet (mac_callback_t sent, void *ptr)
{
	struct packet_queue *q;
	struct neighbor_queue *n;
	static uint8_t initialized = 0;
	static uint8_t seqno;
	const linkaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

	if (!initialized) //SI NO ESTÁ INICIALIZADO
	{
		initialized = 1;
		seqno = random_rand(); //INICIALIZA EL NÚMERO DE SECUENCIA CON UN NÚMERO ALEATORIO (802.15.4.)
	}

	if (seqno == 0) //PACKETBUF_ATTR_MAC_SEQNO NO PUEDE SER 0 (pecuilarity in framer-802154.c.)
		seqno++;

	packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno++);
	packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);

	n = neighbor_queue_from_addr(addr); //COLA DEL VECINO CORRESPONDIENTE SEGÚN LA DIRECCIÓN DADA
	
	if (n == NULL) //EL VECINO NO ESTÁ REGISTRADO Y NO TIENE DIRECCIÓN
	{
		n = memb_alloc(&neighbor_memb); //ASIGNA UNA NUEVA ENTRADA AL VECINO
	
		if (n != NULL) //SE HA ASIGNADO BIEN EL VECINO
		{
		  linkaddr_copy(&n->addr, addr); //INICIALIZA LA ENTRADA
		  
		  n->transmissions = 0;
		  n->collisions = 0;
		  
		  LIST_STRUCT_INIT(n, packet_queue); //INICIALIZA SU COLA 
		  list_add(neighbor_list, n); //AÑADE AL VECINO A LA LISTA DE VECINOS
		}
	}
	
	if (n != NULL) //EL VECINO TIENE DIRECCIÓN REGISTRADA
	{
		if (list_length(n->packet_queue) < CSMA_MAX_PACKET_PER_NEIGHBOR) //SE AÑADE PAQUETE A LA COLA 
		{
		    q = memb_alloc(&packet_memb); //ASIGNA UNA NUEVA ENTRADA AL PAQUETE
			
		    if (q != NULL) //SE HA ASIGNADO BIEN EL PAQUETE
			{
				q->ptr = memb_alloc(&metadata_memb); //ASIGNA METADATOS
				
				if (q->ptr != NULL) //SE HAN ASIGNADO BIEN LOS METADATOS
				{
					q->buf = queuebuf_new_from_packetbuf(); //ASIGNA UN BUFFER A LA COLA
					
					if (q->buf != NULL) //SE HA ASIGNADO BIEN EL BUFFER
					{
						struct qbuf_metadata *metadata = (struct qbuf_metadata *)q->ptr; //COMPLETA ASIGNACIÓN
						metadata->max_transmissions = packetbuf_attr(PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
						
						if (metadata->max_transmissions == 0) //SI NO SE HA DEFINIDO EL MÁXIMO SE UTILIZA EL VALOR CSMA POR DEFECTO
						{
						  metadata->max_transmissions = CSMA_MAX_FRAME_RETRIES + 1; 
						}
						
						metadata->sent = sent;
						metadata->cptr = ptr;
						list_add(n->packet_queue, q);

						LOG_INFO("sending to ");
						LOG_INFO_LLADDR(addr);
						LOG_INFO_(", len %u, seqno %u, queue length %d, free packets %d\n", packetbuf_datalen(), packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO), list_length(n->packet_queue), memb_numfree(&packet_memb));	
						
						if (list_head(n->packet_queue) == q) //SI q ES EL PRIMER PAQUETE EN LA COLA SE MANDA PLANIFICA (ASAP)
							schedule_transmission(n);
						
						return;
					}
					
					memb_free(&metadata_memb, q->ptr); //NO SE HA ASIGNADO BIEN EL BUFFER Y SE BORRAN LOS METADATOS
					LOG_WARN("could not allocate queuebuf, dropping packet\n");
				}
				
				memb_free(&packet_memb, q); //NO SE HAN ASIGNADO BIEN LOS METADATOS Y SE BORRA EL PAQUETE
				LOG_WARN("could not allocate queuebuf, dropping packet\n");
			}

			if (list_length(n->packet_queue) == 0) //NO SE HA ASIGNADO BIEN EL PAQUETE Y SE BORRA EL VECINO Y SU ENTRADA DE LA LISTA
			{
				list_remove(neighbor_list, n);
				memb_free(&neighbor_memb, n);
			}
		} 
		
		else //SI LA COLA YA ESTÁ LLENA NO SE PUEDEN AÑADIR PAQUETES
		  LOG_WARN("Neighbor queue full\n");
		
		LOG_WARN("could not allocate packet, dropping packet\n"); 
	} 
	
	else //ERROR DE DIRECCIÓN DEL VECINO: NO ESTABA REGISTRADO Y NO SE HA PODIDO ASIGNAR
		LOG_WARN("could not allocate neighbor, dropping packet\n");
	
	mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1);
}


void csma_output_init (void) //SE INICIALIZAN LAS ENTRADAS DE UN VECINO DETERMINADO, SU PAQUETE Y METADATOS
{
	memb_init(&packet_memb);
	memb_init(&metadata_memb);
	memb_init(&neighbor_memb);
}
