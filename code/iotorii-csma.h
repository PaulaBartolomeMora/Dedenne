

//#ifndef CSMA_H_
//#define CSMA_H_

#ifndef IOTORII_CSMA_H_
#define IOTORII_CSMA_H_

#include "hlmacaddr.h"
#include "net/linkaddr.h"

#include "contiki.h"
#include "net/mac/mac.h"
#include "dev/radio.h"

/*---------------------------------------------------------------------------*/

#ifdef IOTORII_CONF_NODE_TYPE
#define IOTORII_NODE_TYPE IOTORII_CONF_NODE_TYPE
#else
#define IOTORII_NODE_TYPE 0 //To support the traditional MAC operation
#endif

#ifdef CSMA_CONF_SEND_SOFT_ACK
#define CSMA_SEND_SOFT_ACK CSMA_CONF_SEND_SOFT_ACK
#else /* CSMA_CONF_SEND_SOFT_ACK */
#define CSMA_SEND_SOFT_ACK 0
#endif /* CSMA_CONF_SEND_SOFT_ACK */

#ifdef CSMA_CONF_ACK_WAIT_TIME
#define CSMA_ACK_WAIT_TIME CSMA_CONF_ACK_WAIT_TIME
#else /* CSMA_CONF_ACK_WAIT_TIME */
#define CSMA_ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* CSMA_CONF_ACK_WAIT_TIME */

#ifdef CSMA_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define CSMA_AFTER_ACK_DETECTED_WAIT_TIME CSMA_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* CSMA_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define CSMA_AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* CSMA_CONF_AFTER_ACK_DETECTED_WAIT_TIME */

#define CSMA_ACK_LEN 3

/* Default MAC len for 802.15.4 classic */
#ifdef  CSMA_MAC_CONF_LEN
#define CSMA_MAC_LEN CSMA_MAC_CONF_LEN
#else
#define CSMA_MAC_LEN 127 - 2
#endif

/*---------------------------------------------------------------------------*/

struct neighbour_table_entry //ESTRUCTURA DE ENTRADA DE TABLA
{
	struct neighbour_table_entry *next;
	linkaddr_t addr;
	uint8_t id;
};

typedef struct neighbour_table_entry neighbour_table_entry_t;
uint8_t number_of_neighbours;

/* just a default - with LLSEC, etc */
#define CSMA_MAC_MAX_HEADER 21

#if IOTORII_NODE_TYPE == 0 //SI OPERACIÓN MAC TRADICIONAL
extern const struct mac_driver csma_driver;
#elif IOTORII_NODE_TYPE > 0 //1 PARA NODO ROOT Y 2 PARA NODOS COMUNES
extern const struct mac_driver iotori_csma_driver;
#endif

/*---------------------------------------------------------------------------*/

//VER CSMA-SECURITY.C -->

//FUNCIONES DE SEGURIDAD CSMA DE CABECERA
int csma_security_create_frame (void);
int csma_security_parse_frame (void);

//MANTENIMIENTO DE CLAVE PARA CSMA
int csma_security_set_key (uint8_t index, const uint8_t *key);


#endif /* IOTORII_CSMA_H_ */
