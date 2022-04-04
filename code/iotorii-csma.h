

//#ifndef CSMA_H_
//#define CSMA_H_

#ifndef IOTORII_CSMA_H_
#define IOTORII_CSMA_H_

#include "hlmacaddr.h"
#include "hlmac-table.h"
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
	uint8_t number_id;
	uint8_t flag; //FLAG PADRE/HIJO OUTPUT/INPUT
	uint8_t load; //CARGA DE CADA NODO
};

typedef struct neighbour_table_entry neighbour_table_entry_t;
uint8_t number_of_neighbours;
uint8_t number_of_neighbours_flag; //PARA COMPROBAR SI EL NODO ES EDGE

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

/*---------------------------------------------------------------------------*/

//adicional
char *link_addr_to_str (const linkaddr_t addr);

#endif /* IOTORII_CSMA_H_ */

/*

init (void)
///////////
	--> csma_output_init()	...INICIALIZA MIEMBRO
	--> on()
	--> ctimer_set(&hello_timer, hello_start_time, iotorii_handle_hello_timer, NULL);
		--> max_payload()
		--> send_packet(NULL, NULL)
	--> ctimer_set(&statistic_timer, statistic_start_time, iotorii_handle_statistic_timer, NULL);
	--> //ROOT// ctimer_set(&sethlmac_timer, sethlmac_start_time, iotorii_handle_sethlmac_timer, NULL);
		--> hlmac_create_root_addr(&root_addr, 1)
		-->	hlmactable_add(root_addr) ...AÑADE LA DIRECCIÓN HLMAC ASIGNADA A LA TABLA DEL NODO SIEMPRE QUE NO SUPERE EL MAX
		--> iotorii_send_sethlmac(root_addr, linkaddr_node_addr)
			--> max_payload()
///////////



send_packet (mac_callback_t sent, void *ptr)
///////////
	-->	init_sec()
	-->	csma_output_packet(sent, ptr)	...PAQUETE DE SALIDA
		--> neighbor_queue_from_addr(addr) 
		--> schedule_transmission(n)
///////////



input_packet (void)	...SE RECIBE PAQUETE Y SE REALIZAN COMPROBACIONES
///////////
	--> iotorii_operation()
		--> //msg HELLO// iotorii_handle_incoming_hello() ...PROCESA HELLO Y AÑADE AL VECINO A SU TABLA DE VECINOS
		--> //msg setHLMAC// iotorii_handle_incoming_sethlmac_or_load()	...PROCESA setHLMAC
			--> iotorii_extract_address() ...SE COGE LA DIRECCIÓN DEL EMISOR
			--> hlmactable_has_loop(*received_hlmac_addr) ...COMPRUEBA QUE NO HAY BUCLE
			--> hlmactable_add(*received_hlmac_addr) ...AÑADE LA DIRECCIÓN HLMAC ASIGNADA A LA TABLA DEL NODO SIEMPRE QUE NO SUPERE EL MAX
		--> //msg LOAD// iotorii_handle_incoming_hello() ...PROCESA LOAD MESSAGE
///////////


on

off

max_payload
*/





/*
1) ROOT: iotorii_handle_sethlmac_timer()
   -> iotorii_send_sethlmac()
   
   COMÚN: iotorii_handle_sethlmac_timer()
   -> ctimer





*/