

#ifndef CSMA_OUTPUT_H_
#define CSMA_OUTPUT_H_

#include "contiki.h"
#include "net/mac/mac.h"

/*---------------------------------------------------------------------------*/

void csma_output_packet (mac_callback_t sent, void *ptr);
void csma_output_init (void);

#endif /* CSMA_OUTPUT_H_ */


/*

transmit_from_queue()
	- SE TRANSMITE DESDE LA COLA DEL VECINO
	- SE REALIZAN COMPROBACIONES
	- SE MANDA EL PRIMER PAQUETE -> LLAMA A send_one_packet()

send_one_packet() 
	- EMPAQUETA (CABECERAS)
	- ENVÍA EL PAQUETE -> LLAMA A packet_sent()
	- ret DEVUELVE ESTADO
	
packet_sent()
	- SE COMPRUEBA EL ESTADO DE LA TX DEL PAQUETE
	- SI ES CORRECTA, SE REALIZA LA TX -> SE LLAMA A tx_done()

tx_done()
	- SE ENVÍA EL PAQUETE 
	- SE NOTIFICA
	- SE BORRA DE LA COLA -> SE LLAMA A free_packet()
	
*/