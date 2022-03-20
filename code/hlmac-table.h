

#ifndef HLMACTABLE_H_
#define HLMACTABLE_H_

#include "contiki.h"
#include "hlmacaddr.h"

/*---------------------------------------------------------------------------*/

struct hlmac_table_entery
{
	struct hlmac_table_entery *next;
	hlmacaddr_t address;
};

typedef struct hlmac_table_entery hlmac_table_entery_t;
uint8_t number_of_hlmac_addresses;

/*---------------------------------------------------------------------------*/

//INICIALIZA LA TABLA DE DIRECCIONES HLMAC
void hlmac_table_init (void);

//AÑADE UNA DIRECCIÓN HLMAC AL FINAL DE LA TABLA
uint8_t hlmactable_add (const hlmacaddr_t addr);

//COMPRUEBA SI LA DIRECCION HLMAC DADA CREA UN BUCLE O NO
//DEVUELVE UN 1 SI HAY BUCLE Y 0 SI NO
uint8_t hlmactable_has_loop (const hlmacaddr_t addr);

//ENCUENTRA EL PREFIJO MAS LARGO DADO PARA UNA DIRECCIÓN HLMAC
//DEVUELVE EL PREFIJO SI EXISTE Y SI NO "UNSPECIFIED"
hlmacaddr_t* hlmactable_get_longest_matchhed_prefix (const hlmacaddr_t addr);


//DEVUELVE EL NÚMERO DE SALTOS
//#if LOG_DBG_STATISTIC == 1
int hlmactable_calculate_sum_hop (void);
#endif

#endif /* HLMACTABLE_H_ */
