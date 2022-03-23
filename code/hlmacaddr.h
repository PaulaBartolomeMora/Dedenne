

#ifndef HLMACADDR_H_
#define HLMACADDR_H_

#include "contiki.h"

#ifndef LOG_CONF_DBG_DEVELOPER
#define LOG_DBG_DEVELOPER 0
#else
#define LOG_DBG_DEVELOPER LOG_CONF_DBG_DEVELOPER
#endif

#ifndef LOG_CONF_DBG_STATISTIC
#define LOG_DBG_STATISTIC 0
#else
#define LOG_DBG_STATISTIC LOG_CONF_DBG_STATISTIC
#endif


//POR DEFECTO IOTORII TIENE CONFIFURADO: n-HLMAC = 1 (1-HLMAC)
//SI HLMAC_CONF_MAX_HLMAC == -1, TIENE CONFIGURADO: n-HLMAC = unlimited (all-HLMAC)
#ifndef HLMAC_CONF_MAX_HLMAC
#define HLMAC_MAX_HLMAC 1
#else
#define HLMAC_MAX_HLMAC HLMAC_CONF_MAX_HLMAC
#endif

/*---------------------------------------------------------------------------*/

typedef struct 
{
	uint8_t *address; //ANCHURA DE DIRECCIÓN [0 255]
	uint8_t len; //LONGITUD DE DIRECCIÓN [0 255]
} hlmacaddr_t;

/*---------------------------------------------------------------------------*/

//DEFINE DIRECCIÓN HLMAC NO ESPECIFICADA
extern const hlmacaddr_t UNSPECIFIED_HLMAC_ADDRESS;

//COMPRUEBA SI UNA DIRECCIÓN HLMAC ESTÁ SIN ESPECIFICAR
uint8_t hlmac_is_unspecified_addr (const hlmacaddr_t addr);

//DEVUELVE LA LONGITUD DE UNA DIRECCIÓN HLMAC (CAMPO LEN DE ADDR)
uint8_t hlmac_get_len (const hlmacaddr_t addr);

//DEVUELVE EL CAMPO DIRECCIÓN DE ADDR
uint8_t *hlmac_get_address (const hlmacaddr_t addr);

//CREA NUEVA DIRECCIÓN HLMAC CON ID ROOT
void hlmac_create_root_addr (hlmacaddr_t *root_addr, const uint8_t root_id);

//void setIndexValue(unsigned uint8_t k, unsigned uint8_t addrbyte);

//AÑADE UN ID A UNA DIRECCIÓN HLMAC DETERMINADA, DEVUELVE ADDR SIN SU ÚLTIMO ID
void hlmac_add_new_id (hlmacaddr_t *addr, const uint8_t new_id);

//BORRA EL ÚLTIMO ID DE UNA DIRECCIÓN HLMAC DETERMINADA
void hlmac_remove_Last_id (hlmacaddr_t *addr);

//COMPARA LA LONGITUD DE DOS DIRECCIONES HLMAC
//DEVUELVE 0 SI SON IGUALES, +1 SI 2 ES MAYOR, -1 SI 1 ES MAYOR
char hlmac_cmp (const hlmacaddr_t addr1, const hlmacaddr_t addr2);

//DEVUELVE EL VALOR DE UN ID DE LA DIRECCIÓN DADO SU ÍNDICE
uint8_t get_addr_index_value (const hlmacaddr_t addr, const uint8_t k);

//DEVUELVE EL CAMPO ADDRESS DE UNA DIRECCIÓN HLMAC DETERMINADA COMO STRING Y AÑADE '\0' AL FINAL
char *hlmac_addr_to_str(hlmacaddr_t addr);



#endif /* HLMACADDR_H_ */
