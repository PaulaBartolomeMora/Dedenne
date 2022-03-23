

#include "contiki.h"
#include "hlmacaddr.h"
#include <stdio.h> //For sprintf()
#include <stdlib.h> //For malloc()

#include "sys/log.h"
#define LOG_MODULE "IoTorii-HLMAC-Address"
#define LOG_LEVEL LOG_LEVEL_MAC

const hlmacaddr_t UNSPECIFIED_HLMAC_ADDRESS = {NULL, 0};

/*---------------------------------------------------------------------------*/

uint8_t hlmac_is_unspecified_addr (const hlmacaddr_t addr)
{
	if ((addr.len == 0) && (!addr.address))
		return 1; //NO ESTÁ ESPECIFICADA
	else
		return 0;
}
 

uint8_t hlmac_get_len (const hlmacaddr_t addr)
{
	return addr.len;
}


uint8_t *hlmac_get_address (const hlmacaddr_t addr)
{
	uint8_t *address = (uint8_t *) malloc (sizeof(uint8_t) * addr.len);
	uint8_t i;
	
	for (i = 0; i < addr.len; i++)
	{ 
		address[i] = addr.address[i];
	}
	return address;
}


void hlmac_create_root_addr (hlmacaddr_t *root_addr, const uint8_t root_id)
{
  root_addr->address = (uint8_t*) malloc (sizeof(uint8_t));
  root_addr->address[0] = root_id;
  root_addr->len = 1;
  
  //LOG_DBG("Root.address: %d, Root.len: %d", node_hlmac_address.address[0], node_hlmac_address.len);
}


void hlmac_add_new_id (hlmacaddr_t *addr, const uint8_t new_id)
{
	uint8_t *temp = addr->address; //SE GUARDA LA DIRECCIÓN
	addr->address = (uint8_t*) malloc (sizeof(uint8_t) * (addr->len + 1));
	uint8_t i;
	
	for (i = 0; i < addr->len; i++)
	{
		addr->address[i] = temp[i]; //SE PASA LA DIRECCIÓN
	}
	
	addr->address[i] = new_id; //SE AÑADE AL FINAL DE LA DIRECCIÓN
	(addr->len)++; //SE INCREMENTA LA LONGITUD AL AÑADIR EL ID
	
	if (temp != NULL) //SI ADDR ESTÁ ESPECIFICADO (TEMP ES NULL SI NO LO ESTÁ)
	{ 
		free(temp);
		temp = NULL;
	}
}


void hlmac_remove_Last_id (hlmacaddr_t *addr)
{
	if (addr->len == 0) //NO HAY DIRECCIÓN
		return;
	
	if (addr->len == 1) //SI SOLO HAY UN ID DIRECTAMENTE SE BORRA
	{
		free(addr->address);
		addr->address = NULL;
		addr->len = 0;
		return;
	}
	
	//SI addr->len == 2 
	uint8_t *temp = addr->address;
	addr->address = (uint8_t*) malloc (sizeof(uint8_t) * (addr->len - 1));
	uint8_t i;
	
	for (i = 0; i < addr->len-1; i++) 
	{ 
		addr->address[i] = temp[i]; //SE GUARDA LA DIRECCIÓN HASTA LA PENÚLTIMA POSICIÓN
	}
	
	(addr->len)--; //SE DECREMENTA LA LONGITUD	
	free(temp);
	temp = NULL;
}


char hlmac_cmp (const hlmacaddr_t addr1, const hlmacaddr_t addr2)
{
	uint8_t min_len = (addr1.len < addr2.len) ? addr1.len : addr2.len; //SE DEFINE LA LONGITUD MIN
	uint8_t i;

	//COMPARACIÓN CON EL CAMPO ADDRESS
	for (i = 0; i < min_len; i++) 
	{
		if (addr1.address[i] < addr2.address[i]) //SI LA SEGUNDA ES MAYOR
			return +1;
		else if (addr1.address[i] > addr2.address[i]) //SI LA PRIMERA ES MAYOR
			return -1;
	}
	
	//COMPARACIÓN CON EL CAMPO LEN
	if (addr1.len == addr2.len) //LAS LONGITUDES SON IGUALES
		return 0;
		
	if (addr1.len < addr2.len) //SI LA SEGUNDA ES MAYOR
	{
		for (; i < addr2.len; i++)
		{ 
		    if (addr2.address[i] > 0) //COMPRUEBA QUE LAS ID NO SEAN .0
			    return +1;
		}
		return 0; //SI ALGUNA ID ES .0
	}
	
	if (addr1.len > addr2.len)
	{
		for (; i< addr1.len; i++)
		{ 
		    if (addr1.address[i] > 0) //COMPRUEBA QUE LAS ID NO SEAN .0
			    return -1; 
		}
		return 0; //SI ALGUNA ID ES .0
	}
	return -2; //SI ERROR 
}


uint8_t get_addr_index_value (const hlmacaddr_t addr, const uint8_t k)
{
	if (k < (addr.len)) //SI EL ÍNDICE ES CORRECTO Y ESTÁ DENTRO DEL RANGO
		return addr.address[k]; 
	else //ERROR DE ÍNDICE
	{
		const char *addr_str = hlmac_addr_to_str(addr);
		LOG_ERR("get_addr_index_value(): index %d is out of range %s", k, addr_str);
		//free(addr_str); // \fexme const => err, leakage?
		return 0; 
	}
}


char *hlmac_addr_to_str (const hlmacaddr_t addr)
{
	char *address = (char*) malloc (sizeof(char) * (addr.len * (2 + 1) + 1)); //SE AÑADE ESPACIO PARA LOS . Y PARA '\0'
	char *s = (char *)address;
	uint8_t i;
	
	for (i = 0; i < addr.len; i++, s += 3)
	    sprintf(s, "%2.2X.", (char)get_addr_index_value(addr, i));
	
	*(s) = '\0'; //SE AÑADE '\0' AL FINAL
	
	return address;
}


