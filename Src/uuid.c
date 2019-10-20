/*
 * uuid.c
 *
 *  Created on: 18 ��� 2019 �.
 *      Author: ˸��
 */

#include "uuid.h"
#include "string.h"

char *pAlphabeth = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

// limitation: remainder must fit into 16 bits
void big_num_div(uint16_t *bigNumber, uint8_t size, uint16_t divisor, uint16_t *bigResult, uint16_t *remainder)
{
	uint32_t locRem = 0;
	uint32_t locRes = 0;
	for (int i = 0; i < size; ++i)
	{
		// take next 16 bits
		uint32_t value = (locRem << 16) + bigNumber[i];

		locRes = value / divisor;
		locRem = value % divisor;

		bigResult[i] = locRes;
	}

	// return the last local remainder as the overall remainder
	*remainder = locRem;
}

uint8_t big_num_non_zero(uint16_t *bigNumber, uint8_t size)
{
	uint16_t *end = bigNumber + size;
	while (bigNumber < end)
	{
		if (*bigNumber) return 1;
		bigNumber += 1;
	}
	return 0;
}

// needs a buffer of at least 20 bytes
void get_uid_str(char * pUid) {

	uint32_t base = 62;
	uint8_t len = 6;

	uint8_t digits = 0;
	pUid[19] = 0;

	uint16_t bigResult[len];
	uint16_t bigNumber[len];
	uint16_t remainder;
	memcpy(bigNumber, STM32_UUID_16, len*2);

	// 32-bit / 16-bit => 16-bit result + 16-bit remainder
    while (big_num_non_zero(bigNumber, len) && digits < 19)
    {
    	big_num_div(bigNumber, len, base, bigResult, &remainder);

    	pUid[digits] = pAlphabeth[remainder];
    	++digits;
        // copy result to source
    	memcpy(bigNumber, bigResult, len*sizeof(uint16_t));
    }
    pUid[digits] = 0;
}
