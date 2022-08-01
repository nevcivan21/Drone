/*
 * AESK_Data_Pack_lib.c
 *
 *  Created on: 17 Eki 2019
 *      Author: yemrelaydin
 */
#include "AESK_Data_Pack_Lib.h"


/***************************************  DATA PACKAGE START   ********************************************************/
/*
 *  BU KÜTÜPHANE LITTLE ENDIAN ve IEEE 754 FLOATING POINT bulunan ÝÞLEMCÝLER ÝÇÝN TASARLANMIÞTIR. 
 *	BU KÜTÜPHANEYÝ KULLANMADAN ÖNCE LÜTFEN ÝÞLEMCÝNÝZÝN HAFIZA YÖNETÝM YAPISINI ÝNCELEYÝNÝZ.
 *  EÐER ÝÞLEMCÝNÝZ BIG ENDIAN YAPISINDAYSA _LE ve _BE BÝRBÝRÝNÝN TERSÝ OLARAK ÇALIÞACAKTIR.
 *
 *
 */

/***************************************  UINT16 PACKAGE START ********************************************************/
/* Bu fonksiyon uint16_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt düþük indekse aktarýlýr.
 * Parametreler : uint16_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint16_t yemre = 0x4555;
 *  		uint8_t dizi[2];
 *
 *  		dizi[0] = 0x55;
 *  		dizi[1] = 0x45;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT16toUINT8_LE(uint16_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];

	*index = *index + sizeof(uint16_t);
}



/* Bu fonksiyon uint16_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt yüksek indekse aktarýlýr.
 * Parametreler : uint16_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint16_t yemre = 0x4555;
 *  		uint8_t dizi[2];
 *
 *  		dizi[0] = 0x45;
 *  		dizi[1] = 0x55;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT16toUINT8_BE(uint16_t *packData, uint8_t *packBuf, uint16_t *index)
{
		packBuf[*index] = ((uint8_t*)packData)[1];
		packBuf[(*index) + 1] = ((uint8_t*)packData)[0];

		*index = *index + sizeof(uint16_t);
}
/***************************************  INT16 PACKAGE END *********************************************************/

/***************************************  UINT16 PACKAGE START ********************************************************/
/* Bu fonksiyon int16_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt düþük indekse aktarýlýr.
 * Parametreler : uint16_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		int16_t yemre = 0x4555;
 *  		uint8_t dizi[2];
 *
 *  		dizi[0] = 0x55;
 *  		dizi[1] = 0x45;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_INT16toUINT8_LE(int16_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];

	*index = *index + sizeof(int16_t);
}



/* Bu fonksiyon int16_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt yüksek indekse aktarýlýr.
 * Parametreler : uint16_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		int16_t yemre = 0x4555;
 *  		uint8_t dizi[2];
 *
 *  		dizi[0] = 0x45;
 *  		dizi[1] = 0x55;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_INT16toUINT8_BE(int16_t *packData, uint8_t *packBuf, uint16_t *index)
{
		packBuf[*index] = ((uint8_t*)packData)[1];
		packBuf[(*index) + 1] = ((uint8_t*)packData)[0];

		*index = *index + sizeof(int16_t);
}
/***************************************  INT16 PACKAGE END *********************************************************/


/***************************************  UINT32 PACKAGE START *********************************************************/

/* Bu fonksiyon uint32_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt düþük indekse aktarýlýr.
 * Parametreler : uint32_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint32_t yemre = 0x45550012;
 *  		uint8_t dizi[4];
 *
 *  		dizi[0] = 0x12;
 *  		dizi[1] = 0x00;
 *  		dizi[2] = 0x55;
 *  		dizi[3] = 0x45;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT32toUINT8_LE(uint32_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[3];

	*index = *index + sizeof(uint32_t);
}


/* Bu fonksiyon uint32_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt yüksek indekse aktarýlýr.
 * Parametreler : uint32_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint32_t yemre = 0x45550012;
 *  		uint8_t dizi[4];
 *
 *  		dizi[0] = 0x45;
 *  		dizi[1] = 0x55;
 *  		dizi[2] = 0x00;
 *  		dizi[3] = 0x12;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT32toUINT8_BE(uint32_t *packData, uint8_t* packBuf, uint16_t *index)
{
		packBuf[*index] = ((uint8_t*)packData)[3];
		packBuf[(*index) + 1] = ((uint8_t*)packData)[2];
		packBuf[(*index) + 2] = ((uint8_t*)packData)[1];
		packBuf[(*index) + 3] = ((uint8_t*)packData)[0];

		*index = *index + sizeof(uint32_t);
}

/***************************************  UINT32 PACKAGE END *********************************************************/

/***************************************  UINT32 PACKAGE START *********************************************************/

/* Bu fonksiyon int32_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt düþük indekse aktarýlýr.
 * Parametreler : uint32_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint32_t yemre = 0x45550012;
 *  		uint8_t dizi[4];
 *
 *  		dizi[0] = 0x12;
 *  		dizi[1] = 0x00;
 *  		dizi[2] = 0x55;
 *  		dizi[3] = 0x45;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_INT32toUINT8_LE(int32_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[3];

	*index = *index + sizeof(int32_t);
}


/* Bu fonksiyon int32_t türünden veriyi uint8_t olarak paketlemeye yarar.
 * Buradaki tasarýmda düþük anlamlý bayt yüksek indekse aktarýlýr.
 * Parametreler : uint32_t *packData = paketlenecek verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketin toplanacaðý dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		int32_t yemre = 0x45550012;
 *  		uint8_t dizi[4];
 *
 *  		dizi[0] = 0x45;
 *  		dizi[1] = 0x55;
 *  		dizi[2] = 0x00;
 *  		dizi[3] = 0x12;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_INT32toUINT8_BE(int32_t *packData, uint8_t* packBuf, uint16_t *index)
{
		packBuf[*index] = ((uint8_t*)packData)[3];
		packBuf[(*index) + 1] = ((uint8_t*)packData)[2];
		packBuf[(*index) + 2] = ((uint8_t*)packData)[1];
		packBuf[(*index) + 3] = ((uint8_t*)packData)[0];

		*index = *index + sizeof(int32_t);
}

/***************************************  INT32 PACKAGE END *********************************************************/

/***************************************  FLOAT32 PACKAGE START *********************************************************/

void AESK_FLOAT32toUINT8_LE(float *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[3];

	*index = *index + sizeof(float);
}

void AESK_FLOAT32toUINT8_BE(float *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[3];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[0];

	*index = *index + sizeof(float);
}
/***************************************  FLOAT32 PACKAGE END *********************************************************/
/***************************************  FLOAT64(DOUBLE) PACKAGE START *********************************************************/

void AESK_FLOAT64toUINT8_LE(double *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[3];
	packBuf[(*index) + 4] = ((uint8_t*)packData)[4];
	packBuf[(*index) + 5] = ((uint8_t*)packData)[5];
	packBuf[(*index) + 6] = ((uint8_t*)packData)[6];
	packBuf[(*index) + 7] = ((uint8_t*)packData)[7];

	*index = *index + sizeof(double);
}

void AESK_FLOAT64toUINT8_BE(double *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[7];
	packBuf[(*index) + 1] = ((uint8_t*)packData)[6];
	packBuf[(*index) + 2] = ((uint8_t*)packData)[5];
	packBuf[(*index) + 3] = ((uint8_t*)packData)[4];
	packBuf[(*index) + 4] = ((uint8_t*)packData)[3];
	packBuf[(*index) + 5] = ((uint8_t*)packData)[2];
	packBuf[(*index) + 6] = ((uint8_t*)packData)[1];
	packBuf[(*index) + 7] = ((uint8_t*)packData)[0];

	*index = *index + sizeof(double);
}


/***************************************  FLOAT64(DOUBLE) PACKAGE END *********************************************************/
/***************************************  UINT8 PACKAGE START *********************************************************/

void AESK_UINT8toUINT8CODE(uint8_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];

	*index = *index + sizeof(uint8_t);
}

void AESK_INT8toUINT8CODE(int8_t *packData, uint8_t *packBuf, uint16_t *index)
{
	packBuf[*index] = ((uint8_t*)packData)[0];

		*index = *index + sizeof(int8_t);
}
/***************************************  UINT8 PACKAGE END *********************************************************/
/***************************************  DATA PACKAGE END  *********************************************************/



/***************************************  DATA ENCODE START  *********************************************************/

/***************************************  INT16_T ENCODE START  *********************************************************/
/* Bu fonksiyon uint8_t türünden veriyi int16_t olarak paketlemeye yarar.
 * Buradaki tasarým Little Endian paketlenmiþ veriyi birleþtirir.
 * Parametreler : int16_t *packData = paketlin açýlacaðý verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketlenen dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		int16_t yemre;
 *  		uint8_t dizi[4];
 *  		dizi[0] = 0x00;
 *  		dizi[1] = 0x12
 *  		yemre = 0x1200;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT8toINT16_LE(int16_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *)packData)[0] = packBuf[*index];
	((uint8_t *)packData)[1] = packBuf[(*index) + 1];

	*index = *index + sizeof(int16_t);
}




/* Bu fonksiyon uint8_t türünden veriyi int16_t olarak paketlemeye yarar.
 * Buradaki tasarým Big Endian paketlenmiþ veriyi birleþtirir.
 * Parametreler : int16_t *packData = paketlin açýlacaðý verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketlenen dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		int16_t yemre;
 *  		uint8_t dizi[2];
 *  		dizi[0] = 0x00;
 *  		dizi[1] = 0x12
 *  		yemre = 0x1200;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT8toINT16_BE(int16_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *)packData)[1] = packBuf[*index];
	((uint8_t *)packData)[0] = packBuf[(*index) + 1];

	*index = *index + sizeof(int16_t);
}
/***************************************  INT16_T ENCODE END  *********************************************************/




/***************************************  UINT16_T ENCODE START  *********************************************************/

/* Bu fonksiyon uint8_t türünden veriyi int16_t olarak paketlemeye yarar.
 * Buradaki tasarým Little Endian paketlenmiþ veriyi birleþtirir.
 * Parametreler : uint32_t *packData = paketlin açýlacaðý verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketlenen dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint16_t yemre;
 *  		uint8_t dizi[4];
 *  		dizi[2] = 0x00;
 *  		dizi[3] = 0x12
 *  		yemre = 0x1200;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT8toUINT16_LE(uint16_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *)packData)[0] = packBuf[*index];
	((uint8_t *)packData)[1] = packBuf[(*index) + 1];

	*index = *index + sizeof(int16_t);
}


/* Bu fonksiyon uint8_t türünden veriyi int16_t olarak paketlemeye yarar.
 * Buradaki tasarým Big Endian paketlenmiþ veriyi birleþtirir.
 * Parametreler : uint16_t *packData = paketlin açýlacaðý verinin adresi girilir.
 * 				: uint8_t  *packBuf  = paketlenen dizi girilir.
 * 				: uint16_t *index    = paketin indeks sýrasý girilir.
 * ÖRNEK:
 *  		uint16_t yemre;
 *  		uint8_t dizi[2];
 *  		dizi[0] = 0x00;
 *  		dizi[1] = 0x12
 *  		yemre = 0x0012;
 *  		fonksiyonun çýkýþý yukarýda gösterildiði gibi olur.
 */
void AESK_UINT8toUINT16_BE(uint16_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *)packData)[1] = packBuf[*index];
	((uint8_t *)packData)[0] = packBuf[(*index) + 1];

	*index = *index + sizeof(int16_t);
}
/***************************************  UINT16_T ENCODE END  *********************************************************/


/***************************************  INT32_T ENCODE START  *********************************************************/
void AESK_UINT8toINT32_LE(int32_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *) packData)[0] = packBuf[*index];
	((uint8_t *) packData)[1] = packBuf[(*index) + 1];
	((uint8_t *) packData)[2] = packBuf[(*index) + 2];
	((uint8_t *) packData)[3] = packBuf[(*index) + 3];

	*index = *index + sizeof(int32_t);
}

void AESK_UINT8toINT32_BE(int32_t *packData, uint8_t* packBuf, uint16_t *index)
{
	((uint8_t *) packData)[3] = packBuf[*index];
	((uint8_t *) packData)[2] = packBuf[(*index) + 1];
	((uint8_t *) packData)[1] = packBuf[(*index) + 2];
	((uint8_t *) packData)[0] = packBuf[(*index) + 3];

	*index = *index + sizeof(int32_t);
}
/***************************************  INT32_T ENCODE END  *********************************************************/

/***************************************  UINT32_T ENCODE START  *********************************************************/
void AESK_UINT8toUINT32_BE(uint32_t *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[3] = packBuf[(*index)];
	((uint8_t*) packData)[2] = packBuf[(*index) + 1];
	((uint8_t*) packData)[1] = packBuf[(*index) + 2];
	((uint8_t*) packData)[0] = packBuf[(*index) + 3];

	*index = *index + sizeof(uint32_t);
}

void AESK_UINT8toUINT32_LE(uint32_t *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[0] = packBuf[(*index)];
	((uint8_t*) packData)[1] = packBuf[(*index) + 1];
	((uint8_t*) packData)[2] = packBuf[(*index) + 2];
	((uint8_t*) packData)[3] = packBuf[(*index) + 3];

	*index = *index + sizeof(uint32_t);
}

/***************************************  UINT32_T ENCODE END  *********************************************************/


/***************************************  UINT8_T - INT8_T ENCODE START  *********************************************************/
void AESK_UINT8toUINT8ENCODE(uint8_t *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[0] = packBuf[(*index)];

	*index = *index + sizeof(uint8_t);
}

void AESK_UINT8toINT8ENCODE(int8_t *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[0] = packBuf[(*index)];

	*index = *index + sizeof(uint8_t);
}
/***************************************  UINT8_T - INT8_T ENCODE END  *********************************************************/

/***************************************  FLOAT32 ENCODE START  *********************************************************/
void AESK_UINT8toFLOAT32_LE(float *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[0] = packBuf[(*index)];
	((uint8_t*) packData)[1] = packBuf[(*index) + 1];
	((uint8_t*) packData)[2] = packBuf[(*index) + 2];
	((uint8_t*) packData)[3] = packBuf[(*index) + 3];

	*index = *index + sizeof(float);
}

void AESK_UINT8toFLOAT32_BE(float *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[3] = packBuf[(*index)];
	((uint8_t*) packData)[2] = packBuf[(*index) + 1];
	((uint8_t*) packData)[1] = packBuf[(*index) + 2];
	((uint8_t*) packData)[0] = packBuf[(*index) + 3];

	*index = *index + sizeof(float);
}
/***************************************  FLOAT32 ENCODE END  *********************************************************/

/***************************************  FLOAT64 ENCODE START  *********************************************************/
void AESK_UINT8toFLOAT64_LE(double *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[0] = packBuf[(*index)];
	((uint8_t*) packData)[1] = packBuf[(*index) + 1];
	((uint8_t*) packData)[2] = packBuf[(*index) + 2];
	((uint8_t*) packData)[3] = packBuf[(*index) + 3];
	((uint8_t*) packData)[4] = packBuf[(*index) + 4];
	((uint8_t*) packData)[5] = packBuf[(*index) + 5];
	((uint8_t*) packData)[6] = packBuf[(*index) + 6];
	((uint8_t*) packData)[7] = packBuf[(*index) + 7];
	*index = *index + sizeof(double);
}

void AESK_UINT8toFLOAT64_BE(double *packData, uint8_t *packBuf, uint16_t *index)
{
	((uint8_t*) packData)[7] = packBuf[(*index)];
	((uint8_t*) packData)[6] = packBuf[(*index) + 1];
	((uint8_t*) packData)[5] = packBuf[(*index) + 2];
	((uint8_t*) packData)[4] = packBuf[(*index) + 3];
	((uint8_t*) packData)[3] = packBuf[(*index) + 4];
	((uint8_t*) packData)[2] = packBuf[(*index) + 5];
	((uint8_t*) packData)[1] = packBuf[(*index) + 6];
	((uint8_t*) packData)[0] = packBuf[(*index) + 7];
	*index = *index + sizeof(double);
}

/***************************************  FLOAT64 ENCODE END  *********************************************************/


/*
 * BU BÖLÜM AESK VERÝ YAPISINA ÖZEL OLARAK TASARLANMIÞTIR. AESK VERÝ YAPISI MANTIÐINDA KULLANILAN
 * FLOAT DEÐERÝN BÝR KAT SAYI ÝLE ÇARPILARAK INTEGER HALINE GETÝRÝLMESÝ KULLANILARAK YAPILAN TASARIMDA
 * FONKSÝYON ÝÇERÝSÝNDE YER ALAN CONVERTER PARAMETRESÝ:
 *
 *   FLOAT_CONVERTER_1 			10
 *	 FLOAT_CONVERTER_2 			100
 *	 FLOAT_CONVERTER_3 			1000
 *
 *	 OLARAK KAYITLIDIR.
 *
 */
void AESK_UINT16toFLOAT_BE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index)
{
	 uint16_t data ;
	((uint8_t *)&data)[1] = packBuf[*index];
	((uint8_t *)&data)[0] = packBuf[(*index) + 1];

	*packData = (float)data / CONVERTER;
	*index = *index + sizeof(uint16_t);
}

/*
 * BU BÖLÜM AESK VERÝ YAPISINA ÖZEL OLARAK TASARLANMIÞTIR. AESK VERÝ YAPISI MANTIÐINDA KULLANILAN
 * FLOAT DEÐERÝN BÝR KAT SAYI ÝLE ÇARPILARAK INTEGER HALINE GETÝRÝLMESÝ KULLANILARAK YAPILAN TASARIMDA
 * FONKSÝYON ÝÇERÝSÝNDE YER ALAN CONVERTER PARAMETRESÝ:
 *
 *   FLOAT_CONVERTER_1 			10
 *	 FLOAT_CONVERTER_2 			100
 *	 FLOAT_CONVERTER_3 			1000
 *
 *	 OLARAK KAYITLIDIR.
 *
 */
void AESK_UINT16toFLOAT_LE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index)
{
	 uint16_t data;
	((uint8_t *)&data)[0] = packBuf[*index];
	((uint8_t *)&data)[1] = packBuf[(*index) + 1];

	*packData = (float)data / CONVERTER;
	*index = *index + sizeof(uint16_t);
}

/*
 * BU BÖLÜM AESK VERÝ YAPISINA ÖZEL OLARAK TASARLANMIÞTIR. AESK VERÝ YAPISI MANTIÐINDA KULLANILAN
 * FLOAT DEÐERÝN BÝR KAT SAYI ÝLE ÇARPILARAK INTEGER HALINE GETÝRÝLMESÝ KULLANILARAK YAPILAN TASARIMDA
 * FONKSÝYON ÝÇERÝSÝNDE YER ALAN CONVERTER PARAMETRESÝ:
 *
 *   FLOAT_CONVERTER_1 			10
 *	 FLOAT_CONVERTER_2 			100
 *	 FLOAT_CONVERTER_3 			1000
 *
 *	 OLARAK KAYITLIDIR.
 *
 */
void AESK_INT16toFLOAT_BE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index)
{
	 int16_t data ;
	((uint8_t *)&data)[1] = packBuf[*index];
	((uint8_t *)&data)[0] = packBuf[(*index) + 1];

	*packData = (float)data / CONVERTER;
	*index = *index + sizeof(int16_t);
}

/*
 * BU BÖLÜM AESK VERÝ YAPISINA ÖZEL OLARAK TASARLANMIÞTIR. AESK VERÝ YAPISI MANTIÐINDA KULLANILAN
 * FLOAT DEÐERÝN BÝR KAT SAYI ÝLE ÇARPILARAK INTEGER HALINE GETÝRÝLMESÝ KULLANILARAK YAPILAN TASARIMDA
 * FONKSÝYON ÝÇERÝSÝNDE YER ALAN CONVERTER PARAMETRESÝ:
 *
 *   FLOAT_CONVERTER_1 			10
 *	 FLOAT_CONVERTER_2 			100
 *	 FLOAT_CONVERTER_3 			1000
 *
 *	 OLARAK KAYITLIDIR.
 *
 */
void AESK_INT16toFLOAT_LE(float *packData, uint8_t *packBuf, uint16_t CONVERTER, uint16_t *index)
{
	 int16_t data;
	((uint8_t *)&data)[0] = packBuf[*index];
	((uint8_t *)&data)[1] = packBuf[(*index) + 1];

	*packData = (float)data / CONVERTER;

	*index = *index + sizeof(int16_t);
}



