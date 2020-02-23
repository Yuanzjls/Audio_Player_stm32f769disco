#include "ff.h"
#include "main.h"
#define CODEPAGE FF_CODE_PAGE
static const BYTE DbcTbl[] = {0x81, 0xFE, 0x00, 0x00, 0x40, 0x7E, 0x80, 0xFE, 0x00, 0x00};
static int dbc_1st (BYTE c)
{
#if FF_CODE_PAGE == 0		/* Variable code page */
	if (DbcTbl && c >= DbcTbl[0]) {
		if (c <= DbcTbl[1]) return 1;					/* 1st byte range 1 */
		if (c >= DbcTbl[2] && c <= DbcTbl[3]) return 1;	/* 1st byte range 2 */
	}
#elif FF_CODE_PAGE >= 900	/* DBCS fixed code page */
	if (c >= DbcTbl[0]) {
		if (c <= DbcTbl[1]) return 1;
		if (c >= DbcTbl[2] && c <= DbcTbl[3]) return 1;
	}
#else						/* SBCS fixed code page */
	if (c != 0) return 0;	/* Always false */
#endif
	return 0;
}

/* Test if the character is DBC 2nd byte */
static int dbc_2nd (BYTE c)
{
#if FF_CODE_PAGE == 0		/* Variable code page */
	if (DbcTbl && c >= DbcTbl[4]) {
		if (c <= DbcTbl[5]) return 1;					/* 2nd byte range 1 */
		if (c >= DbcTbl[6] && c <= DbcTbl[7]) return 1;	/* 2nd byte range 2 */
		if (c >= DbcTbl[8] && c <= DbcTbl[9]) return 1;	/* 2nd byte range 3 */
	}
#elif FF_CODE_PAGE >= 900	/* DBCS fixed code page */
	if (c >= DbcTbl[4]) {
		if (c <= DbcTbl[5]) return 1;
		if (c >= DbcTbl[6] && c <= DbcTbl[7]) return 1;
		if (c >= DbcTbl[8] && c <= DbcTbl[9]) return 1;
	}
#else						/* SBCS fixed code page */
	if (c != 0) return 0;	/* Always false */
#endif
	return 0;
}
uint16_t filename_utf16[FF_LFN_BUF + 1];
static DWORD user_tchar2uni (	/* Returns character in UTF-16 encoding (>=0x10000 on double encoding unit, 0xFFFFFFFF on decode error) */
	const TCHAR** str		/* Pointer to pointer to TCHAR string in configured encoding */
)
{
	DWORD uc;
	const TCHAR *p = *str;

#if FF_LFN_UNICODE == 1		/* UTF-16 input */
	WCHAR wc;

	uc = *p++;	/* Get a unit */
	if (IsSurrogate(uc)) {	/* Surrogate? */
		wc = *p++;		/* Get low surrogate */
		if (!IsSurrogateH(uc) || !IsSurrogateL(wc)) return 0xFFFFFFFF;	/* Wrong surrogate? */
		uc = uc << 16 | wc;
	}

#elif FF_LFN_UNICODE == 2	/* UTF-8 input */
	BYTE b;
	int nf;

	uc = (BYTE)*p++;	/* Get a unit */
	if (uc & 0x80) {	/* Multiple byte code? */
		if ((uc & 0xE0) == 0xC0) {	/* 2-byte sequence? */
			uc &= 0x1F; nf = 1;
		} else {
			if ((uc & 0xF0) == 0xE0) {	/* 3-byte sequence? */
				uc &= 0x0F; nf = 2;
			} else {
				if ((uc & 0xF8) == 0xF0) {	/* 4-byte sequence? */
					uc &= 0x07; nf = 3;
				} else {					/* Wrong sequence */
					return 0xFFFFFFFF;
				}
			}
		}
		do {	/* Get trailing bytes */
			b = (BYTE)*p++;
			if ((b & 0xC0) != 0x80) return 0xFFFFFFFF;	/* Wrong sequence? */
			uc = uc << 6 | (b & 0x3F);
		} while (--nf != 0);
		if (uc < 0x80 || IsSurrogate(uc) || uc >= 0x110000) return 0xFFFFFFFF;	/* Wrong code? */
		if (uc >= 0x010000) uc = 0xD800DC00 | ((uc - 0x10000) << 6 & 0x3FF0000) | (uc & 0x3FF);	/* Make a surrogate pair if needed */
	}

#elif FF_LFN_UNICODE == 3	/* UTF-32 input */
	uc = (TCHAR)*p++;	/* Get a unit */
	if (uc >= 0x110000) return 0xFFFFFFFF;	/* Wrong code? */
	if (uc >= 0x010000) uc = 0xD800DC00 | ((uc - 0x10000) << 6 & 0x3FF0000) | (uc & 0x3FF);	/* Make a surrogate pair if needed */

#else		/* ANSI/OEM input */
	BYTE b;
	WCHAR wc;

	wc = (BYTE)*p++;			/* Get a byte */
	if (dbc_1st((BYTE)wc)) {	/* Is it a DBC 1st byte? */
		b = (BYTE)*p++;			/* Get 2nd byte */
		if (!dbc_2nd(b)) return 0xFFFFFFFF;	/* Invalid code? */
		wc = (wc << 8) + b;		/* Make a DBC */
	}
	if (wc != 0) {
		wc = ff_oem2uni(wc, CODEPAGE);	/* ANSI/OEM ==> Unicode */
		if (wc == 0) return 0xFFFFFFFF;	/* Invalid code? */
	}
	uc = wc;

#endif
	*str = p;	/* Next read pointer */
	return uc;
}

uint16_t GBK2UTF16(const char * str)
{
	DWORD uc;
	uint16_t i = 0;
	do
	{
		uc = user_tchar2uni(&str);
		if (uc!=0xFFFFFFFF)
		{
			filename_utf16[i] = uc;
			i++;
		}
	}while(uc!=0 && uc!=0xFFFFFFFF);

	return i;
}
