/****************************************************************************
目的： 定义DGPS流动站软件的所有函数

编写时间：2011.7.6
作者：    王甫红
版本:     V2.0
版权：    武汉大学测绘学院
****************************************************************************/

#ifndef _Decode_H_
#define _Decode_H_

#include <stdio.h>
#include "DecodeStruct.h"
#include "GnssStruct.h"
#include "NavEphBDS.h"
#include "NavEphGPS.hpp"


#define OEM7SyncByte1		0xAA        // oem7 start sync code 1
#define OEM7SyncByte2		0x44        // oem7 start sync code 2
#define OEM7SyncByte3		0x12        // oem7 start sync code 3

#define OEM7HeaderLength    28          // oem7 header length (bytes)
#define OEM7CRCLength	    4			// oem7 crc length
#define ID_RANGE    43
#define ID_RANGECMP 140
#define ID_RAWEPHEM 41
#define ID_BD2EPHEM 1047
#define ID_GPSEPHEM 7
#define ID_BDSEPHEM 1696
#define ID_IONUTC   8           /* message id: Ionospheric and UTC model information */
#define ID_PSRPOS   47          /* message id: Pseudorange Position, BESTPOS 42,PDPPOS 469, PSRPOS 47*/
#define ID_BESTPOS  42
#define ID_PDPPOS   469

#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define MAXVAL      8388608.0

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

/* get fields (little-endian) ------------------------------------------------*/

#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))

// 从p指向的内存位置开始，读取2个字节（16位）的数据，
// 将其解释为一个unsigned short（无符号短整型）的值，并返回这个值。
// unsigned short通常用于表示16位的无符号整数。
inline static unsigned short U2(unsigned char* p) { unsigned short u; memcpy(&u, p, 2); return u; }

inline static unsigned int   U4(unsigned char* p) { unsigned int   u; memcpy(&u, p, 4); return u; }

// 从p指向的内存位置开始，读取4个字节（32位）的数据，
// 将其解释为一个int（整型）的值，并返回这个值。
// int通常用于表示32位的有符号整数。
inline static int            I4(unsigned char* p) { int            i; memcpy(&i, p, 4); return i; }

// 从p指向的内存位置开始，读取4个字节（32位）的数据，
// 将其解释为一个float（浮点型）的值，并返回这个值。
// float通常用于表示32位的浮点数。
inline static float          R4(unsigned char* p) { float          r; memcpy(&r, p, 4); return r; }

inline static double         R8(unsigned char* p) { double         r; memcpy(&r, p, 8); return r; }

// decode novatel oem4 and unicore

unsigned int crc32(const unsigned char *buff, int len);
int decode_rangeb_oem6(unsigned char* buff, EPOCHOBSDATA* obs);
int decode_rangeb_oem7(unsigned char* buff, ObsData* obsData);

int decode_GPSEph(unsigned char* buff, NavEphGPS* navEphGPS);
int decode_gpsephem(unsigned char* buff, GPSEPHREC* eph);

int decode_BDSEph(unsigned char* buff, NavEphBDS* bd);
int decode_bdsephem(unsigned char* buff, GPSEPHREC* eph);

int decode_ionutc(unsigned char* buff, IONOPARA* para);

int decode_psrpos(unsigned char* buff, double pos[]);

int exsign(unsigned int v, int bits);
unsigned int getbitu(const unsigned char *buff, int pos, int len);
int getbits(const unsigned char *buff, int pos, int len);


#endif