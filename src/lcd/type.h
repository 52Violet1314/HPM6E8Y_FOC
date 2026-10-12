/* Reserved for LCD driver compatibility. */

#ifndef __TYPE_H
#define __TYPE_H

typedef signed char        s8;
typedef short              s16;
typedef int                s32;

typedef unsigned char      u8;
typedef unsigned short     u16;
typedef unsigned int       u32;

void delay_ms(u32 ms);
void delay_us(u32 us);
u8 KeyGetValue(u8 key);

#endif
