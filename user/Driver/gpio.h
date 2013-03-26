#include <Cfg/config.h>


#define ST_PA8		0x08
#define ST_PA11		0x0b
#define ST_PA12		0x0c

#define SPI2_IRQ		0x11
#define ST_PB5			0x15



void           GPIO_Configuration( void );
extern const vu16 io_bit[16];
u8             rd_io( u8 pin );
void           wr_io( u8 pin, u8 val );
void           GPIO_Configuration( void );
