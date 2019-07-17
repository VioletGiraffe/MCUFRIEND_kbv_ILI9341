#include "MCUFRIEND_kbv.h"
#include "utility/mcufriend_shield.h"

#define MIPI_DCS_REV1 (1 << 0)
#define AUTO_READINC (1 << 1)
#define READ_BGR (1 << 2)
#define READ_LOWHIGH (1 << 3)
#define READ_24BITS (1 << 4)
#define XSA_XEA_16BIT (1 << 5)
#define READ_NODUMMY (1 << 6)
#define INVERT_GS (1 << 8)
#define INVERT_SS (1 << 9)
#define MV_AXIS (1 << 10)
#define INVERT_RGB (1 << 11)
#define REV_SCREEN (1 << 12)
#define FLIP_VERT (1 << 13)
#define FLIP_HORIZ (1 << 14)

#if (defined(USES_16BIT_BUS)) //only comes from SPECIALs
#define USING_16BIT_BUS 1
#else
#define USING_16BIT_BUS 0
#endif

constexpr uint16_t _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;

MCUFRIEND_kbv::MCUFRIEND_kbv(int CS, int RS, int WR, int RD, int _RST) : Adafruit_GFX(240, 320)
{
	// we can not access GPIO pins until AHB has been enabled.
}

constexpr uint16_t color565_to_555(uint16_t color)
{
	return (color & 0xFFC0) | ((color & 0x1F) << 1) | ((color & 0x01)); //lose Green LSB, extend Blue LSB
}
constexpr uint16_t color555_to_565(uint16_t color)
{
	return (color & 0xFFC0) | ((color & 0x0400) >> 5) | ((color & 0x3F) >> 1); //extend Green LSB
}
constexpr uint8_t color565_to_r(uint16_t color)
{
	return ((color & 0xF800) >> 8); // transform to rrrrrxxx
}
constexpr uint8_t color565_to_g(uint16_t color)
{
	return ((color & 0x07E0) >> 3); // transform to ggggggxx
}
constexpr uint8_t color565_to_b(uint16_t color)
{
	return ((color & 0x001F) << 3); // transform to bbbbbxxx
}

void MCUFRIEND_kbv::reset(void)
{
	setWriteDir();
	CTL_INIT();
	CS_IDLE;
	RD_IDLE;
	WR_IDLE;
	RESET_IDLE;
	delay(50);
	RESET_ACTIVE;
	delay(100);
	RESET_IDLE;
	delay(100);
	WriteCmdData(0xB0, 0x0000); //R61520 needs this to read ID

	_resetPerformed = true;
}

void MCUFRIEND_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
{
	CS_ACTIVE;
	WriteCmd(cmd);
	WriteData(dat);
	CS_IDLE;
}

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t *block)
{
	CS_ACTIVE;
	WriteCmd(cmd);
	for (; N > 0; --N)
	{
		uint8_t u8 = *block++;
		write8(u8);
	}
	CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
	uint8_t d[4] = {d1, d2, d3, d4};
	WriteCmdParamN(cmd, 4, d);
}

//#define WriteCmdParam4(cmd, d1, d2, d3, d4) {uint8_t d[4];d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;WriteCmdParamN(cmd, 4, d);}
void MCUFRIEND_kbv::pushCommand(uint16_t cmd, uint8_t *block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void)
{
	uint16_t ret;
	uint8_t lo;
#if USING_16BIT_BUS
	READ_16(ret);  //single strobe to read whole bus
	if (ret > 255) //ID might say 0x00D3
		return ret;
#else
	READ_8(ret);
#endif
	//all MIPI_DCS_REV1 style params are 8-bit
	READ_8(lo);
	return (ret << 8) | lo;
}

uint16_t MCUFRIEND_kbv::readReg(uint16_t reg, int8_t index)
{
	uint16_t ret;
	CS_ACTIVE;
	WriteCmd(reg);
	setReadDir();
	delay(1); //1us should be adequate
	//    READ_16(ret);
	do
	{
		ret = read16bits();
	} while (--index >= 0); //need to test with SSD1963
	RD_IDLE;
	CS_IDLE;
	setWriteDir();
	return ret;
}

uint32_t MCUFRIEND_kbv::readReg32(uint16_t reg)
{
	uint16_t h = readReg(reg, 0);
	uint16_t l = readReg(reg, 1);
	return ((uint32_t)h << 16) | (l);
}

uint32_t MCUFRIEND_kbv::readReg40(uint16_t reg)
{
	uint16_t h = readReg(reg, 0);
	uint16_t m = readReg(reg, 1);
	uint16_t l = readReg(reg, 2);
	return ((uint32_t)h << 24) | (m << 8) | (l >> 8);
}

uint16_t MCUFRIEND_kbv::readID(void)
{
	if (!_resetPerformed)
		reset();

	const uint16_t ret = readReg32(0xD3); //for ILI9488, 9486, 9340, 9341
	const uint8_t msb = ret >> 8;
	if (!(msb == 0x93 || msb == 0x94 || msb == 0x98 || msb == 0x77 || msb == 0x16)) //0x9488, 9486, 9340, 9341, 7796
		Serial.println(F("Invalid controller ID"));

	return ret;
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.
int16_t MCUFRIEND_kbv::readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h)
{
	uint16_t ret, dummy, _MR = _MW;
	int16_t n = w * h, row = 0, col = 0;
	uint8_t r, g, b;

	setAddrWindow(x, y, x + w - 1, y + h - 1);
	while (n > 0)
	{
		if constexpr (!(_lcd_capable & MIPI_DCS_REV1))
		{
			WriteCmdData(_MC, x + col);
			WriteCmdData(_MP, y + row);
		}
		CS_ACTIVE;
		WriteCmd(_MR);
		setReadDir();
		if constexpr (_lcd_capable & READ_NODUMMY)
		{
			;
		}
		else if constexpr ((_lcd_capable & MIPI_DCS_REV1))
		{
			READ_8(r);
		}
		else
		{
			READ_16(dummy);
		}

		while (n)
		{
			if constexpr (_lcd_capable & READ_24BITS)
			{
				READ_8(r);
				READ_8(g);
				READ_8(b);
				if constexpr (_lcd_capable & READ_BGR)
					ret = color565(b, g, r);
				else
					ret = color565(r, g, b);
			}
			else
			{
				READ_16(ret);
				if constexpr (_lcd_capable & READ_LOWHIGH)
					ret = (ret >> 8) | (ret << 8);
				if constexpr (_lcd_capable & READ_BGR)
					ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
			}
#if defined(SUPPORT_9488_555)
			if (is555)
				ret = color555_to_565(ret);
#endif
			*block++ = ret;
			n--;
			if constexpr (!(_lcd_capable & AUTO_READINC))
				break;
		}
		if (++col >= w)
		{
			col = 0;
			if (++row >= h)
				row = 0;
		}
		RD_IDLE;
		CS_IDLE;
		setWriteDir();
	}
	if constexpr (!(_lcd_capable & MIPI_DCS_REV1))
		setAddrWindow(0, 0, width() - 1, height() - 1);
	return 0;
}

void MCUFRIEND_kbv::setRotation(uint8_t r)
{
	uint16_t GS, SS_v, ORG, REV = _lcd_rev;
	uint8_t val, d[3];
	rotation = r & 3; // just perform the operation ourselves on the protected variables
	_width = (rotation & 1) ? HEIGHT : WIDTH;
	_height = (rotation & 1) ? WIDTH : HEIGHT;
	switch (rotation)
	{
	case 0:			//PORTRAIT:
		val = 0x48; //MY=0, MX=1, MV=0, ML=0, BGR=1
		break;
	case 1:			//LANDSCAPE: 90 degrees
		val = 0x28; //MY=0, MX=0, MV=1, ML=0, BGR=1
		break;
	case 2:			//PORTRAIT_REV: 180 degrees
		val = 0x98; //MY=1, MX=0, MV=0, ML=1, BGR=1
		break;
	case 3:			//LANDSCAPE_REV: 270 degrees
		val = 0xF8; //MY=1, MX=1, MV=1, ML=1, BGR=1
		break;
	}
	if constexpr (_lcd_capable & INVERT_GS)
		val ^= 0x80;
	if constexpr (_lcd_capable & INVERT_SS)
		val ^= 0x40;
	if constexpr (_lcd_capable & INVERT_RGB)
		val ^= 0x08;
	if constexpr (_lcd_capable & MIPI_DCS_REV1)
	{
	common_MC:
		_MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
		WriteCmdParamN(0x36, 1, &val);
		_lcd_madctl = val;
		//	    if (_lcd_ID	== 0x1963) WriteCmdParamN(0x13, 0, NULL);   //NORMAL mode
	}
	// cope with 9320 variants
	else
	{
		_MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
		GS = (val & 0x80) ? (1 << 15) : 0;
		WriteCmdData(0x60, GS | 0x2700); // Gate Scan Line (0xA700)
	common_SS:
		SS_v = (val & 0x40) ? (1 << 8) : 0;
		WriteCmdData(0x01, SS_v); // set Driver Output Control
	common_ORG:
		ORG = (val & 0x20) ? (1 << 3) : 0;
		if (val & 0x08)
			ORG |= 0x1000; //BGR
		_lcd_madctl = ORG | 0x0030;
		WriteCmdData(0x03, _lcd_madctl); // set GRAM write direction and BGR=1.
	}
	if ((rotation & 1) && ((_lcd_capable & MV_AXIS) == 0))
	{
		uint16_t x;
		x = _MC, _MC = _MP, _MP = x;
		x = _SC, _SC = _SP, _SP = x; //.kbv check 0139
		x = _EC, _EC = _EP, _EP = x; //.kbv check 0139
	}
	setAddrWindow(0, 0, width() - 1, height() - 1);
	vertScroll(0, HEIGHT, 0); //reset scrolling after a rotation
}

void MCUFRIEND_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	setAddrWindow(x, y, x, y);
	//    CS_ACTIVE; WriteCmd(_MW); write16(color); CS_IDLE; //-0.01s +98B

	WriteCmdData(_MW, color);
}

void MCUFRIEND_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	if constexpr (_lcd_capable & MIPI_DCS_REV1)
	{
		WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1); //Start column instead of _MC
		WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1); //
	}
	else
	{
		WriteCmdData(_MC, x);
		WriteCmdData(_MP, y);
		if (!(x == x1 && y == y1))
		{ //only need MC,MP for drawPixel
			if constexpr (_lcd_capable & XSA_XEA_16BIT)
			{
				if (rotation & 1)
					y1 = y = (y1 << 8) | y;
				else
					x1 = x = (x1 << 8) | x;
			}
			WriteCmdData(_SC, x);
			WriteCmdData(_SP, y);
			WriteCmdData(_EC, x1);
			WriteCmdData(_EP, y1);
		}
	}
}

void MCUFRIEND_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	int16_t end;
	end = x + w;
	if (end > width())
		end = width();
	w = end - x;
	end = y + h;
	if (end > height())
		end = height();
	h = end - y;
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	CS_ACTIVE;
	WriteCmd(_MW);
	if (h > w)
		std::swap(h, w);
		
	uint8_t hi = color >> 8, lo = color & 0xFF;
	for (; h > 0; --h)
	{
		end = w;
		do
		{
			write8(hi);
			write8(lo);
		} while (--end != 0);
	}
	CS_IDLE;
	if constexpr (!(_lcd_capable & MIPI_DCS_REV1))
		setAddrWindow(0, 0, width() - 1, height() - 1);
}

static void pushColors_any(uint16_t cmd, uint8_t *block, int16_t n, bool first, uint8_t flags)
{
	uint16_t color;
	uint8_t h, l;
	bool isconst = flags & 1;
	bool isbigend = (flags & 2) != 0;
	CS_ACTIVE;
	if (first)
	{
		WriteCmd(cmd);
	}

	if (!isconst && !isbigend)
	{
		uint16_t *block16 = (uint16_t *)block;
		for (; n > 0; --n)
		{
			color = *block16++;
			write16(color);
		}
	}
	else

		for (; n > 0; --n)
		{
			if (isconst)
			{
				h = pgm_read_byte(block++);
				l = pgm_read_byte(block++);
			}
			else
			{
				h = (*block++);
				l = (*block++);
			}
			color = (isbigend) ? (h << 8 | l) : (l << 8 | h);
			write16(color);
		}
	CS_IDLE;
}

void MCUFRIEND_kbv::pushColors(uint16_t *block, int16_t n, bool first)
{
	pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void MCUFRIEND_kbv::pushColors(uint8_t *block, int16_t n, bool first)
{
	pushColors_any(_MW, (uint8_t *)block, n, first, 2); //regular bigend
}
void MCUFRIEND_kbv::pushColors(const uint8_t *block, int16_t n, bool first, bool bigend)
{
	pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void MCUFRIEND_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327)
	{
		if (rotation == 2 || rotation == 3)
			top += OFFSET_9327;
	}
#endif
	int16_t bfa = HEIGHT - top - scrollines; // bottom fixed area
	int16_t vsp;
	int16_t sea = top;
	if (_lcd_ID == 0x9327)
		bfa += 32;
	if (offset <= -scrollines || offset >= scrollines)
		offset = 0;		//valid scroll
	vsp = top + offset; // vertical start position
	if (offset < 0)
		vsp += scrollines; //keep in unsigned range
	sea = top + scrollines - 1;
	if constexpr (_lcd_capable & MIPI_DCS_REV1)
	{
		uint8_t d[6];	// for multi-byte parameters
						 /*
        if (_lcd_ID == 0x9327) {        //panel is wired for 240x432 
            if (rotation == 2 || rotation == 3) { //180 or 270 degrees
                if (scrollines == HEIGHT) {
                    scrollines = 432;   // we get a glitch but hey-ho
                    vsp -= 432 - HEIGHT;
                }
                if (vsp < 0)
                    vsp += 432;
            }
            bfa = 432 - top - scrollines;
        }
*/
		d[0] = top >> 8; //TFA
		d[1] = top;
		d[2] = scrollines >> 8; //VSA
		d[3] = scrollines;
		d[4] = bfa >> 8; //BFA
		d[5] = bfa;
		WriteCmdParamN(0x33, 6, d);
		//        if (offset == 0 && rotation > 1) vsp = top + scrollines;   //make non-valid
		d[0] = vsp >> 8; //VSP
		d[1] = vsp;
		WriteCmdParamN(0x37, 2, d);
		if constexpr (_lcd_capable & MIPI_DCS_REV1)
		{
			if (offset == 0)
				WriteCmdParamN(0x13, 0, NULL); //NORMAL i.e. disable scroll
		}
		return;
	}
	// cope with 9320 style variants:
	// 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
	WriteCmdData(0x61, (1 << 1) | _lcd_rev); //!NDL, VLE, REV
	WriteCmdData(0x6A, vsp);				 //VL#
}

void MCUFRIEND_kbv::invertDisplay(boolean i)
{
	uint8_t val;
	_lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;
	if constexpr (_lcd_capable & MIPI_DCS_REV1)
	{
		WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
		return;
	}
	// cope with 9320 style variants:
	WriteCmdData(0x61, _lcd_rev);
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{
#ifdef SUPPORT_8357D_GAMMA
	uint8_t *p = (uint8_t *)table, dat[36]; //HX8357_99 has GAMMA[34]
#else
	auto *p = (const uint8_t *)table;
	uint8_t dat[24]; //R61526 has GAMMA[22]
#endif
	while (size > 0)
	{
		uint8_t cmd = pgm_read_byte(p++);
		uint8_t len = pgm_read_byte(p++);
		if (cmd == TFTLCD_DELAY8)
		{
			delay(len);
			len = 0;
		}
		else
		{
			for (uint8_t i = 0; i < len; i++)
				dat[i] = pgm_read_byte(p++);
			WriteCmdParamN(cmd, len, dat);
		}
		size -= len + 2;
	}
}

void MCUFRIEND_kbv::begin(uint16_t ID)
{
	const uint8_t *table8_ads = NULL;
	int16_t table_size;
	reset();
	_lcd_xor = 0;
	_lcd_ID = ID;
	
	
	static const uint8_t ILI9341_regValues_2_4[] PROGMEM = {
		// BOE 2.4"
		0xF6,
		3,
		0x01,
		0x01,
		0x00, //Interface Control needs EXTC=1 MV_EOR=0, TM=0, RIM=0
		0xCF,
		3,
		0x00,
		0x81,
		0x30, //Power Control B [00 81 30]
		0xED,
		4,
		0x64,
		0x03,
		0x12,
		0x81, //Power On Seq [55 01 23 01]
		0xE8,
		3,
		0x85,
		0x10,
		0x78, //Driver Timing A [04 11 7A]
		0xCB,
		5,
		0x39,
		0x2C,
		0x00,
		0x34,
		0x02, //Power Control A [39 2C 00 34 02]
		0xF7,
		1,
		0x20, //Pump Ratio [10]
		0xEA,
		2,
		0x00,
		0x00, //Driver Timing B [66 00]
		0xB0,
		1,
		0x00, //RGB Signal [00]
		0xB1,
		2,
		0x00,
		0x1B, //Frame Control [00 1B]
		//            0xB6, 2, 0x0A, 0xA2, 0x27, //Display Function [0A 82 27 XX]    .kbv SS=1
		0xB4,
		1,
		0x00, //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
		0xC0,
		1,
		0x21, //Power Control 1 [26]
		0xC1,
		1,
		0x11, //Power Control 2 [00]
		0xC5,
		2,
		0x3F,
		0x3C, //VCOM 1 [31 3C]
		0xC7,
		1,
		0xB5, //VCOM 2 [C0]
		0x36,
		1,
		0x48, //Memory Access [00]
		0xF2,
		1,
		0x00, //Enable 3G [02]
		0x26,
		1,
		0x01, //Gamma Set [01]
		0xE0,
		15,
		0x0f,
		0x26,
		0x24,
		0x0b,
		0x0e,
		0x09,
		0x54,
		0xa8,
		0x46,
		0x0c,
		0x17,
		0x09,
		0x0f,
		0x07,
		0x00,
		0xE1,
		15,
		0x00,
		0x19,
		0x1b,
		0x04,
		0x10,
		0x07,
		0x2a,
		0x47,
		0x39,
		0x03,
		0x06,
		0x06,
		0x30,
		0x38,
		0x0f,
	};

	table8_ads = ILI9341_regValues_2_4, table_size = sizeof(ILI9341_regValues_2_4); //
		
	_lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);

	static const uint8_t reset_off[] PROGMEM = {
		0x01, 0,			//Soft Reset
		TFTLCD_DELAY8, 150, // .kbv will power up with ONLY reset, sleep out, display on
		0x28, 0,			//Display Off
		0x3A, 1, 0x55,		//Pixel read=565, write=565.
	};
	static const uint8_t wake_on[] PROGMEM = {
		0x11, 0, //Sleep Out
		TFTLCD_DELAY8, 150,
		0x29, 0, //Display On
	};
	init_table(&reset_off, sizeof(reset_off));
	init_table(table8_ads, table_size); //can change PIXFMT
	init_table(&wake_on, sizeof(wake_on));

	setRotation(0); //PORTRAIT
	invertDisplay(false);
}
