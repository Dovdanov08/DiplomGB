#include "stm32f4xx_hal.h"
#include "st7735.h"
#include "malloc.h"

int32_t Ymax,Ymin,X;        // X goes from 0 to 127
int32_t Yrange; //YrangeDiv2;
int TimeIndex;
uint16_t PlotBGColor;

static SPI_HandleTypeDef *hspi=0;

void lcd7735_Sendbyte(uint8_t byte) {

	uint8_t send_flag=1;

	while(send_flag)										// ждем пока флаг не установится в нуль для продолжения работы
	{
		if(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
		{
			*((__IO uint8_t *)&hspi->Instance->DR) = byte;
			send_flag=0;
		}
	}
	__HAL_SPI_CLEAR_OVRFLAG(hspi);

}
/*------Функции для передачи данных или команд-------*/
void lcd7735_sendData(uint8_t data)
{
   LCD_DC1;//Set DC HIGH								//
   lcd7735_Sendbyte(data);
}
void lcd7735_sendCmd(uint8_t cmd)
{
   LCD_DC0; //Set DC low
   lcd7735_Sendbyte(cmd);
}
void ST7735_TxData(uint8_t *data, uint16_t size)
{
	LCD_DC1;
	for(int i=0; i<size; i++)
	{
		lcd7735_Sendbyte(*data);
		data++;
	}
}
void ST7735_TxData1(uint8_t *data, uint16_t size)
{
	hspi->TxXferCount=size;
   while (hspi->TxXferCount > 0U)
   {
		/* Wait until TXE flag is set to send data */
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
		{
			*((__IO uint8_t *)&hspi->Instance->DR) = *data;
			data++;
			hspi->pTxBuffPtr += sizeof(uint8_t);
			hspi->TxXferCount--;
		}
	}
	//__HAL_SPI_CLEAR_OVRFLAG(hspi);
}

/*------------------------------------------------------*/


/**
  *@brief  Set the working area of ​​the screen  (установка рабочей зоны экрана)
	*@input  initial x,y
	*@retval None
	*/
 
static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)	// задаём рабочую зону экрана
{
    lcd7735_sendCmd(ST7735_CASET); // Column addr set
	lcd7735_sendData(0x00);            // XS15 ~ XS8
	lcd7735_sendData(x0+ST7735_XSTART);     // XSTART       XS7 ~ XS0
	lcd7735_sendData(0x00);            // XE15 ~ XE8
	lcd7735_sendData(x1+ST7735_XSTART);     // XEND         XE7 ~ XE0
 
	lcd7735_sendCmd(ST7735_RASET); // Row addr set
	lcd7735_sendData(0x00);
	lcd7735_sendData(y0+ST7735_YSTART);     // YSTART
	lcd7735_sendData(0x00);
	lcd7735_sendData(y1+ST7735_YSTART);     // YEND
	lcd7735_sendCmd(ST7735_RAMWR);
}
void ST7735_Init(SPI_HandleTypeDef *hspi_ext)
{
	hspi=hspi_ext;
	__HAL_SPI_ENABLE(hspi);					//Для работы через регистры а не HAL фукнции

	LCD_CS0;
	LCD_RST0;
	HAL_Delay(7);
	LCD_RST1;
	ST7735_Init_Command1();
	ST7735_Init_Command2();
	ST7735_Init_Command3();
	LCD_CS1;
}
 
void ST7735_Init_Command1(void)
{
	lcd7735_sendCmd(ST7735_SWRESET);		//  1: Software reset
	HAL_Delay(150);
	lcd7735_sendCmd(ST7735_SLPOUT);			//  2: Out of sleep mode
	HAL_Delay(500);
	lcd7735_sendCmd(ST7735_FRMCTR1);		//  3: Frame rate ctrl - normal mode
	lcd7735_sendData(0x01);							//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	lcd7735_sendData(0x2C);
	lcd7735_sendData(0x2D);
	lcd7735_sendCmd(ST7735_FRMCTR2);		//  4: Frame rate control - idle mode
	lcd7735_sendData(0x01);							//  Rate = fosc/(1x2+40) * (LINE+2C+2D)
	lcd7735_sendData(0x2C);
	lcd7735_sendData(0x2D);
	lcd7735_sendCmd(ST7735_FRMCTR3);		//  5: Frame rate ctrl - partial mode
	lcd7735_sendData(0x01);							//     Dot inversion mode
	lcd7735_sendData(0x2C);
	lcd7735_sendData(0x2D);
	lcd7735_sendData(0x01);							//     Line inversion mode
	lcd7735_sendData(0x2C);
	lcd7735_sendData(0x2D);
	lcd7735_sendCmd(ST7735_INVCTR);			//  6: Display inversion ctrl
	lcd7735_sendData(0x07);							//     No inversion
	lcd7735_sendCmd(ST7735_PWCTR1);			//  7: Power control
	lcd7735_sendData(0xA2);
	lcd7735_sendData(0x02);							//     -4.6V
	lcd7735_sendData(0x84);							//     AUTO mode
	lcd7735_sendCmd(ST7735_PWCTR2);			//  8: Power control
	lcd7735_sendData(0xC5);							//     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	lcd7735_sendCmd(ST7735_PWCTR3);			//  9: Power control
	lcd7735_sendData(0x0A);							//     Opamp current small
	lcd7735_sendData(0x00);							//     Boost frequency
	lcd7735_sendCmd(ST7735_PWCTR4);			// 10: Power control
	lcd7735_sendData(0x8A);							//     BCLK/2, Opamp current small & Medium low
	lcd7735_sendData(0x2A);
	lcd7735_sendCmd(ST7735_PWCTR5);			// 11: Power control
	lcd7735_sendData(0x8A);
	lcd7735_sendData(0xEE);
	lcd7735_sendCmd(ST7735_VMCTR1);			// 12: Power control
	lcd7735_sendData(0x0E);
	lcd7735_sendCmd(ST7735_INVOFF);			// 13: Don't invert display
	lcd7735_sendCmd(ST7735_MADCTL);			// 14: Memory access control (directions)
	lcd7735_sendData(ST7735_ROTATION);	//     row addr/col addr, bottom to top refresh
	lcd7735_sendCmd(ST7735_COLMOD);			// 15: set color mode
	lcd7735_sendData(0x05);							//     16-bit color
}
 
void ST7735_Init_Command2(void)
{
	lcd7735_sendCmd(ST7735_CASET);			//  1: Column addr set
	lcd7735_sendData(0x00);							//     XSTART = 0
	lcd7735_sendData(0x00);
	lcd7735_sendData(0x00);							//     XEND = 127
	lcd7735_sendData(0x7F);
	lcd7735_sendCmd(ST7735_RASET);			//  2: Row addr set
	lcd7735_sendData(0x00);							//     XSTART = 0
	lcd7735_sendData(0x00);
	lcd7735_sendData(0x00);							//     XEND = 127
	lcd7735_sendData(0x7F);
}
 
void ST7735_Init_Command3(void)
{
	lcd7735_sendCmd(ST7735_GMCTRP1);  //  1: Magical unicorn dust
	lcd7735_sendData(0x02);
	lcd7735_sendData(0x1C);
	lcd7735_sendData(0x07);
	lcd7735_sendData(0x12);
	lcd7735_sendData(0x37);
	lcd7735_sendData(0x32);
	lcd7735_sendData(0x29);
	lcd7735_sendData(0x2D);
	lcd7735_sendData(0x29);
	lcd7735_sendData(0x25);
	lcd7735_sendData(0x2B);
	lcd7735_sendData(0x39);
	lcd7735_sendData(0x00);
	lcd7735_sendData(0x01);
	lcd7735_sendData(0x03);
	lcd7735_sendData(0x10);
	lcd7735_sendCmd(ST7735_GMCTRN1);  //  2: Sparkles and rainbows
	lcd7735_sendData(0x03);
	lcd7735_sendData(0x1D);
	lcd7735_sendData(0x07);
	lcd7735_sendData(0x06);
	lcd7735_sendData(0x2E);
	lcd7735_sendData(0x2C);
	lcd7735_sendData(0x29);
	lcd7735_sendData(0x2D);
	lcd7735_sendData(0x2E);
	lcd7735_sendData(0x2E);
	lcd7735_sendData(0x37);
	lcd7735_sendData(0x3F);
	lcd7735_sendData(0x00);
	lcd7735_sendData(0x00);
	lcd7735_sendData(0x02);
	lcd7735_sendData(0x10);
	lcd7735_sendCmd(ST7735_NORON);
	HAL_Delay(10);
	lcd7735_sendCmd(ST7735_DISPON);
	HAL_Delay(100);
}



void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    uint8_t data[2];
    uint16_t width = w, height=h;

    data[0]=color>>8;
    data[1]=color&0xFF;
    LCD_CS0;

    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

    for(int i=0; i<(width*height); i++ )
    {
    	ST7735_TxData(data,2);
    }

	LCD_CS1;  //Unselect
}
void ST7735_FillRectangleFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    LCD_CS0;
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

    // Prepare whole line in a single buffer
    uint8_t pixel[] = { color >> 8, color & 0xFF };
    uint8_t *line = malloc(w * sizeof(pixel));
    for(x = 0; x < w; ++x)
    	memcpy(line + x * sizeof(pixel), pixel, sizeof(pixel));

    LCD_DC1;
    for(y = h; y > 0; y--)
    {
    	ST7735_TxData1(line,w * sizeof(pixel));
    }
    free(line);
    LCD_CS0;
}


void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}
void ST7735_FillScreenFast(uint16_t color)
{
    ST7735_FillRectangleFast(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;
    uint8_t data[2];

    LCD_CS0;
    ST7735_SetAddressWindow(x, y, x+1, y+1);					//установка рабочей зоны
    data[0] = color >> 8;
	data[1] = color & 0xFF;
	ST7735_TxData(data,2);
    LCD_CS1;  //unselect
}
void ST7735_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color)		//вертикальная линия
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;
    uint8_t data[2];

    LCD_CS0;
    ST7735_SetAddressWindow(x, y, x, y+h);
    data[0]=color >>8;
    data[1]=color & 0xFF;
    for(int i=0;i<h; i++)
    	ST7735_TxData(data,2);
    LCD_CS1;
}
void ST7735_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color)			// горизонтальная линия
{
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;
    uint8_t data[2];

    LCD_CS0;
    ST7735_SetAddressWindow(x, y, x+w, y);
    data[0]=color >>8;
    data[1]=color & 0xFF;
    for(int i=0;i<w; i++)
    	ST7735_TxData(data,2);
    LCD_CS1;
}



void ST7735_ProgressBar(uint16_t x, uint16_t y,uint16_t h, uint16_t data, uint16_t color)
{
	uint16_t level=0 , w=100;

    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    level=data;

	LCD_CS0;
	ST7735_DrawFastVLine(x,y,h, ST7735_WHITE);
	ST7735_DrawFastVLine(x+w,y,h, ST7735_WHITE);

	ST7735_DrawFastHLine(x,y,w, ST7735_WHITE);
	ST7735_DrawFastHLine(x,y+h,w, ST7735_WHITE);

	ST7735_FillRectangleFast(x+1,y+1,99,h-1,ST7735_BLACK);
	ST7735_FillRectangleFast(x+1,y+1,level,h-1, color);

	LCD_CS1;
}
void ST7735_DrawSymbol(uint16_t x, uint16_t y, uint16_t font, uint16_t color, uint8_t ch)
{
	if(((x+font) >= ST7735_WIDTH) || ((y+font) >= ST7735_HEIGHT))
		return;
	uint8_t data[2];

	ST7735_SetAddressWindow(x, y, x+7, y+10);

	data[0]=color>>8;
	data[1]=color&0xFF;
	for(int i=0;i< (7*10); i++)
	{
		ST7735_TxData(data,2);
	}
}



uint32_t ST7735_DrawString(uint16_t x, uint16_t y, char *pt, int16_t textColor, uint8_t size){
  uint32_t count = 0;
  if(y>15) return 0;
  while(*pt){
    ST7735_DrawCharS(x*6, y*10, *pt, textColor, ST7735_BLACK, size);
    pt++;
    x = x+1;
    if(x>20) return count;  // number of characters printed
    count++;
  }
  return count;  // number of characters printed
}

void ST7735_DrawCharS(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; 
  int32_t i, j;
  if((x >= ST7735_WIDTH)            || 
     (y >= ST7735_HEIGHT)           || 
     ((x + 5 * size - 1) < 0) || 
     ((y + 8 * size - 1) < 0))   
    return;
 
  for (i=0; i<6; i++ ) {
    if (i == 5)
      line = 0x0;
    else
      line = Font[(c*5)+i];
    for (j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) 
          ST7735_DrawPixel(x+i, y+j, textColor);
        else {  
          ST7735_FillRectangle(x+(i*size), y+(j*size), size, size, textColor);
        }
      } else if (bgColor != textColor) {
        if (size == 1) // default size
          ST7735_DrawPixel(x+i, y+j, bgColor);
        else {  // big size
         ST7735_FillRectangle(x+i*size, y+j*size, size, size, bgColor);
        }
      }
      line >>= 1;
    }
  }
}
void ST7735_Graphic(uint16_t datax,uint16_t datay, uint16_t colorGr, uint16_t cololBg)
{
	uint16_t dataxarray[ST7735_WIDTH];
	uint16_t datayarray[ST7735_HEIGHT];
	uint16_t i=0;

	for( i=0; i<(ST7735_WIDTH);i++)
		dataxarray[i]=0;

	for( i=0; i<(ST7735_HEIGHT);i++)
		datayarray[i]=0;
	dataxarray[45]= datax;
	datayarray[45]= datay;

	for( i=0; i<(ST7735_WIDTH*ST7735_HEIGHT);i++)
		ST7735_DrawPixel(dataxarray[i], datayarray[i], colorGr);

}

void ST7735_Drawaxes(uint16_t axisColor, uint16_t bgColor, char *xLabel,char *yLabel1, uint16_t label1Color, char *yLabel2, uint16_t label2Color,int32_t ymax, int32_t ymin)
{
  int i;

  Ymax = ymax;
  Ymin = ymin;
  Yrange = Ymax - Ymin;
  TimeIndex = 0;
  PlotBGColor = bgColor;
	LCD_CS0;
  ST7735_FillRectangleFast(0, 0, 128, 160, bgColor);
  ST7735_DrawFastHLine(10, 140, 101, axisColor);
  ST7735_DrawFastVLine(10, 17, 124, axisColor);
  for(i=20; i<=110; i=i+10){
    ST7735_DrawPixel(i, 141, axisColor);
  }
  for(i=17; i<120; i=i+10){
    ST7735_DrawPixel(0, i, axisColor);
  }
  i = 50;
  //while((*xLabel) && (i < 100)){
    ST7735_DrawCharS(12, 145, *xLabel, axisColor, bgColor, 1);
    //i = i + 6;
    //xLabel++;
  //}
  if(*yLabel2){ // two labels
    i = 26;
    while((*yLabel2) && (i < 50)){
      ST7735_DrawCharS(1, i, *yLabel2, label2Color, bgColor, 1);
      i = i + 8;
      yLabel2++;
    }
    i = 82;
  }else{ // one label
    i = 42;
  }
  while((*yLabel1) && (i < 120)){
   ST7735_DrawCharS(2, i, *yLabel1, label1Color, bgColor, 1);
    i = i + 8;
    yLabel1++;
  }
	LCD_CS1;
}
	



void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
int16_t skipC = 0;                      
  int16_t originalWidth = w;              
  int i = w*(h - 1);
 
  if((x >= ST7735_WIDTH) || ((y - h + 1) >= ST7735_HEIGHT) || ((x + w) <= 0) || (y < 0)){
    return;                             
  }
  if((w > ST7735_WIDTH) || (h > ST7735_HEIGHT)){    
   
    return;
  }
  if((x + w - 1) >= ST7735_WIDTH){            
    skipC = (x + w) - ST7735_WIDTH;           
    w = ST7735_WIDTH - x;
  }
  if((y - h + 1) < 0){                  
    i = i - (h - y - 1)*originalWidth;  
    h = y + 1;
  }
  if(x < 0){                            
    w = w + x;
    skipC = -1*x;                       
    i = i - x;                          
    x = 0;
  }
  if(y >= ST7735_HEIGHT){                     
    h = h - (y - ST7735_HEIGHT + 1);
    y = ST7735_HEIGHT - 1;
  }
	
	LCD_CS0;  //Select
 
  ST7735_SetAddressWindow(x, y-h+1, x+w-1, y);
 
  for(y=0; y<h; y=y+1){
    for(x=0; x<w; x=x+1){
                                        
      lcd7735_sendData((uint8_t)(data[i] >> 8));
                                        
      lcd7735_sendData((uint8_t)data[i]);
      i = i + 1;                       
    }
    i = i + skipC;
    i = i - 2*originalWidth;
  }
	LCD_CS1;  //Unselect
}

void ST7735_InvertColors(bool invert) {
  //ST7735_Select();
	LCD_CS0;
  //ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
	lcd7735_sendCmd(invert ? ST7735_INVON : ST7735_INVOFF);
	LCD_CS1;  //Unselect
}
 

