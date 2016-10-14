// пины порта к которым подключен дисплей
#define SCLK GPIO_PIN_4//GPIO_Pin_0
#define MOSI GPIO_PIN_5//GPIO_Pin_1
#define DC   GPIO_PIN_6//GPIO_Pin_2
#define RST  GPIO_PIN_8//GPIO_Pin_3


#define LCD_PORT GPIOB	// порт к которому подключен дисплей

#define LCD_PH RCC_APB2Periph_GPIOB // шина к которой подключена дисплей

// Управление линией LCD_RST
#define LCD_RST1  HAL_GPIO_WritePin(LCD_PORT, RST, GPIO_PIN_SET);//GPIO_SetBits  (LCD_PORT, RST)
#define LCD_RST0  HAL_GPIO_WritePin(LCD_PORT, RST, GPIO_PIN_RESET);//GPIO_ResetBits(LCD_PORT, RST)
// Управление линией LCD_DC
#define LCD_DC1   HAL_GPIO_WritePin(LCD_PORT, DC, GPIO_PIN_SET);//GPIO_SetBits  (LCD_PORT, DC)
#define LCD_DC0   HAL_GPIO_WritePin(LCD_PORT, DC, GPIO_PIN_RESET);//GPIO_ResetBits(LCD_PORT, DC)
// Управление линией LCD_SCK
#define LCD_SCK1   HAL_GPIO_WritePin(LCD_PORT, SCLK, GPIO_PIN_SET);//GPIO_SetBits  (LCD_PORT, SCLK)
#define LCD_SCK0   HAL_GPIO_WritePin(LCD_PORT, SCLK, GPIO_PIN_RESET);//GPIO_ResetBits(LCD_PORT, SCLK)
// Управление линией LCD_MOSI
#define LCD_MOSI1   HAL_GPIO_WritePin(LCD_PORT, MOSI, GPIO_PIN_SET);//GPIO_SetBits  (LCD_PORT, MOSI)
#define LCD_MOSI0   HAL_GPIO_WritePin(LCD_PORT, MOSI, GPIO_PIN_RESET);//GPIO_ResetBits(LCD_PORT, MOSI)

extern	SPI_HandleTypeDef hspi1;

void lcd8544_init(void);  // инициалиазация дисплея

void lcd8544_refresh(void); // обновление экрана из буфера

void lcd8544_putpix(unsigned char x, unsigned char y, unsigned char mode); // вывод пиксела

void lcd8544_line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char mode); // вывод линии

void lcd8544_rect(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, unsigned char mode); // прямоугольник

void lcd8544_putchar(unsigned char px, unsigned char py, unsigned char ch, unsigned char mode); //  вывод символа

void lcd8544_dec(unsigned int numb, unsigned char dcount, unsigned char x, unsigned char y, unsigned char mode); // вывод десятичного числа

void lcd8544_putstr(unsigned char x, unsigned char y, const unsigned char str[], unsigned char mode); // вывод строки


