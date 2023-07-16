#include "../system/include/cmsis/stm32f4xx.h"

int16_t AXES[2] = {0, 0};

// Регистры для LIS302DL
#define LIS302DL_ADDR     (0x3B)
#define WHO_AM_I          (0x0F)
#define CTRL_REG1         (0x20)
#define CTRL_REG2         (0x21)
#define CTRL_REG3         (0x22)
#define HP_FILTER_RESET   (0x23)
#define STATUS_REG        (0x27)
#define OUT_X             (0x29)
#define OUT_Y             (0x2B)
#define OUT_Z             (0x2D)


#define X_OFFSET 18

void MyDelay(uint32_t time) {
	for (uint32_t t = 0; t < time; t++);
}

void GPIO_init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER |= GPIO_MODER_MODER3_0;
	GPIOE->ODR |= GPIO_ODR_OD3;
}

void SPI1_init() {
	//// GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
	GPIOA->AFR[0] |= ((GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0) |
					  (GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0) |
					  (GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0));
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);

	//// SPI
	// 0. Включаем тактирование
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// 1. Настраиваем на Master Mode
	SPI1->CR1 |= SPI_CR1_MSTR;

	// 2. Отключаем
	// однонаправленный режим работы по одной линии данных
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	// выход запрещен (режим "только прием"), теперь работает одновременные передача и прием
	SPI1->CR1 &= ~SPI_CR1_RXONLY;

	// 3. Настраиваем прием на 8 бит
	SPI1->CR1 &= ~SPI_CR1_DFF;

	// 4. Настройка управления slave устройством
	SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);

	// 5. Управление скоростью передачи 000: fPCLK/2
	SPI1->CR1 &= ~SPI_CR1_BR;

	// 6. Установка передачи в первый режим MSB
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

	// 7. Настройка полярности тактов(CPOL) и фазы тактов(CPHA)
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 &= ~SPI_CR1_CPOL;

	// 8. Включить CRC
	SPI1->CR1 |= SPI_CR1_CRCEN;

	// 9. Выбор формата Motorola, какой то внутренний, пока не понятно что это
	SPI1->CR2 = 0x0000;

	// 10. Включаем SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

uint16_t SPI1_write(uint8_t data) {
	uint16_t buffer = 0;
	while(!((SPI1->SR) & SPI_SR_TXE)){}
	SPI1->DR = data;
	while(!(SPI1->SR & SPI_SR_RXNE)){}
	buffer = SPI1->DR;  // После того как произвели запись надо считать значение из SPIx->DR
	return buffer;
}

uint16_t SPI1_read(uint8_t address) {
	uint16_t data = 0;
	data = SPI1_write(address);
	data = SPI1_write(0x00);
	return data;
}

void LIS302DL_write(uint8_t address, uint8_t data) {
	GPIOE->ODR &= ~GPIO_ODR_OD3;
	SPI1_write(address);
	SPI1_write(data);
	GPIOE->ODR |= GPIO_ODR_OD3;
}

uint8_t LIS302DL_read(uint8_t address) {
	uint8_t data = 0;
	GPIOE->ODR &= ~GPIO_ODR_ODR_3;
	address |= 0x80;
	data = SPI1_read(address);
	GPIOE->ODR |= GPIO_ODR_ODR_3;
	return data;
}

void LIS302DL_init() {
	GPIO_init();
	SPI1_init();
	LIS302DL_write(CTRL_REG1, 0x47);
	MyDelay(1000);
}

int16_t Convert_To_Val(uint16_t val){
	if ((val & 0x80) == 0x80){
		val = ~val;
		++val;
		val = -(((val & 0x00FF) * 2300) / 127);
		return val;
	}
	else
		return (( val * 2300 ) / 127);
}

int16_t getX() {
	uint16_t X_sensor = 0;
	int16_t X_result = 0;
	X_sensor = LIS302DL_read(OUT_X);
	X_result = Convert_To_Val(X_sensor) + X_OFFSET;
	return X_result;
}

int16_t getY() {
	uint16_t Y_sensor = 0;
	int16_t Y_result = 0;
	Y_sensor = LIS302DL_read(OUT_Y);
	Y_result = Convert_To_Val(Y_sensor);
	return Y_result;
}

int16_t getZ() {
	uint16_t Z_sensor = 0;
	int16_t Z_result = 0;
	Z_sensor = LIS302DL_read(OUT_Z);
	Z_result = Convert_To_Val(Z_sensor);
	return Z_result;
}

#define S 0     // длительности импульса в циклах, по умолчанию

void TIM4_IRQHandler(void) {
	TIM4->SR &= ~TIM_SR_UIF;
	TIM4->CR1 &= ~TIM_CR1_CEN;

    if (AXES[0] < 0) {
        TIM4->CCR2 = (-AXES[0]);
        if (TIM4->CCR4 != 0)
            --TIM4->CCR4;
        else
            TIM4->CCR4 = 0;
    } else {
        TIM4->CCR4 = AXES[0];
        if (TIM4->CCR2 != 0)
            --TIM4->CCR2;
        else
            TIM4->CCR2 = 0;
    }

    if (AXES[1] < 0) {
        TIM4->CCR1 = (-AXES[1]);
        if (TIM4->CCR3 != 0)
            --TIM4->CCR3;
        else
            TIM4->CCR3 = 0;
    } else {
        TIM4->CCR3 = AXES[1];
        if (TIM4->CCR1 != 0)
            --TIM4->CCR1;
        else
            TIM4->CCR1 = 0;
    }
       
    // 3. Запускаем таймер
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM4_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
    GPIOD->AFR[1] |= ((0x2 << 16) | (0x2 << 20) | (0x2 << 24) | (0x2 << 28)); 
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 3000;
    TIM4->ARR = 100;
    
    // Тут включается PWM1,2,3,4, строка OC1M, OC2M, OC3M, OC4M
    TIM4->CCMR1 |= 0x6060;
    TIM4->CCMR2 |= 0x6060;

    // Установка длительности импульса в циклах
    TIM4->CCR1 = S;
    TIM4->CCR2 = S;
    TIM4->CCR2 = S;
    TIM4->CCR3 = S;
    
    // Включение CC1,2,3,4
    TIM4->CCER |= 0x1111;//(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    
    TIM4->DIER |= TIM_DIER_UIE;    
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 2);
    TIM4->CR1 |= TIM_CR1_CEN;
}

void updateAXES() {
    AXES[0] = (getX() / 2);
    AXES[1] = (getY() / 2);
}

int main() {
	LIS302DL_init();
	MyDelay(10000);
    TIM4_init();
	while (1) {
        updateAXES();	
    }
}


