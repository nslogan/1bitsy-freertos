
#include "stm32f415xx.h"

#include "FreeRTOS.h"
#include "task.h"

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)
	#define PERIPH_COUNT_IRQn	82
#else
	#error "PERIPH_COUNT_IRQn not defined for device"
#endif

int main( void );

// Type of an interrupt function. Simplifies vector table structure.
typedef void (*vector_table_entry_t)(void);

typedef struct {
	// ARMv7-M Core Exceptions (ARMv7-M ARM Table B1-4)
	uint32_t *initial_sp_value; /**< Initial stack pointer value. */
	vector_table_entry_t reset;
	vector_table_entry_t nmi;
	vector_table_entry_t hard_fault;
	vector_table_entry_t memory_manage_fault;
	vector_table_entry_t bus_fault;
	vector_table_entry_t usage_fault;
	vector_table_entry_t reserved_1;
	vector_table_entry_t reserved_2;
	vector_table_entry_t reserved_3;
	vector_table_entry_t reserved_4;
	vector_table_entry_t sv_call;
	vector_table_entry_t debug_monitor;
	vector_table_entry_t reserved_5;
	vector_table_entry_t pend_sv;
	vector_table_entry_t systick;
	// Peripheral Interrupts
	vector_table_entry_t irq[PERIPH_COUNT_IRQn];
} vector_table_t;

int main( void );

// void Reset_Handler      ( void );
// void NMI_Handler        ( void );
// void HardFault_Handler  ( void );
// void MemManage_Handler  ( void );
// void BusFault_Handler   ( void );
// void UsageFault_Handler ( void );
void SVC_Handler        ( void );
// void DebugMon_Handler   ( void );
void PendSV_Handler     ( void );
void SysTick_Handler    ( void );

void blocking_handler( void )
{
	while (1);
}

void null_handler( void )
{
	/* Do nothing. */
}

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

void reset_handler(void)
{
	uint32_t *pSrc, *pDest;

	/* Initialize the relocate segment */
	pSrc = &_etext;
	pDest = &_srelocate;

	if ( pSrc != pDest ) {
		for (; pDest < &_erelocate; ) {
			*pDest++ = *pSrc++;
		}
	}

	/* Clear the zero segment */
	for ( pDest = &_szero; pDest < &_ezero; ) {
		*pDest++ = 0;
	}

	/* Set the vector table base address */
	pSrc = (uint32_t *) & _sfixed;
	SCB->VTOR = ((uint32_t) pSrc & SCB_VTOR_TBLOFF_Msk);

	// 	LowLevelInit();
	// 	/* Initialize the C library */
	// 	__libc_init_array();

	/* Branch to main function */
	main();

	/* Infinite loop */
	while (1);
}

__attribute__((section(".vectors")))
vector_table_t vector_table = {
	.initial_sp_value = &_estack, // From linker script
	.reset = reset_handler,
	// .nmi = nmi_handler,
	.nmi = null_handler,
	// .hard_fault = hard_fault_handler,
	.hard_fault = blocking_handler,
	// .memory_manage_fault = mem_manage_handler,
	.memory_manage_fault = blocking_handler,
	// .bus_fault = bus_fault_handler,
	.bus_fault = blocking_handler,
	// .usage_fault = usage_fault_handler,
	.usage_fault = blocking_handler,
	// .debug_monitor = debug_monitor_handler,
	.debug_monitor = null_handler,
	// .sv_call = null_handler,
	// .sv_call = blocking_handler,
	.sv_call = SVC_Handler, // FreeRTOS
	// .pend_sv = null_handler,
	// .pend_sv = blocking_handler,
	.pend_sv = PendSV_Handler, // FreeRTOS
	// .systick = null_handler,
	// .systick = blocking_handler,
	.systick = SysTick_Handler, // FreeRTOS

	// Peripheral interrupts
	.irq = {
		// [<IRQn_Type::*_IRQn>] = <handler>
	}
};

#define GPIO0		(1 << 0)
#define GPIO1		(1 << 1)
#define GPIO2		(1 << 2)
#define GPIO3		(1 << 3)
#define GPIO4		(1 << 4)
#define GPIO5		(1 << 5)
#define GPIO6		(1 << 6)
#define GPIO7		(1 << 7)
#define GPIO8		(1 << 8)
#define GPIO9		(1 << 9)
#define GPIO10		(1 << 10)
#define GPIO11		(1 << 11)
#define GPIO12		(1 << 12)
#define GPIO13		(1 << 13)
#define GPIO14		(1 << 14)
#define GPIO15		(1 << 15)

#define GPIO_MODE(n, mode)		((mode) << (2 * (n)))
#define GPIO_MODE_MASK(n)		(0x3 << (2 * (n)))
#define GPIO_MODE_INPUT			0x0
#define GPIO_MODE_OUTPUT		0x1
#define GPIO_MODE_AF			0x2
#define GPIO_MODE_ANALOG		0x3

#define GPIO_OTYPE_PP			0x0 // Push Pull
#define GPIO_OTYPE_OD			0x1 // Open Drain

#define GPIO_OSPEED(n, speed)		((speed) << (2 * (n)))
#define GPIO_OSPEED_MASK(n)		(0x3 << (2 * (n)))
#define GPIO_OSPEED_2MHZ		0x0
#define GPIO_OSPEED_25MHZ		0x1
#define GPIO_OSPEED_50MHZ		0x2
#define GPIO_OSPEED_100MHZ		0x3

#define GPIO_PUPD(n, pupd)		((pupd) << (2 * (n)))
#define GPIO_PUPD_MASK(n)		(0x3 << (2 * (n)))
#define GPIO_PUPD_NONE			0x0
#define GPIO_PUPD_PULLUP		0x1
#define GPIO_PUPD_PULLDOWN		0x2

#define GPIO_LCKK			(1 << 16)

#define GPIO_AFR(n, af)			((af) << ((n) * 4))
#define GPIO_AFR_MASK(n)		(0xf << ((n) * 4))
#define GPIO_AF0			0x0
#define GPIO_AF1			0x1
#define GPIO_AF2			0x2
#define GPIO_AF3			0x3
#define GPIO_AF4			0x4
#define GPIO_AF5			0x5
#define GPIO_AF6			0x6
#define GPIO_AF7			0x7
#define GPIO_AF8			0x8
#define GPIO_AF9			0x9
#define GPIO_AF10			0xa
#define GPIO_AF11			0xb
#define GPIO_AF12			0xc
#define GPIO_AF13			0xd
#define GPIO_AF14			0xe
#define GPIO_AF15			0xf

void gpio_set( GPIO_TypeDef * gpio, uint16_t gpios );

void gpio_clear( GPIO_TypeDef * gpio, uint16_t gpios );

void gpio_toggle( GPIO_TypeDef * gpio, uint16_t gpios );

void gpio_mode_setup( GPIO_TypeDef * gpio, uint8_t mode, uint8_t pull_up_down, uint16_t gpios );

void gpio_set_output_options( GPIO_TypeDef * gpio, uint8_t otype, uint8_t speed, uint16_t gpios );

// void gpio_set_af(GPIO_TypeDef * gpio, uint8_t alt_func_num, uint16_t gpios);

void task_blink( void * arg )
{
	(void)arg;

	TickType_t xNextWakeTime = xTaskGetTickCount();

	while (1) {
		// Flip the LED
		gpio_toggle( GPIOA, GPIO8 );

		vTaskDelayUntil( &xNextWakeTime, pdMS_TO_TICKS( 500 ) );
	}
}

int main()
{
	// -- System initialization -- //

	// Enable GPIO clock (GPIOA)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// Set GPIO for LED as output (PA8)
	gpio_mode_setup( GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 );
	gpio_set_output_options( GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8 );

	// Initially set LED
	gpio_clear( GPIOA, GPIO8 );

	// Add a task that blinks LED0 in a blocking manner
	xTaskCreate( task_blink, /* The function that implements the task. */
		"blink", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
		configMINIMAL_STACK_SIZE, /* The size of the stack to allocate to the task. */
		NULL, /* The parameter passed to the task - not used in this case. */
		1, /* The priority assigned to the task. */
		NULL ); /* The task handle is not required, so NULL is passed. */

	vTaskStartScheduler();

	return 0;
}

void gpio_set( GPIO_TypeDef * gpio, uint16_t gpios )
{
	gpio->BSRR = gpios;
}

void gpio_clear( GPIO_TypeDef * gpio, uint16_t gpios )
{
	gpio->BSRR = (gpios << 16);
}

void gpio_toggle( GPIO_TypeDef * gpio, uint16_t gpios )
{
	uint32_t port = gpio->ODR;
	gpio->BSRR = ((port & gpios) << 16) | (~port & gpios);
}

void gpio_mode_setup( GPIO_TypeDef * gpio, uint8_t mode, uint8_t pull_up_down, uint16_t gpios )
{
	uint16_t i;
	uint32_t moder, pupd;

	/*
	 * We want to set the config only for the pins mentioned in gpios,
	 * but keeping the others, so read out the actual config first.
	 */
	moder = gpio->MODER;
	pupd = gpio->PUPDR;

	for (i = 0; i < 16; i++) {
		if (!((1 << i) & gpios)) {
			continue;
		}

		moder &= ~GPIO_MODE_MASK(i);
		moder |= GPIO_MODE(i, mode);
		pupd &= ~GPIO_PUPD_MASK(i);
		pupd |= GPIO_PUPD(i, pull_up_down);
	}

	/* Set mode and pull up/down control registers. */
	gpio->MODER = moder;
	gpio->PUPDR = pupd;
}

void gpio_set_output_options( GPIO_TypeDef * gpio, uint8_t otype, uint8_t speed, uint16_t gpios )
{
	uint16_t i;
	uint32_t ospeedr;

	if (otype == 0x1) {
		gpio->OTYPER |= gpios;
	} else {
		gpio->OTYPER &= ~gpios;
	}

	ospeedr = gpio->OSPEEDR;

	for (i = 0; i < 16; i++) {
		if (!((1 << i) & gpios)) {
			continue;
		}
		ospeedr &= ~GPIO_OSPEED_MASK(i);
		ospeedr |= GPIO_OSPEED(i, speed);
	}

	gpio->OSPEEDR = ospeedr;
}

// void gpio_set_af(GPIO_TypeDef * gpio, uint8_t alt_func_num, uint16_t gpios)
// {
// 	uint16_t i;
// 	uint32_t afrl, afrh;

// 	afrl = GPIO_AFRL(gpioport);
// 	afrh = GPIO_AFRH(gpioport);

// 	for (i = 0; i < 8; i++) {
// 		if (!((1 << i) & gpios)) {
// 			continue;
// 		}
// 		afrl &= ~GPIO_AFR_MASK(i);
// 		afrl |= GPIO_AFR(i, alt_func_num);
// 	}

// 	for (i = 8; i < 16; i++) {
// 		if (!((1 << i) & gpios)) {
// 			continue;
// 		}
// 		afrh &= ~GPIO_AFR_MASK(i - 8);
// 		afrh |= GPIO_AFR(i - 8, alt_func_num);
// 	}

// 	GPIO_AFRL(gpioport) = afrl;
// 	GPIO_AFRH(gpioport) = afrh;
// }
