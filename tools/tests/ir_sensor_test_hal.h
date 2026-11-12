#ifndef IR_SENSOR_TEST_HAL_H
#define IR_SENSOR_TEST_HAL_H
#include <stdint.h>

typedef struct { uint32_t BSRR; } GPIO_TypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } TestHandle;
typedef struct { uint32_t SrcAddress, DstAddress, DataSize; } DMA_NodeConfTypeDef;
extern GPIO_TypeDef test_gpio;
extern ADC_HandleTypeDef hadc1;
extern TestHandle hlptim1, handle_GPDMA1_Channel0, handle_GPDMA1_Channel1;
extern TestHandle Node_GPDMA1_Channel0, Node_GPDMA1_Channel1;
extern TestHandle List_GPDMA1_Channel0, List_GPDMA1_Channel1;
extern uint32_t test_irq_mask, test_adc_starts, test_adc_stops, test_timer_starts;
extern void (*test_irq_restored_hook)(void);
#define GPIOA (&test_gpio)
#define IR1_Pin (1U << 1)
#define IR2_Pin (1U << 2)
#define IR3_Pin (1U << 3)
#define IR4_Pin (1U << 4)
#define GPIO_PIN_RESET 0U
#define HAL_OK 0
#define LPTIM_CHANNEL_1 1
#define LPTIM_CHANNEL_2 2
static inline uint32_t __get_PRIMASK(void) { return test_irq_mask; }
static inline void __disable_irq(void) { test_irq_mask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) {
    test_irq_mask = mask;
    if (mask == 0U && test_irq_restored_hook) {
        void (*hook)(void) = test_irq_restored_hook;
        test_irq_restored_hook = 0;
        hook();
    }
}
static inline void __DMB(void) {}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint32_t pins, int state)
{ (void)state; gpio->BSRR = pins << 16; }
static inline int HAL_DMAEx_List_GetNodeConfig(DMA_NodeConfTypeDef *config, void *node)
{ (void)config; (void)node; return HAL_OK; }
static inline int HAL_DMAEx_List_BuildNode(DMA_NodeConfTypeDef *config, void *node)
{ (void)config; (void)node; return HAL_OK; }
static inline int HAL_DMAEx_List_InsertNode(void *list, void *previous, void *node)
{ (void)list; (void)previous; (void)node; return HAL_OK; }
static inline int HAL_DMAEx_List_SetCircularMode(void *list)
{ (void)list; return HAL_OK; }
static inline int HAL_DMAEx_List_Start_IT(void *handle)
{ (void)handle; return HAL_OK; }
static inline int HAL_ADC_Start_DMA(void *handle, uint32_t *data, int count)
{ (void)handle; (void)data; (void)count; ++test_adc_starts; return HAL_OK; }
static inline int HAL_ADC_Stop_DMA(void *handle)
{ (void)handle; ++test_adc_stops; return HAL_OK; }
static inline int HAL_LPTIM_IC_Start(void *handle, int channel)
{ (void)handle; (void)channel; ++test_timer_starts; return HAL_OK; }
static inline int HAL_LPTIM_IC_Stop(void *handle, int channel)
{ (void)handle; (void)channel; return HAL_OK; }
void Error_Handler(void);
#endif
