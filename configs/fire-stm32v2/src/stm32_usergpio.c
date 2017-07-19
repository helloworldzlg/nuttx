#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include "stm32_gpio.h"
#include <nuttx/ioexpander/gpio.h>

#ifdef CONFIG_DEV_GPIO

#define BCAS_GPIO_ID_DISTANCE           (10)

typedef struct
{
  uint8_t gpio_name[8];
  uint8_t gpio_pin;
  uint8_t gpio_pintype;
  uint8_t gpio_signo;
  uint8_t gpio_pid;
  bool gpio_rising;
  bool gpio_falling;
  uint32_t gpio_pinset;
  void (*handle)(int);
}GPIO_CONFIG_DES_S;

typedef enum
{
	PC_9 = BCAS_GPIO_ID_DISTANCE,
	PF_14,
	PF_15,
	PC_7,
	PF12,
	PF13,
	PB14,
	PB15,
}USER_GPIO_ID_E;

void lforward_speed_cb(int param);
void lbackward_speed_cb(int param);
void rforward_speed_cb(int param);
void rbackward_speed_cb(int param);
void lbumper_trigger_cb(int param);
void rbumper_trigger_cb(int param);

/* GPIO config list for bcas */
GPIO_CONFIG_DES_S g_gpio_cfg_dec[] = {
  /* left wheel motor direction control */
  { "PC9",  99,  GPIO_OUTPUT_PIN,     PC_9, 0, false, false, GPIO_PORTC | GPIO_PIN9  | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | GPIO_MODE_50MHz, NULL},
  
  /* left wheel encode pause input */  
  { "PF14", 54,  GPIO_INTERRUPT_PIN,  PF_14, 0, true, false, GPIO_PORTF | GPIO_PIN14 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, lforward_speed_cb},
  { "PF15", 55,  GPIO_INTERRUPT_PIN,  PF_15, 0, true, false, GPIO_PORTF | GPIO_PIN15 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, lbackward_speed_cb},
  
  /* right wheel motor direction control */
  { "PC7",  97,  GPIO_OUTPUT_PIN,     PC_7, 0, false, false, GPIO_PORTC | GPIO_PIN7  | GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR |GPIO_MODE_50MHz, NULL},
    
  /* right wheel encode pause input */
  { "PF12", 50,  GPIO_INTERRUPT_PIN,  PF12, 0, true, false, GPIO_PORTF | GPIO_PIN12 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, rforward_speed_cb},
  { "PF13", 53,  GPIO_INTERRUPT_PIN,  PF13, 0, true, false, GPIO_PORTF | GPIO_PIN13 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, rbackward_speed_cb},

  /* left & right bumper pause input */
  { "PB14", 75,  GPIO_INTERRUPT_PIN,  PB14, 0, true, true, GPIO_PORTB | GPIO_PIN14 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, rbumper_trigger_cb},
  { "PB15", 76,  GPIO_INTERRUPT_PIN,  PB15, 0, true, true, GPIO_PORTB | GPIO_PIN15 | GPIO_INPUT  | GPIO_CNF_INPULLUP | GPIO_EXTI, lbumper_trigger_cb},

};

/*****************************************************************************
 * Function      : forward_speed_cb
 * Description   : 
 * Input         : int param  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
void lforward_speed_cb(int param)
{
  //printf("%s\n", __func__);
}

void rforward_speed_cb(int param)
{
  //printf("%s\n", __func__);
}

/*****************************************************************************
 * Function      : backward_speed_cb
 * Description   : 
 * Input         : int param  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
void lbackward_speed_cb(int param)
{
  //printf("%s\n", __func__);
}

void rbackward_speed_cb(int param)
{
  //printf("%s\n", __func__);
}

/*****************************************************************************
 * Function      : lbumper_trigger_cb
 * Description   : 
 * Input         : int param  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
void lbumper_trigger_cb(int param)
{
  //printf("lbumper_trigger\n");
}

void rbumper_trigger_cb(int param)
{
  //printf("rbumper_trigger\n");
}


struct gpio_dev_s g_gpio_dev_s[sizeof(g_gpio_cfg_dec)/sizeof(GPIO_CONFIG_DES_S)];

/*****************************************************************************
 * Function      : user_gpio_read
 * Description   : 
 * Input         : FAR struct gpio_dev_s *dev  
                FAR bool *value             
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int user_gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  uint32_t pinset = g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].gpio_pinset;

  *value = stm32_gpioread(pinset);
  return 1;
}

/*****************************************************************************
 * Function      : user_gpio_write
 * Description   : 
 * Input         : FAR struct gpio_dev_s *dev  
                bool value                  
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int user_gpio_write(FAR struct gpio_dev_s *dev, bool value)
{
  uint32_t pinset = g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].gpio_pinset;

  stm32_gpiowrite(pinset, value);
  return 0;
}

/*****************************************************************************
 * Function      : user_gpio_attach
 * Description   : 
 * Input         : FAR struct gpio_dev_s *dev
                pin_interrupt_t callback    
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int user_gpio_attach(FAR struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  bool risingedge;
  bool fallingedge;
  int32_t arg = 0;
 
  risingedge  = g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].gpio_rising;
  fallingedge = g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].gpio_falling;
  
  uint32_t pinset = g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].gpio_pinset;
  pin_interrupt_t handle = (pin_interrupt_t)g_gpio_cfg_dec[dev->gp_signo - BCAS_GPIO_ID_DISTANCE].handle;

  stm32_gpiosetevent(pinset, risingedge, fallingedge, false, (xcpt_t)handle, (void*)&arg);

  return 0;
}

/*****************************************************************************
 * Function      : user_gpio_enable
 * Description   : 
 * Input         : FAR struct gpio_dev_s *dev  
                bool enable                 
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int user_gpio_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  return 0;
}

struct gpio_operations_s g_gpio_op_s = {
  .go_read   = user_gpio_read,
  .go_write  = user_gpio_write,
  .go_attach = user_gpio_attach,
  .go_enable = user_gpio_enable,
};

/*****************************************************************************
 * Function      : stm32_usergpio_init
 * Description   : user gpio init & dev register
 * Input         : None
 * Output        : None
 * Return        : 
 * Others        : 
 * Record
 * 1.Date        : 20170713
 *   Author      : zhanglg
 *   Modification: Created function

*****************************************************************************/
int stm32_usergpio_init(void)
{
  int result;
  uint8_t index;
  uint8_t bcas_gpio_num = sizeof(g_gpio_cfg_dec)/sizeof(GPIO_CONFIG_DES_S);

  struct gpio_dev_s *p_gpio_op = NULL;
  GPIO_CONFIG_DES_S *p_bcas_gpio_dec = (GPIO_CONFIG_DES_S*)&g_gpio_cfg_dec;

  for (index = 0; index < bcas_gpio_num; index++)
  {
    p_gpio_op = &g_gpio_dev_s[index];
    memset((void*)p_gpio_op, 0, sizeof(struct gpio_dev_s));

    p_gpio_op->gp_pintype = p_bcas_gpio_dec[index].gpio_pintype;
    p_gpio_op->gp_signo   = p_bcas_gpio_dec[index].gpio_signo;
    p_gpio_op->gp_pid     = getpid();
    p_gpio_op->gp_ops     = &g_gpio_op_s;

    result = stm32_configgpio(p_bcas_gpio_dec[index].gpio_pinset);
    if (result < 0)
    {
      syslog(LOG_ERR, "ERROR: %s line %d failed!!!, ret %d\n", __func__, __LINE__, result);
      return result;
    }

    result = gpio_pin_register(p_gpio_op, p_bcas_gpio_dec[index].gpio_pin);
    if (result < 0)
    {
      syslog(LOG_ERR, "ERROR: %s line %d failed!!!, ret %d\n", __func__, __LINE__, result);
      return result;
    }
  }

  return 0;
}

#endif
