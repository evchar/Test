#ifndef __DRV_KEYS_H__
#define __DRV_KEYS_H__


#define KEY_LONG_PERIOD         125 
#define KEY_CONTINUE_PERIOD     75 

/*定义按键状态 */
typedef enum
{
    KEY_STATE_INIT = 0,
    KEY_STATE_WOBBLE,
    KEY_STATE_PRESS,
    KEY_STATE_LONG,
    KEY_STATE_CONTINUE,
    KEY_STATE_RELEASE
}KEY_STATE;


#define KEY0_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOB_IDR_OFFSET * 32 + 9 * 4)
#define KEY1_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOC_IDR_OFFSET * 32 + 4 * 4)
//#define KEY2_Bit            *(uint32_t *)(PERIPH_BB_BASE + GPIOE_IDR_OFFSET * 32 + 2 * 4)



//定义按键返回值状态(按下,长按,连_发,释放) 
#define KEY_0                    0x01 
#define KEY_1                    0x02 
#define KEY_2                    0x04 
#define KEY_NULL                 0x00

#define KEY_DOWN                 0x80 
#define KEY_LONG                 0x40 
#define KEY_CONTINUE             0x20 
#define KEY_UP                   0x10

#define KEY0_DOWN              (KEY_0|KEY_DOWN)
#define KEY1_DOWN              (KEY_1|KEY_DOWN)
#define KEY2_DOWN              (KEY_2|KEY_DOWN)

#define KEY0_LONG              (KEY_0|KEY_LONG)
#define KEY1_LONG              (KEY_1|KEY_LONG)
#define KEY2_LONG              (KEY_2|KEY_LONG)


#define KEY0_CONTINUE          (KEY_0|KEY_CONTINUE)
#define KEY1_CONTINUE          (KEY_1|KEY_CONTINUE)
#define KEY2_CONTINUE          (KEY_2|KEY_CONTINUE)

#define KEY0_UP                (KEY_0|KEY_UP)
#define KEY1_UP                (KEY_1|KEY_UP)
#define KEY2_UP                (KEY_2|KEY_UP)

void GetKey(u8 *pKeyValue) ;


#endif



