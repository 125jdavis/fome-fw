/*
 * @file CJ125.h
 *
 * @date: Jul 17, 2016
 * @author Andrey Belomutskiy, (c) 2012-2016
 */

#ifndef HW_LAYER_SENSORS_CJ125_H_
#define HW_LAYER_SENSORS_CJ125_H_

#define	IDENT_REG_RD	0x48 // Read Identity Register, decimal 72
#define	INIT_REG1_WR	0x56 // Write To Initialization Register 1, decimal 86
#define	CJ125_NORMAL	0x89 // 137 decimal
#define	CJ125_CALBRT	0x9D // 157 decimal

void initCJ125(void);

typedef enum {
CJ125_IDLE = 0

} cj125_state_e;

#endif /* HW_LAYER_SENSORS_CJ125_H_ */
