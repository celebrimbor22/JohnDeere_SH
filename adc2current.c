
/* Forward declaration for rtModel */
#include "rtwtypes.h"


/* Block signals (default storage) */
typedef struct {
    float  Gain;                         /* '<S1>/Gain' */
    float  Vin;                          /* '<S1>/Divide' */
    float Subtract;                     /* '<S3>/Subtract' */
    uint16_T adcSat;                     /* '<Root>/Saturation' */
    uint16_T currentHigh;                /* '<S2>/Cast' */
    uint16_T Cast1;                      /* '<S2>/Cast1' */
    uint16_T courrentLow;                /* '<S2>/Gain' */
    boolean_T Compare;                   /* '<S4>/Compare' */
    boolean_T Compare_m;                 /* '<S5>/Compare' */
} B_adc2current_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
    uint16_T adcOut;                     /* '<Root>/Input' */
} ExtU_adc2current_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
    float  Iout;                         /* '<Root>/Iout' */
    uint16_T CurrentState;              /* '<Root>/CurrentState' */
} ExtY_adc2current_T;



unsigned short testCurrent(unsigned short ADCin);
/* Block signals (default storage) */
B_adc2current_T adc2current_B;

/* External inputs (root inport signals with default storage) */
ExtU_adc2current_T adc2current_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_adc2current_T adc2current_Y;


/* Model step function */
int adc2current_step(int aux)
{
  uint16_T u0;

  /* Saturate: '<Root>/Saturation' incorporates:
   *  Inport: '<Root>/Input'
   */
  u0 = adc2current_U.adcOut;
  if (u0 > 4095) {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = 4095U;
  } else if (u0 < 1985) {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = 1985U;
  } else {
    /* Saturate: '<Root>/Saturation' */
    adc2current_B.adcSat = u0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Gain: '<S1>/Gain' */
  adc2current_B.Gain = 3.29998779296875 * (float)adc2current_B.adcSat;

  /* Product: '<S1>/Divide' incorporates:
   *  Constant: '<Root>/Constant'
   */
  adc2current_B.Vin = adc2current_B.Gain / 4095.0;

  /* Sum: '<S3>/Subtract' incorporates:
   *  Constant: '<Root>/Constant1'
   */
  adc2current_B.Subtract = adc2current_B.Vin - 2.5;

  /* Outport: '<Root>/Iout' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Product: '<S3>/Divide1'
   */
  adc2current_Y.Iout = adc2current_B.Subtract / 0.18;

  /* RelationalOperator: '<S4>/Compare' incorporates:
   *  Constant: '<S4>/Constant'
   *  Outport: '<Root>/Iout'
   */
  adc2current_B.Compare = (adc2current_Y.Iout >= 4.0);

  /* DataTypeConversion: '<S2>/Cast' */
  adc2current_B.currentHigh = adc2current_B.Compare;

  /* RelationalOperator: '<S5>/Compare' incorporates:
   *  Constant: '<S5>/Constant'
   *  Outport: '<Root>/Iout'
   */
  adc2current_B.Compare_m = (adc2current_Y.Iout <= 0.1);

  /* DataTypeConversion: '<S2>/Cast1' */
  adc2current_B.Cast1 = adc2current_B.Compare_m;

  /* Gain: '<S2>/Gain' */
  adc2current_B.courrentLow = (uint16_T)(adc2current_B.Cast1 << 1);

  /* Outport: '<Root>/CurrentState' incorporates:
   *  Sum: '<S2>/Add'
   */
  adc2current_Y.CurrentState = (uint16_T)((uint32_T)adc2current_B.currentHigh +
    adc2current_B.courrentLow);

  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

unsigned short testCurrent(unsigned short ADCin)
{
    adc2current_U.adcOut = ADCin;
    int aux = adc2current_step(0);
    return adc2current_Y.CurrentState;
}
