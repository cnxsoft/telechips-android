#ifndef __TCC_VPU_ENC_INTERFACE_H__
#define __TCC_VPU_ENC_INTERFACE_H__

extern int TCC_VPU_Enc_Init(int bitstream_format, unsigned int uiWidth, unsigned int uiHeight, unsigned char quality);
extern int TCC_VPU_Enc_Deinit(void);
extern unsigned int TCC_VPU_Enc_Get_CFGData(unsigned char  *pBuffer, unsigned int *size, unsigned char isSPS);
extern int TCC_VPU_Encode(unsigned char *p_input, unsigned int input_len, unsigned char *p_output, unsigned int *output_len);

extern unsigned int TCC_VPU_Enc_Get_VirtualBuffer(void);
extern unsigned int TCC_VPU_Enc_Get_PhysicalBuffer(void);
extern void TCC_VPU_Enc_Inc_BufferIdx(void);
#endif//__TCC_VPU_ENC_INTERFACE_H__