#ifndef TCC_SPDIF_BITSTREAM_H              
#define TCC_SPDIF_BITSTREAM_H

#include <stdint.h>

#define array_size(array) (sizeof(array) / sizeof(array[0]))

#define int2be32(i) spdif_swab_s32(i)
#define int2le32(i) (i)
#define int2be16(i) spdif_swab_s16(i)
#define int2le16(i) (i)

#define be2int32(i) spdif_swab_s32(i)
#define le2int32(i) (i)
#define be2int16(i) spdif_swab_s16(i)
#define le2int16(i) (i)

#define uint2be32(i) spdif_swab_u32(i)
#define uint2le32(i) (i)
#define uint2be16(i) spdif_swab_u16(i)
#define uint2le16(i) (i)

#define be2uint32(i) spdif_swab_u32(i)
#define le2uint32(i) (i)
#define be2uint16(i) spdif_swab_u16(i)
#define le2uint16(i) (i)



int32_t bitstream_convert(const uint8_t *in_buf, int32_t size, int32_t in_bs, uint8_t *out_buf, int32_t out_bs);

typedef int32_t (*bitstream_converter)(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
bitstream_converter bitstream_conversion(int32_t bs_from, int32_t bs_to);

int32_t bitstream_copy(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_swab16(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);

int32_t bitstream_8_to_14be(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_8_to_14le(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_14be_to_8(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_14le_to_8(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);

int32_t bitstream_16le_to_14be(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_16le_to_14le(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_14be_to_16le(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);
int32_t bitstream_14le_to_16le(const uint8_t *in_buf, int32_t size, uint8_t *out_buf);


#endif /* TCC_SPDIF_BITSTREAM_H */
