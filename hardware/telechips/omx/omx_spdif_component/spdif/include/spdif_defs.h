#ifndef TCC_SPDIR_DEFS_H
#define TCC_SPDIR_DEFS_H


#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif


#define DTS_MODE_AUTO    0
#define DTS_MODE_WRAPPED 1
#define DTS_MODE_PADDED  2

#define DTS_CONV_NONE    0
#define DTS_CONV_16BIT   1
#define DTS_CONV_14BIT   2

// bitstream types
#define BITSTREAM_NONE  (-1)
#define BITSTREAM_8     0
#define BITSTREAM_16BE  1
#define BITSTREAM_16LE  2
#define BITSTREAM_32BE  3
#define BITSTREAM_32LE  4
#define BITSTREAM_14BE  5  
#define BITSTREAM_14LE  6  

#if 0
inline size_t align32(const void *ptr)
{ 
	return (size_t)ptr & 3; 
}
#endif

typedef struct
{
  unsigned int	frame_size;
  unsigned int	scan_size;
  unsigned int	nsamples;
  int      	  	bs_type;
  unsigned int	spdif_type;
}spdif_header_info_s;



#define MAX_SPDIF_FRAME_SIZE 65536//16384//8192

// data tpye define
// value of pc bit 0~4
typedef enum
{
	SPDIF_NULL_DATA = 0,
	SPDIF_AC3,				// 1536
	SPDIF_SMPTE,			// 
	SPDIF_PAUSE,			
	SPDIF_MPEG1_1,			// 384
	SPDIF_MPEG1_OR_MPEG2,	// 1152 (without extension)
	SPDIF_MPEG2_EXT,		// 1152 (with extension)	
	SPDIF_MPEG2_AAC,		// 1024
	SPDIF_MPEG2_LAYER1,		// 768
	SPDIF_MPEG2_LAYER2,		// 2304
	SPDIF_MPEG2_LAYER3,		// 1152
	SPDIF_DTS_TYPE1,		// 512
	SPDIF_DTS_TYPE2,		// 1024
	SPDIF_DTS_TYPE3,		// 2048
	SPDIF_ATRAC,			// 512
	SPDIF_ATRAC_2_3,		// 1024
	SPDIF_ATRAC_X,			// 2048
	SPDIF_DTS_TYPE4,		// See iec 61937_5
	SPDIF_WMA,				// 512~2048 
	SPDIF_AAC_LOW,			// 2048~4096
	SPDIF_MPEG4_AAC,		// 512~4096
	SPDIF_ENHANCE_AC3,		// 6144
	SPDIF_MAT,				// 15360
	// 23~23 : reserved
	// 27~30 : SMPTE 338M
	SPDIF_EXT_DATATYPE = 31,
}SPDIF_DATA_TYPE;


typedef struct spdif_header_structure_s
{
    unsigned short sync1;   // Pa sync word 1
    unsigned short sync2;   // Pb sync word 2
    unsigned short type;    // Pc data type
    unsigned short len;     // Pd length-code (bits)
}spdif_header_type_s;

typedef struct spdif_ddp_header_structure_s
{

    unsigned short sync1;   // Pa sync word 1
    unsigned short sync2;   // Pb sync word 2
    unsigned short type;    // Pc data type
    unsigned short len;     // Pd length-code (bits)
}spdif_ddp_header_type_s;


#endif /* TCC_SPDIR_DEFS_H */

