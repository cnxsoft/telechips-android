#ifndef TCC_SPDIF_DTS_H
#define TCC_SPDIF_DTS_H

static const int dts_sampling_rate[] =
{
	0,
	8000,
	16000,
	32000,
	0,
	0,
	11025,
	22050,
	44100,
	0,
	0,
	12000,
	24000,
	48000,
	96000,
	192000
};

int dts_header_parse(unsigned char* hdr,spdif_header_info_s *hinfo, int dtsiv_mode);
int dts_header_parse_iv(unsigned char* hdr,spdif_header_info_s *hinfo, int dtsiv_mode, int packet_size);

#endif /* TCC_SPDIF_DTS_H */
