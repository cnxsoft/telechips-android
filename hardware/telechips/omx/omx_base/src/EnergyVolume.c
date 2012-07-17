#include <stdlib.h>
#include <stdio.h>

#define SAMPLE_COUNT 512
#define STEREO 1


void GetEnergyofVolume ( short *left, short *right, int buff_length,int LR,unsigned int* left_level, unsigned int* right_level)
{
	short curr_left, curr_right;
	unsigned int temp_left, temp_right;	
	unsigned int buff_cnt, cnt_offset, cntofdivide;
	
	temp_left = 0;
	temp_right = 0;
	buff_cnt = 0;
	cnt_offset=0;
	cntofdivide = 0;

	if (buff_length > SAMPLE_COUNT)
	{
		cnt_offset = (buff_length/SAMPLE_COUNT);		// divide by SAMPLE_COUNT
	}
	else
	{
		cnt_offset = 1;
	}

	while (buff_cnt < buff_length)					// buff_length = MP3=1152, WMA = 2~2300 OGG 4XX ~ 1024
	{
		curr_left = *(left+buff_cnt);

		if (curr_left < 0)
		{
			curr_left ^= 0xFFFF;
		}

		temp_left += curr_left;

		if (LR == STEREO)
		{
			curr_right = *(right+buff_cnt);

			if (curr_right <0)
			{
				curr_right ^= 0xFFFF;
			}

			temp_right += curr_right;
		}

		buff_cnt += cnt_offset;
		cntofdivide++;	
	}

	// Get Average
	if(cntofdivide)
	{
		*left_level = temp_left = temp_left  / cntofdivide;

		if (LR == STEREO)
		{
			*right_level = temp_right = temp_right / cntofdivide;
		}
		else
		{
			*left_level = temp_right = temp_left;
		}
	}
}

