/*===========================================================================

       M u l t i m e d i a   C D K        

           V O R B I S   D E C O D E R   S O U R C E   F I L E

===========================================================================*/

#include "adec.h"
#include "codec_internal.h"
#include "ivorbiscodec.h"

#include "utils/Log.h"

#ifndef NULL
#define NULL 0
#endif
#define MAX_INST  5
#define VORBIS_HANDLE_DUMMY_SIZE  8
#define ABS(x)  ((x)>0?(x):-(x))
typedef struct 
{
  int   *Handle;
  int   used;
  short *pcm;
  int   downmix;
  int   nSamples;

  ogg_packet      op;
  vorbis_info     vi;
  vorbis_comment    vc;
  vorbis_dsp_state  vd;
}vorbis_instance_t;

static vorbis_instance_t  inst_vorbis[MAX_INST];
short   convbuffer[4096*6];

void  (*cdk_ogg_free     ) ( void* );             //!< cdk_ogg_free
void  (*cdk_ogg_memset     ) ( void*, int, unsigned int );    //!< cdk_ogg_memset

int _vorbis_unpack_books(vorbis_info *vi,oggpack_buffer *opb);
int _vorbis_unpack_info(vorbis_info *vi,oggpack_buffer *opb);
int _vorbis_unpack_comment(vorbis_comment *vc,oggpack_buffer *opb);

static int vorbis_parseheader(vorbis_info *vi,vorbis_comment *vc,ogg_packet *op, int *UseByte);
static void down_mix_to_stereo(int samples, short *pcm, int inCh, adec_output_t *p_adec_out);
static int convert_vorbis_to_cdk_pcm(int inCh, int downMix, int samples, short *pcm, adec_output_t *p_adec_out);

int TCC_VORBIS_DEC( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
  TCAS_ERROR_TYPE   eTcasError;
  int         ret,samples;
   ogg_buffer buf;
   ogg_reference ref;
   oggpack_buffer bits;

  switch(iOpCode)
  {
    case AUDIO_INIT:
      {
        int i,inst_size,exlen;
        int run=0;
        char    *p ;
        adec_init_t *init;
        adec_output_t *p_adec_out;
        int    *Handle  =NULL;
        
        // Get adec_init
        if(pParam1 == NULL)
        {
          return  TCAS_ERROR_NULL_INSTANCE;
        }
        else
        {
          init = (adec_init_t *)pParam1;
        }
        
        p_adec_out  = init->m_psAudiodecOutput;

        cdk_ogg_free  = 0x0;        
        cdk_ogg_memset  = 0x0;        
        
        cdk_ogg_free  = init->m_pfFree;
        cdk_ogg_memset  = init->m_pfMemset;

        // call back function null check        
        if((cdk_ogg_free==NULL) || (cdk_ogg_memset==NULL))
            return TCAS_ERROR_NULL_INSTANCE;

        // find instance
        for(i=1;i<MAX_INST;i++)
        {
          if(inst_vorbis[i].used  ==  0)
          {
            *pHandle  = i;  
            break;
          }
        }
        if(i>=MAX_INST)
          return TCAS_ERROR_NULL_INSTANCE;
        
        // Alloc handle
        inst_size = VORBIS_HANDLE_DUMMY_SIZE;
        Handle  = inst_vorbis[i].Handle = (int*)init->m_pfMalloc(inst_size);
        if(Handle ==  NULL)
          return  TCAS_ERROR_NULL_INSTANCE;
        cdk_ogg_memset(Handle,0,inst_size);
        cdk_ogg_memset(&inst_vorbis[i].op,0,sizeof(ogg_packet));
        // Vorbis decoder Init
        vorbis_info_init (&(inst_vorbis[i].vi));        
        vorbis_comment_init (&(inst_vorbis[i].vc));

        // Header loading from extra data of demuxer
        inst_vorbis[i].op.packet = &ref;
        inst_vorbis[i].op.bytes = init->m_iExtraDataLen  ;

        buf.data = (unsigned char *)init->m_pucExtraData;
        buf.size = init->m_iExtraDataLen;
        buf.refcount = 1;
        buf.ptr.owner = NULL;

        inst_vorbis[i].op.packet->buffer = &buf;
        inst_vorbis[i].op.packet->length = init->m_iExtraDataLen;
        inst_vorbis[i].op.packet->begin = 0;
        inst_vorbis[i].op.packet->next = NULL;
        inst_vorbis[i].op.b_o_s =1;
        
        // Header  #1 
        if (vorbis_parseheader (&(inst_vorbis[i].vi), &(inst_vorbis[i].vc), &(inst_vorbis[i].op), &samples) < 0)       
        {
          /* error case; not a vorbis header */
          return  TCAS_ERROR_OPEN_FAIL;
        }

        inst_vorbis[i].op.bytes = inst_vorbis[i].op.bytes - samples;//inst_vorbis[i].op.consumed_byte;
        buf.data += samples;
        buf.size -= samples;
        buf.refcount = 1;
        buf.ptr.owner = NULL;
        inst_vorbis[i].op.packet->buffer = &buf;
        inst_vorbis[i].op.packet->length = buf.size;
        inst_vorbis[i].op.packet->begin = 0;
        inst_vorbis[i].op.packet->next = NULL;

        // Header  #2 
        if (vorbis_parseheader (&(inst_vorbis[i].vi), &(inst_vorbis[i].vc), &(inst_vorbis[i].op), &samples) < 0)       
        {
          /* error case; not a vorbis header */
          return  TCAS_ERROR_OPEN_FAIL;
        }
 
        inst_vorbis[i].op.bytes = inst_vorbis[i].op.bytes - samples;//inst_vorbis[i].op.consumed_byte;
        buf.data += samples;
        buf.size -= samples;
        buf.refcount = 1;
        buf.ptr.owner = NULL;
        inst_vorbis[i].op.packet->buffer = &buf;
        inst_vorbis[i].op.packet->length = buf.size;
        inst_vorbis[i].op.packet->begin = 0;
        inst_vorbis[i].op.packet->next = NULL;

        // Header  #3 
        if (vorbis_parseheader (&(inst_vorbis[i].vi), &(inst_vorbis[i].vc), &(inst_vorbis[i].op), &samples) < 0)
        {
          /* error case; not a vorbis header */
          return  TCAS_ERROR_OPEN_FAIL;
        }
        inst_vorbis[i].op.bytes = inst_vorbis[i].op.bytes - samples;//inst_vorbis[i].op.consumed_byte;
        buf.data += samples;
        buf.size -= samples;
        buf.refcount = 1;
        buf.ptr.owner = NULL;
        inst_vorbis[i].op.packet->buffer = &buf;
        inst_vorbis[i].op.packet->length = buf.size;
        inst_vorbis[i].op.packet->begin = 0;
        inst_vorbis[i].op.packet->next = NULL;

        ret = vorbis_dsp_init (&(inst_vorbis[i].vd), &(inst_vorbis[i].vi));
        if (ret)
        {
          return  TCAS_ERROR_OPEN_FAIL;
        }

        if (inst_vorbis[i].vi.channels > 6)   // limit decoding Max channel
        {
          return TCAS_ERROR_NOT_SUPPORT_FORMAT;
        }

        if(ret  ==  TCAS_SUCCESS)
        {
          inst_vorbis[i].downmix = init->m_iDownMixMode;
          if ((init->m_iDownMixMode) && (inst_vorbis[i].vi.channels > 2))
          {
            p_adec_out->m_uiNumberOfChannel = 2;
          }
          else
            p_adec_out->m_uiNumberOfChannel = inst_vorbis[i].vi.channels;

          p_adec_out->m_uiSamplesPerChannel = 2048;
          p_adec_out->m_uiBitsPerSample   = 16;
          p_adec_out->m_eSampleRate     = inst_vorbis[i].vi.rate;;          

          *pHandle  = i;      
          inst_vorbis[i].used =1;
          ret = TCAS_SUCCESS;
        }
      }
      break;

      case AUDIO_DECODE:
      {
        int i,out_ch,consumed_byte,remainbits;
        int    *Handle  ;
        int       total_samples;

        adec_input_t  *p_adec_in;   
        adec_output_t *p_adec_out;  

        i = *pHandle;
        Handle  = inst_vorbis[i].Handle;

        // Parameters Mapping
        if((pParam1 ==NULL)||(pParam2 ==NULL))
        {
          return  TCAS_ERROR_NULL_INSTANCE;
        }
        else
        {
          p_adec_in = (adec_input_t *)pParam1;
          p_adec_out  = (adec_output_t  *)pParam2;
        }
        buf.data = (unsigned char *)p_adec_in->m_pcStream;
        buf.size = p_adec_in->m_iStreamLength;
        buf.refcount = 1;
        buf.ptr.owner = NULL;

        ref.buffer = &buf;
        ref.begin = 0;
        ref.length = buf.size;
        ref.next = NULL;

        inst_vorbis[i].op.packet = &ref;
        inst_vorbis[i].op.bytes = ref.length;
        inst_vorbis[i].op.b_o_s = 0;
        inst_vorbis[i].op.e_o_s = 0;
        inst_vorbis[i].op.granulepos = 0;
        inst_vorbis[i].op.packetno = 0;
		
        vorbis_dsp_synthesis (&(inst_vorbis[i].vd), &(inst_vorbis[i].op), 1);

	      samples = vorbis_dsp_pcmout (&(inst_vorbis[i].vd), &convbuffer[0],((sizeof(convbuffer)>>1)/inst_vorbis[i].vi.channels));
  	    out_ch = convert_vorbis_to_cdk_pcm(inst_vorbis[i].vi.channels, inst_vorbis[i].downmix, samples, &convbuffer[0], p_adec_out);
    	  vorbis_dsp_read (&(inst_vorbis[i].vd), samples);  /* tell libvorbis how many samples we actually consumed */        

        p_adec_in->m_pcStream += p_adec_in->m_iStreamLength;
        p_adec_in->m_iStreamLength  = 0;
        
        p_adec_out->m_uiNumberOfChannel   =   out_ch;
        p_adec_out->m_uiSamplesPerChannel =   samples;
        p_adec_out->m_uiBitsPerSample   = 16;
        ret = TCAS_SUCCESS;
      }
      break;
      case AUDIO_CLOSE:
      {
        int i;
        i = *pHandle;

        if(i==0)
          return TCAS_SUCCESS;

        if(inst_vorbis[i].Handle)
          cdk_ogg_free(inst_vorbis[i].Handle);
        if(inst_vorbis[i].pcm)
          cdk_ogg_free(inst_vorbis[i].pcm);

        /* clean up this logical bitstream; before exit we see if we're
        * followed by another [chained] */
        vorbis_dsp_clear (&(inst_vorbis[i].vd));
        vorbis_comment_clear (&(inst_vorbis[i].vc));
        vorbis_info_clear (&(inst_vorbis[i].vi)); /* must be called last */       

        inst_vorbis[i].used   = 0;
        inst_vorbis[i].Handle   = NULL;
        inst_vorbis[i].pcm    = NULL;
        ret   =   TCAS_SUCCESS;

	  }
  }

  return ret;
}

static void readstring(oggpack_buffer *o,char *buf,int bytes){
  while(bytes--){
    *buf++=(char)oggpack_read(o,8);
  }
}

static int vorbis_parseheader(vorbis_info *vi,vorbis_comment *vc,ogg_packet *op, int *UseByte){
  oggpack_buffer opb;
  int	ret;

  if(op){
    int size;
    oggpack_readinit(&opb,op->packet);
		size = opb.bitsLeftInSegment;

    /* Which of the three types of header is this? */
    /* Also verify header-ness, vorbis */
    {
      char buffer[6];
      int packtype=oggpack_read(&opb,8);
      memset(buffer,0,6);
      readstring(&opb,buffer,6);

      if(memcmp(buffer,"vorbis",6)){
        /* not a vorbis header */
        return(OV_ENOTVORBIS);
      }
      switch(packtype){
      case 0x01: /* least significant *bit* is read first */
        if(!op->b_o_s){
          /* Not the initial packet */
          return(OV_EBADHEADER);
        }
        if(vi->rate!=0){
          /* previously initialized info header */
          return(OV_EBADHEADER);
        }

        ret = _vorbis_unpack_info(vi,&opb);
        break;

      case 0x03: /* least significant *bit* is read first */
        if(vi->rate==0){
          /* um... we didn't get the initial header */
          return(OV_EBADHEADER);
        }

        ret = _vorbis_unpack_comment(vc,&opb);
        break;

      case 0x05: /* least significant *bit* is read first */
        if(vi->rate==0 || vc->vendor==NULL){
          /* um... we didn;t get the initial header or comments yet */
          return(OV_EBADHEADER);
        }

        ret = _vorbis_unpack_books(vi,&opb);
        break;

      default:
        /* Not a valid vorbis header type */
        return(OV_EBADHEADER);
        break;
      }
    }
	size -= opb.bitsLeftInSegment;
	size >>= 3;
	size += ((opb.bitsLeftInWord) ? 1:0);
	*UseByte = size;

	return ret;
  }
  return(OV_EBADHEADER);
}

/*5.1ch : L,C,R,LS,RS,lfe*/
const int VORBIS_6CH_2[2*8] = {
 0x6229, 0x4568, 0x0, 0x53b1, 0x334a, 0x1e40, 0x0, 0x0,
 0x0, 0x4568, 0x6229, 0x334a, 0x53b1, 0x1e40, 0x0, 0x0,
};

#define  SAT_SHORT(lSamp,Out)             \
    if(lSamp > 32767)    Out = 0x7fff;      \
    else if(lSamp < -32768)   Out = 0x8000;   \
    else  Out = (short)lSamp;

static void down_mix_to_stereo(int samples, short *pcm, int inCh, adec_output_t *p_adec_out)
{
  int i, outoffset;
  short *L,*R; 
  int   *CoeffL,*CoeffR;
  short *tempPcm = pcm;
  int temp;
  
  L = (short *)p_adec_out->m_pvChannel[0];
  R = L + 1;
  outoffset = 2;

  CoeffL = (int *)&VORBIS_6CH_2[0];
  CoeffR = (int *)&VORBIS_6CH_2[8];

  for(i=0;i<samples;i++)
  {
    // to make L
    temp = (((TCAS_S64)(tempPcm[0] * CoeffL[0])  +   (TCAS_S64)(tempPcm[1] * CoeffL[1]) + 
                /*  (TCAS_S64)(tempPcm[2] * CoeffL[2])  + */(TCAS_S64)(tempPcm[3] * CoeffL[3]) + 
                    (TCAS_S64)(tempPcm[4] * CoeffL[4])  +   (TCAS_S64)(tempPcm[5] * CoeffL[5]))>>15);
    SAT_SHORT(temp, *L);
    // to make R
    temp = (((TCAS_S64)(tempPcm[2] * CoeffR[2])  +   (TCAS_S64)(tempPcm[1] * CoeffR[1]) + 
                /*  (TCAS_S64)(tempPcm[0] * CoeffR[0])  + */(TCAS_S64)(tempPcm[3] * CoeffR[3]) + 
                    (TCAS_S64)(tempPcm[4] * CoeffR[4])  +   (TCAS_S64)(tempPcm[5] * CoeffR[5]))>>15);
    SAT_SHORT(temp, *R);
    L += outoffset;
    R += outoffset;
    tempPcm += 6;    
  }
}


static int convert_vorbis_to_cdk_pcm(int inCh, int downMix, int samples, short *pcm, adec_output_t *p_adec_out)
{
  int out_ch = 2;

  if((inCh == 6) && (downMix))  //20110908 Helena,support only 5.1ch -> 2ch, 
  {
    if(samples > 0) 
      down_mix_to_stereo(samples, pcm,inCh,p_adec_out);

    out_ch = 2;
  }
  else
  {
    int ii,j;
    out_ch = inCh;

    if(samples > 0)
    {
      memcpy(p_adec_out->m_pvChannel[0], pcm, sizeof(short) * out_ch * samples);
    }
  }
  
  return out_ch;
}
