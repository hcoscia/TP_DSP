/**
  ******************************************************************************
  * @file    application.c 
  * @author  Gustavo Muro
  * @version V0.0.1
  * @date    30/05/2015
  * @brief   Archivo de aplicación.
  ******************************************************************************
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its
  *    contributors may be used to endorse or promote products derived from this
  *    software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "application.h"
#include "ff.h"
#include "waveplayer.h"
#include "waverecorder.h"
#include "ff.h"    
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "main.h"
#include "utils.h"

#include "complexCompFrec.h"
#include "Vectores_SEN_COS.h"
#include <inttypes.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  APPSTATE_IDLE = 0,
  APPSTATE_GEN_SINE,
  APPSTATE_MOUNT_FS,
  APPSTATE_UMOUNT_FS,
  APPSTATE_WRITE,
  APPSTATE_PLAY,
}appState_enum;

typedef enum
{
  SECSTATE_DET659 = 0,
	SECSTATE_DET554,
	SECSTATE_DET440,
}secState_enum;

typedef struct
{
	int16_t *psen440;
	int16_t *pcos440;
	int16_t *psen554;
	int16_t *pcos554;
	int16_t *psen659;
	int16_t *pcos659;
	int16_t Muestras440;
	int16_t Muestras554;
	int16_t Muestras659;
}modoFs_struct;


/* Private define ------------------------------------------------------------*/

#define SINE_GEN_AUDIO_SAMPLE_RATE    8000

#define SINE_GEN_DURATION             10

#define SINE_GEN_1KHZ_LENGTH          (SINE_GEN_AUDIO_SAMPLE_RATE/1000)

#define SINE_GEN_500HZ_LENGTH         (SINE_GEN_AUDIO_SAMPLE_RATE/500)

/* Private variables ---------------------------------------------------------*/
static FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
static char USBDISKPath[4];          /* USB Host logical drive path */
static appState_enum appState = APPSTATE_IDLE;

static secState_enum secState = SECSTATE_DET659;
modoFs_struct modoFs;

static uint8_t usbConnected = 0;

/* Variable used by FatFs*/
static FIL FileRead;
static FIL FileWrite;

static const int16_t sine_1khz_FS8khz[SINE_GEN_1KHZ_LENGTH] =
{
  0, 23169, 32767, 23169, 0, -23169, -32767, -23169
};

static const int16_t sine_500hz_FS8khz[SINE_GEN_500HZ_LENGTH] =
{
  0,12539,23169,30272,32767,30272,23169,12539,0,-12539,-23169,-30272,-32767,-30272,-23169,-12539
};



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int8_t secuencia(int16_t *pBuff)
{
	int8_t ret = 0;
	int64_t Re,Im,E2 = 0;
	int64_t umbral = 5000;
	int8_t shift = 30;
	static int8_t count = 0;
	switch (secState)
	{
		case SECSTATE_DET659:
				//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen659_FS8Khz,(int16_t*)cos659_FS8Khz,Muestras659_8K,&Im,&Re);	
				complexCompFrec_process(pBuff,Nventana,modoFs.psen659,modoFs.pcos659,modoFs.Muestras659,&Im,&Re);
		
				Re >>= shift;
				Im >>= shift;
				
				E2 = Re * Re + Im * Im;
				
				if (umbral<E2)
				{
					BSP_LED_Off(LED3);
					BSP_LED_Toggle(LED4);
					BSP_LED_Off(LED5);
					count++;
//					printf("%d) E2_659 =%" PRId64 "\n",count,E2);
					if (11 == count)
					{
						secState = SECSTATE_DET554;
						BSP_LED_On(LED4);
						count = 0;
					}
				}

			break;
		
		case SECSTATE_DET554:
				//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen554_FS8Khz,(int16_t*)cos554_FS8Khz,Muestras554_8K,&Im,&Re);	
				complexCompFrec_process(pBuff,Nventana,modoFs.psen554,modoFs.pcos554,modoFs.Muestras554,&Im,&Re);
		
				Re >>= shift;
				Im >>= shift;
				
				E2 = Re*Re + Im * Im;
				
				if (umbral<E2)
				{
					BSP_LED_Toggle(LED3);
					BSP_LED_Off(LED5);
					count++;
//					printf("%d) E2_554 =%" PRId64 "\n",count,E2);
					if (11 == count)
					{
						secState = SECSTATE_DET440;
						BSP_LED_On(LED3);
						count = 0;
					}
				}
			break;
		
		case SECSTATE_DET440:
				//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen440_FS8Khz,(int16_t*)cos440_FS8Khz,Muestras440_8K,&Im,&Re);
				complexCompFrec_process(pBuff,Nventana,modoFs.psen440,modoFs.pcos440,modoFs.Muestras440,&Im,&Re);		
	
				Re >>= shift;
				Im >>= shift;
				
				E2 = Re*Re + Im * Im;
				
				if (umbral<E2)
				{
					BSP_LED_Toggle(LED5);
					count++;
//					printf("%d) E2_440 =%" PRId64 "\n",count,E2);
					if (15 == count)
					{
						secState = SECSTATE_DET659;
						BSP_LED_On(LED5);
						count = 0;
					}
				}
			break;
		
		default:
			secState = SECSTATE_DET659;
			break;
	}
	return ret;
}

int32_t getDataCB(int16_t *pBuff, int32_t length)
{
  UINT bytesread = 0;
//############ Test ################	
//  int64_t Re,Im, E440, E554, E659 = 0;
//	
//	//int64_t umbral = 12800; //50 * 256 (no funciona con ruido)
//	int64_t umbral = 5000;
//	int8_t shift = 30;
//	
//##################################	
	
  f_read(&FileRead, pBuff, length*sizeof(int16_t), (void *)&bytesread);

//############ Final ################	
	TickTock_Start();	
		secuencia(pBuff);
	TickTock_Stop();
//###################################	

//############ Test ################
//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen440_FS8Khz,(int16_t*)cos440_FS8Khz,Muestras440_8K,&Im,&Re);
//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen440_FS16Khz,(int16_t*)cos440_FS16Khz,Muestras440_16K,&Im,&Re);
//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen440_FS44_1Khz,(int16_t*)cos440_FS44_1Khz,Muestras440_44_1K,&Im,&Re);

//	if (complexCompFrec_process(pBuff,Nventana,modoFs.psen440,modoFs.pcos440,modoFs.Muestras440,&Im,&Re) == -1)
//		printf("Error en complexCompFrec_process().\n");
//	else
//  {
//		printf("------------------ 440 ------------------\n");
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		Re >>= shift;
//		Im >>= shift;
//		
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		E440 = Re * Re + Im * Im;
//		
//		printf("E440 = %lld\n", (long long) E440);

//		if (umbral<E440)
//		{
//			BSP_LED_Off(LED3);
//			BSP_LED_Off(LED4);
//			BSP_LED_On(LED5);
//		}
//	}
//		
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen554_FS8Khz,(int16_t*)cos554_FS8Khz,Muestras554_8K,&Im,&Re);
//		
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen554_FS16Khz,(int16_t*)cos554_FS16Khz,Muestras554_16K,&Im,&Re);
//		
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen554_FS44_1Khz,(int16_t*)cos554_FS44_1Khz,Muestras554_44_1K,&Im,&Re);
//	
//	if (complexCompFrec_process(pBuff,Nventana,modoFs.psen554,modoFs.pcos554,modoFs.Muestras554,&Im,&Re) == -1)
//		printf("Error en complexCompFrec_process().\n");
//	else
//  {		
//		printf("------------------ 554 ------------------\n");
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		Re >>= shift;
//		Im >>= shift;
//		
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		E554 = Re*Re + Im * Im;
//		
//		printf("E554 = %lld\n", (long long) E554);
//		
//		if (umbral<E554)
//		{
//			BSP_LED_On(LED3);
//			BSP_LED_Off(LED4);
//			BSP_LED_Off(LED5);
//		}
//	}
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen659_FS8Khz,(int16_t*)cos659_FS8Khz,Muestras659_8K,&Im,&Re);			
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen659_FS16Khz,(int16_t*)cos659_FS16Khz,Muestras659_16K,&Im,&Re);
//	//complexCompFrec_process(pBuff,Nventana,(int16_t*)sen659_FS44_1Khz,(int16_t*)cos659_FS44_1Khz,Muestras659_44_1K,&Im,&Re);
//	
//if (complexCompFrec_process(pBuff,Nventana,modoFs.psen659,modoFs.pcos659,modoFs.Muestras659,&Im,&Re) == -1)
//		printf("Error en complexCompFrec_process().\n");
//	else
//  {
//		printf("------------------ 659 ------------------\n");
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		Re >>= shift;
//		Im >>= shift;
//		
//		printf("%lld,%lld\n",(long long) Re,(long long) Im);
//		
//		E659 = Re*Re + Im * Im;
//		
//		printf("E659 = %lld\n", (long long) E659);
//		
//		if (umbral<E659)
//		{
//			BSP_LED_Off(LED3);
//			BSP_LED_On(LED4);
//			BSP_LED_Off(LED5);
//		}
//		
//		

//	}		
//	//printf("%" PRId64 ",%" PRId64 ",%" PRId64 "\n",(E659/1073676289),(E554/1073676289),(E440/1073676289));
//		//printf("%" PRId64 ",%" PRId64 ",%" PRId64 "\n",E659,E554,E440);
//		printf("################################################################\n");
//		printf("%lld,%lld,%lld\n",(long long)E659,(long long)E554,(long long)E440);
//		printf("################################################################\n");
////##################################	

	
  return bytesread;
}

int32_t getDataSineCB(int16_t *pBuff, int32_t length)
{
  static int8_t count = 0;
  int32_t ret = length * 2;
  
  TickTock_Start();
  
  while (length)
  {
    *pBuff = sine_500hz_FS8khz[count];
    count++;
    if (SINE_GEN_500HZ_LENGTH <= count)
    {
      count = 0;
    }
    pBuff++;
    length--;
  }
  
  TickTock_Stop();
  
  return ret;
}


/* Exported functions ------------------------------------------------------- */

extern void application_init(void)
{
  /*##-1- Link the USB Host disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) != 0)
  {
    Error_Handler();
  }
  
  TickTock_Init();
}

extern void application_task(void)
{
  UINT bytesread = 0;
  WAVE_FormatTypeDef waveformat;
  
  switch (appState)
  {
    case APPSTATE_IDLE:
      if (usbConnected)
      {
        appState = APPSTATE_MOUNT_FS;
      }
      break;
    
    case APPSTATE_GEN_SINE:
      waveformat.SampleRate = SINE_GEN_AUDIO_SAMPLE_RATE;
      waveformat.FileSize = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * \
                            sizeof(int16_t) + sizeof(WAVE_FormatTypeDef);
      waveformat.NbrChannels = CHANNEL_MONO;
      WavePlayerStart(waveformat, getDataSineCB, 70);
      break;
    
    case APPSTATE_MOUNT_FS:
      if (f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) 
      {
        /* FatFs initialization fails */
        Error_Handler();
      }
      else
      {
        appState = APPSTATE_PLAY;
      }
      break;
    
    case APPSTATE_UMOUNT_FS:
      f_mount(NULL, (TCHAR const*)"", 1);
      appState = APPSTATE_IDLE;
      break;
    
    case APPSTATE_WRITE:
      if (f_open(&FileWrite, WAVE_NAME_COMPLETO, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        waveformat.SampleRate = SINE_GEN_AUDIO_SAMPLE_RATE;
        waveformat.FileSize = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * \
                              sizeof(int16_t) + sizeof(WAVE_FormatTypeDef);
        waveformat.NbrChannels = WAVE_CHANNEL_MONO;
        waveformat.ByteRate = SINE_GEN_AUDIO_SAMPLE_RATE * WAVE_CHANNEL_MONO * sizeof(int16_t);
        waveformat.BitPerSample = __REV16(WAVE_16_BIT_PER_SAMPLE);
        waveformat.SubChunk2Size = SINE_GEN_AUDIO_SAMPLE_RATE * SINE_GEN_DURATION * sizeof(int16_t);
      
        WaveRecord(&FileWrite, waveformat, getDataSineCB);
        f_close(&FileWrite);
        appState = APPSTATE_PLAY;
      }
      break;

    case APPSTATE_PLAY:
      if (f_open(&FileRead, WAVE_NAME_COMPLETO, FA_READ) != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        /* Read sizeof(WaveFormat) from the selected file */
        f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
				
				switch (waveformat.SampleRate)
				{
					case 8000:
							modoFs.psen440 = (int16_t*) &sen440_FS8Khz[0];
							modoFs.pcos440 = (int16_t*) &cos440_FS8Khz[0];
							modoFs.psen554 = (int16_t*) &sen554_FS8Khz[0];
							modoFs.pcos554 = (int16_t*) &cos554_FS8Khz[0];
							modoFs.psen659 = (int16_t*) &sen659_FS8Khz[0];
							modoFs.pcos659 = (int16_t*) &cos659_FS8Khz[0];
							modoFs.Muestras440 = Muestras440_8K ;
							modoFs.Muestras554 = Muestras554_8K ;
							modoFs.Muestras659 = Muestras659_8K ;
						break;
					case 16000:
							modoFs.psen440 = (int16_t*) &sen440_FS16Khz[0];
							modoFs.pcos440 = (int16_t*) &cos440_FS16Khz[0];
							modoFs.psen554 = (int16_t*) &sen554_FS16Khz[0];
							modoFs.pcos554 = (int16_t*) &cos554_FS16Khz[0];
							modoFs.psen659 = (int16_t*) &sen659_FS16Khz[0];
							modoFs.pcos659 = (int16_t*) &cos659_FS16Khz[0];
							modoFs.Muestras440 = Muestras440_16K ;
							modoFs.Muestras554 = Muestras554_16K ;
							modoFs.Muestras659 = Muestras659_16K ;
						break;
					case 44100:
							modoFs.psen440 = (int16_t*) &sen440_FS44_1Khz[0];
							modoFs.pcos440 = (int16_t*) &cos440_FS44_1Khz[0];
							modoFs.psen554 = (int16_t*) &sen554_FS44_1Khz[0];
							modoFs.pcos554 = (int16_t*) &cos554_FS44_1Khz[0];
							modoFs.psen659 = (int16_t*) &sen659_FS44_1Khz[0];
							modoFs.pcos659 = (int16_t*) &cos659_FS44_1Khz[0];
							modoFs.Muestras440 = Muestras440_44_1K ;
							modoFs.Muestras554 = Muestras554_44_1K ;
							modoFs.Muestras659 = Muestras659_44_1K ;
						break;
				}
				
        WavePlayerStart(waveformat, getDataCB, 70);
        f_close(&FileRead);
      }
      break;
    
    default:
      appState = APPSTATE_IDLE;
      break;
  }
}

extern void application_conect(void)
{
  usbConnected = 1;
}
extern void application_disconect(void)
{
  usbConnected = 0;
}

/* End of file ---------------------------------------------------------------*/

