
#include "diskio.h"
#include "ff.h"

#define ADC1_DR_Address ( (uint32_t) 0x4001244C )
#define ADC_NUM         ( 4096 )                  // unsigned short
#define ADC_NUM_DIV_2   ( ADC_NUM/2 )

#define FILE_NAME "0:e.m"

// cycle-buffer  -- 8kB
unsigned short  gl_adc_buff[ADC_NUM];
unsigned short* p_beg_adc_buff;

FATFS fs;       // main FAT_FS struct
FIL file;       // file object
unsigned int must_write;
unsigned int res;
unsigned int first_half;  

void init( void );
void start( void );

void         init_TIM5( void );
unsigned int init_sd( void );
unsigned int write( const void* data, unsigned int num );
unsigned int read( void* buf, unsigned int num );
  
//точка входа
//=======================================================================================
int main( void )
{ 
  first_half = 1;
  
  init_TIM5();
  res = init_sd();
  
  if( res )
  { 
    return 0; 
  }
  
  start();
  
  while( 1 )
  {
    if( must_write )
    {     
      if( first_half )
      {
        first_half = 0;
        write( p_beg_adc_buff, 4096 );
      }
      else
      {
        first_half = 1;
        write( p_beg_adc_buff + ADC_NUM_DIV_2, 4096 );
      }          
      
      must_write = 0;
    }  
  }
}

// initialization of sd thread
// return 0, if all is ok
//==============================================================================
unsigned int init_sd( void )
{
  DSTATUS card_status = 0;
  FRESULT res = FR_OK; 
  
  card_status = disk_initialize( 0 );
  if( card_status )
  {
    return 1;
  }

  res = f_mount( 0, &fs );   // mounts disk 0 with fs           
  if( res )
  {
    return 2;    
  }
 
  // opens/creates file with name FILE_NAME
  // FA_OPEN_ALWAYS - Opens the file if it is existing. If not, a new file is created.
  // FA_WRITE | FA_READ - Data can be read/written from/to the file.
  res = f_open( &file, FILE_NAME, FA_WRITE /*| FA_READ */| FA_OPEN_ALWAYS ); 
  if( res )
  {
    return 3;    
  } 
 
  FILINFO fil_info;
  res = f_stat( FILE_NAME, &fil_info );
  
  return 0;
}

// data - pointer to data to be written on sd-card
// num - size of data in bytes !!!
// return 0, if all is ok
//==============================================================================
unsigned int write( const void* data, unsigned int num )
{
  UINT len;                  // len will storage number of real written bytes
  FRESULT res; 

  // writes num bytes of data to file
  res = f_write( &file, data, num, &len );
  
  if( ( res ) || ( num != len ) )  //if some error was occured
  {
    return 1;    
  } 
  
  return f_sync( &file ); // flushes the cached information of a writing file 
}

// data - pointer to data to be read from sd-card
// num - size of buffer in bytes !!!
// return 0, if all is ok
//==============================================================================
unsigned int read( void* buf, unsigned int num )
{
  UINT len = 0;
  FRESULT res = FR_OK;
  
  res = f_read( &file, buf, num, &len );
  
  if( res || ( len != num ) )
    return 1;
  
  return 0;
}

//==============================================================================
//==============================================================================
//==============================================================================


// initialize TIM5 for FAT_FS purpose
//==============================================================================
void init_TIM5( void )
{
  NVIC_InitTypeDef          NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStruct;
   
  //socket_cp_init(); 
    
  // Enable TIM5 clocks
  RCC->APB1ENR |=  0x08;
  
  // necessary time delay - 10ms
  // 10 ms <-> 100 ips (interrupts per second)
  // PCLK1 = 36 МГц
  // CK_PSC = 2*PCLK1 = 72 MGz
  // number of ips = CK_PSC / (PSC*CNT)
  // ==>
  // 100 ips = 72 * 10^6 / (1000*720)
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;            
  TIM_TimeBaseInitStruct.TIM_Period = 720;   // прерывания 100 раз в секунду
  TIM_TimeBaseInitStruct.TIM_Prescaler = 999; // 1000 - 1
  TIM_TimeBaseInit( TIM5, &TIM_TimeBaseInitStruct );
   
  // Enable the TIM5 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14; // 14 or 1101 1111 > configMAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure ); 

  // TIM5 enable counter
  TIM_Cmd( TIM5, ENABLE );
  
  // Enable TIM5 Update interrupt
  TIM_ITConfig( TIM5, TIM_IT_Update, ENABLE );
}

// deinitialize TIM5 for FAT_FS purpose
//==============================================================================
void de_init_TIM5( void )
{  
  // Disable TIM5 Update interrupt
  TIM_ITConfig( TIM5, TIM_IT_Update, DISABLE );
    
  // TIM5 disable counter
  TIM_Cmd( TIM5, DISABLE );
  
  // Disable TIM5 clocks
  RCC->APB1ENR &= ~0x08;
}

// 
//==============================================================================
void start( void )
{
  init();
}

// initialization of TIM3 and ADC1 for measuring 
//==============================================================================
void init( void )
{
  // takes pointer to global adc buffer
  p_beg_adc_buff = gl_adc_buff;
  
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStruct;
  ADC_InitTypeDef           ADC_InitStructure;
  DMA_InitTypeDef           DMA_InitStructure;
  GPIO_InitTypeDef          GPIO_InitStructure;
  NVIC_InitTypeDef          NVIC_InitStruct;
  
  // common settings ------------------------------------
  
  // setting ADCCLK to 12MHz
  RCC_ADCCLKConfig( RCC_PCLK2_Div6 ); 
  
  // Enable DMA1 clock
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

  // Enable GPIOC and ADC1 clock 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE );
  
  // Configure PC.00, PC.01, PC.02 and PC.03 (ADC Channel 10, 11, 12 and 13)
  // as analog inputs 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init( GPIOC, &GPIO_InitStructure );
                          
  // ADC 1 init------------------------------
  
  // ADC_Channel_10 - IoutCh1 (first in regular group)
  // ADC_Channel_11 - VoutCh1
  // ADC_Channel_12 - IoutCh2 
  // ADC_Channel_13 - VoutCh2
  
  // ADCCLK = 12MHz
  // amount of convertion cycles
  // (for four channels, with sample time = 13.5 cycles) equal 
  // (13.5 + 12.5)*4 = 104 cycles
  // it takes 104 * (1/12)*10^-6 (s) ~ 8.6 mks
  // i.e ADC have ~1.4 mks rest (if TIM3 generates TRGO with 100kHz frequency)
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 4;
  ADC_Init( ADC1, &ADC_InitStructure );

  // ADC1 regular channels configuration  
  ADC_RegularChannelConfig( ADC1, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5 );
  ADC_RegularChannelConfig( ADC1, ADC_Channel_11, 2, ADC_SampleTime_13Cycles5 );
  ADC_RegularChannelConfig( ADC1, ADC_Channel_12, 3, ADC_SampleTime_13Cycles5 );
  ADC_RegularChannelConfig( ADC1, ADC_Channel_13, 4, ADC_SampleTime_13Cycles5 );

  // Enable ADC1 external trigger conversion 
  ADC_ExternalTrigConvCmd( ADC1, ENABLE );

  // Enable ADC1 DMA 
  ADC_DMACmd( ADC1, ENABLE );
  
  // Enable ADC1 
  ADC_Cmd( ADC1, ENABLE );  

  // Enable ADC1 reset calibration register   
  ADC_ResetCalibration( ADC1 );
  // Check the end of ADC1 reset calibration register 
  while( ADC_GetResetCalibrationStatus(ADC1) );

  // Start ADC1 calibration 
  ADC_StartCalibration( ADC1 );
  // Check the end of ADC1 calibration 
  while( ADC_GetCalibrationStatus( ADC1 ) );  
  
  //DMA1 init -------------------------------------------------
  
  //сбрасываем флаг прерывания global interrupt 1 канала DMA 1
  DMA_ClearITPendingBit( DMA1_IT_GL1 );
    
  DMA_DeInit( DMA1_Channel1 );
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&gl_adc_buff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC_NUM;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init( DMA1_Channel1, &DMA_InitStructure );
  
  //Настройка NVIC для DMA IRQ 
  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 12;  // 12 > configMAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStruct ); 
  
  DMA_ITConfig( DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE );
  DMA_ITConfig( DMA1_Channel1, DMA_IT_TE, DISABLE );
  
  // Enable DMA1 channel1 
  DMA_Cmd( DMA1_Channel1, ENABLE );
  
  // TIM 3 init------------------------------
  
  // PCLK1 = 36 МГц
  // F_interrupt = PCLK1*2 / (TIM_Prescaler + 1) / TIM_Period;

  // Enable TIM3 clocks
  RCC->APB1ENR |=  0x02;
  
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;            
  TIM_TimeBaseInitStruct.TIM_Period = 720-1;   // прерывания 50 000 раз в секунду(720)
  TIM_TimeBaseInitStruct.TIM_Prescaler = 1; // 2 - 1
  TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
  
  //выбираем в качестве источника внешнего тригера(TRGO) update event
  TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update ); 
  
  // TIM3 enable counter
  TIM_Cmd( TIM3, ENABLE );
  
  // Disable TIM3 Update interrupt
  TIM_ITConfig( TIM3, TIM_IT_Update, DISABLE );
  
  // after this line, TIM3-ADC1-DMA1 band will start work
}