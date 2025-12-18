#include "LISXXXALH.h"
extern ADC_HandleTypeDef hadc1;
extern volatile uint16_t raw_LISXXXALH[6];

/* Variables */
static ADC_ChannelConfTypeDef _sConfig = { .Rank = ADC_REGULAR_RANK_1,.SamplingTime = ADC_SAMPLETIME_5CYCLE,.SingleDiff = ADC_SINGLE_ENDED,
.OffsetNumber = ADC_OFFSET_NONE,.Offset = 0 }; // 12.11.2024

/* _sConfig.Channel set functions – kept the same */
void ADC_Select_CH1(void) { _sConfig.Channel = ADC_CHANNEL_1; }
void ADC_Select_CH2(void) { _sConfig.Channel = ADC_CHANNEL_2; }
void ADC_Select_CH3(void) { _sConfig.Channel = ADC_CHANNEL_3; }
void ADC_Select_CH4(void) { _sConfig.Channel = ADC_CHANNEL_4; }
void ADC_Select_CH5(void) { _sConfig.Channel = ADC_CHANNEL_5; }
void ADC_Select_CH6(void) { _sConfig.Channel = ADC_CHANNEL_6; }

/* New part : DMA buffer + latest sample mirror */
#define NUM_CH 6
/* Two-full-scans buffer: half complete ISR after 6 samples, complete after 12*/
static __attribute__((aligned(32))) uint16_t adc_dma_buf[NUM_CH * 2];
static volatile uint16_t adc_latest[NUM_CH];
static volatile uint32_t adc_overrun_cnt = 0;

/* One time init: configure scan + DMA circular and start it */
static void ADC_Config_AllRanks_Once(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop(&hadc1);

	/*Configure 1-6 channels once, not per call*/
	const uint32_t chans[NUM_CH] = {
		ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
		ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6 };

	for (uint32_t i = 0; i < NUM_CH; ++i) {
		_sConfig.Channel = chans[i];
		_sConfig.Rank = ADC_REGULAR_RANK_1 + i;
		HAL_ADC_ConfigChannel(&hadc1, &_sConfig);
	}

	/* Start DMA in circular mode: buffer has two scans (2*6 samples) */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, NUM_CH * 2);
	HAL_ADC_Start(&hadc1);
}
/* Call once at startup */
void analogSensor_start_continuous(void)
{
	for (uint32_t i = 0; i < NUM_CH; ++i) {
		adc_latest[i] = 0;
		raw_LISXXXALH[i] = 0;
	}
	adc_overrun_cnt = 0;
	ADC_Config_AllRanks_Once();
}

/* DMA callbacks: update adc_latest[] with the newest scan */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	If(hadc != &hadc1) return;
	/* First half contains a full scan */
	for (uint32_t ch = 0; ch < NUM_CH; ++ch) {
		adc_latest[ch] = adc_dma_buf[ch];
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc != &hadc1) return;
	/* Second half contains the next full scan */
	uint16_t *p = &adc_dma_buf[NUM_CH];
	for (uint32_t ch = 0; ch < NUM_CH; ++ch) {
		adc_latest[ch] = p[ch];
	}
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	If(hadc == &hadc1) {
		if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR)) {
			adc_overrun_cnt++;
			__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
		}
	}
}

/* The original function in non blocking mode */
void analogSensor_operation(uint8_t snsrID)
{
	If(snsrID == 0) { ADC_Select_CH1(); }
	else if (snsrID == 1) { ADC_Select_CH2(); }
	else if (snsrID == 2) { ADC_Select_CH3(); }
	else if (snsrID == 3) { ADC_Select_CH4(); }
	else if (snsrID == 4) { ADC_Select_CH5(); }
	else { ADC_Select_CH6(); }

	if (snsrID < NUM_CH) {
		raw_LISXXXALH[snsrID] = adc_latest[snsrID];
	}
}
