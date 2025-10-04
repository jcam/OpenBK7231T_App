#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"

#include "drv_bp5758d.h"

static byte g_chosenCurrent_rgb = BP5758D_14MA;
static byte g_chosenCurrent_cw = BP5758D_14MA;

static softI2C_t g_softI2C;
static uint16_t g_color_uint10[5];

#define CONVERT_CURRENT_BP5758D(curVal) (curVal>63) ? (curVal+34) : curVal;

static void BP5758D_SetCurrent(byte curValRGB, byte curValCW) {
	// here is a conversion from human-readable format to BP's format
	g_chosenCurrent_rgb = CONVERT_CURRENT_BP5758D(curValRGB);
	g_chosenCurrent_cw = CONVERT_CURRENT_BP5758D(curValCW);
	BP5758D_WriteAll();
}

void BP5758D_Write(float *rgbcw) {

	ADDLOG_DEBUG(LOG_FEATURE_DRV, "Write Requested: %f %f %f %f %f", rgbcw[0], rgbcw[1], rgbcw[2], rgbcw[3], rgbcw[4]);

	for(int i = 0; i < 5; i++){
		// convert 0-255 to 0-1023
		g_color_uint10[i] = MAP(GetRGBCW(rgbcw, g_cfg.ledRemap.ar[i]), 0, 255.0f, 0, 1023.0f);
		//g_color_uint10[i] = (uint16_t)GetRGBCW(rgbcw, g_cfg.ledRemap.ar[i]) * 4;
	}
	BP5758D_WriteAll();
}

void BP5758D_WriteAll() {
	uint8_t data[17];
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "Writing to Lamp: %i %i %i %i %i", g_color_uint10[0], g_color_uint10[1], g_color_uint10[2], g_color_uint10[3], g_color_uint10[4]);

#if WINDOWS
	void Simulator_StoreBP5758DColor(unsigned short *data);
	Simulator_StoreBP5758DColor(g_color_uint10);
#endif

	// If we receive 0 for all channels, we'll assume that the lightbulb is off, and activate BP5758d's sleep mode.
	if (g_color_uint10[0]==0 && g_color_uint10[1]==0 && g_color_uint10[2]==0 && g_color_uint10[3]==0 && g_color_uint10[4]==0) {
		for (int i = 1; i < 17; i++)
			data[i] = 0;

		//Set all currents/channels to zero
		data[0] = BP5758D_ADDR_OUT;
		BP5758D_WriteBuffer(data, 17);

		//Sleep
		data[0] = BP5758D_ADDR_SLEEP;
		BP5758D_WriteBuffer(data, 17);
		return;
	}

	// different start_addr positions depending on output mode (RGBCW, RGB, CW)
	if (g_color_uint10[0] == 0 && g_color_uint10[1] == 0 && g_color_uint10[2] == 0 && (g_color_uint10[3] > 0 || g_color_uint10[4] > 0)) {
		data[0] = BP5758D_ADDR_OUT_2CH;
		data[1] = BP5758D_ENABLE_OUTPUTS_ALL;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = g_chosenCurrent_cw;
		data[6] = g_chosenCurrent_cw;
	} else if ((g_color_uint10[0] > 0 || g_color_uint10[1] > 0 || g_color_uint10[2] > 0) && g_color_uint10[3] == 0 && g_color_uint10[4] == 0) {
		data[0] = BP5758D_ADDR_OUT_3CH;
		data[1] = BP5758D_ENABLE_OUTPUTS_ALL;
		data[2] = g_chosenCurrent_rgb;
		data[3] = g_chosenCurrent_rgb;
		data[4] = g_chosenCurrent_rgb;
		data[5] = 0;
		data[6] = 0;
	} else {
		data[0] = BP5758D_ADDR_OUT;
		data[1] = BP5758D_ENABLE_OUTPUTS_ALL;
		data[2] = g_chosenCurrent_rgb;
		data[3] = g_chosenCurrent_rgb;
		data[4] = g_chosenCurrent_rgb;
		data[5] = g_chosenCurrent_cw;
		data[6] = g_chosenCurrent_cw;
	}

	// Brigtness values are transmitted as two bytes. The light-bulb accepts a 10-bit integer (0-1023) as an input value.
	// The first 5bits of this input are transmitted in second byte, the second 5bits in the first byte.
	for (int i = 7, j = 0; i <= 15; i += 2, j++) {
		data[i] = g_color_uint10[j] & 0x1F;
		data[i + 1] = (g_color_uint10[j] >> 5) & 0x1F;
	}
	BP5758D_WriteBuffer(data, 17);
}

// drv_soft_i2c doesn't follow the i2c needs of BP5758D, so manage it here
static void BP5758D_PreInit() {
	HAL_PIN_Setup_Output(g_softI2C.pin_data);
	HAL_PIN_SetOutputValue(g_softI2C.pin_data, false);
	usleep(BP5758D_DELAY);
	HAL_PIN_Setup_Output(g_softI2C.pin_clk);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, false);
	usleep(BP5758D_DELAY);
}

static void BP5758D_WriteBit(bool value) {
	HAL_PIN_SetOutputValue(g_softI2C.pin_data, value);
	usleep(BP5758D_DELAY);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, true);
	usleep(BP5758D_DELAY);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, false);
	usleep(BP5758D_DELAY);
}

static void BP5758D_WriteByte(uint8_t data) {
	for (uint8_t mask = 0x80; mask; mask >>= 1) {
		BP5758D_WriteBit(data & mask);
	}

	//clear the ack (ignore the response)
	HAL_PIN_Setup_Input(g_softI2C.pin_data);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, true);
	usleep(BP5758D_DELAY);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, false);
	usleep(BP5758D_DELAY);
	HAL_PIN_Setup_Output(g_softI2C.pin_data);
}

static void BP5758D_WriteBuffer(uint8_t *buffer, uint8_t size) {
	HAL_PIN_SetOutputValue(g_softI2C.pin_data, false);
	usleep(BP5758D_DELAY);
	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, false);
	usleep(BP5758D_DELAY);

	for (uint8_t i = 0; i < size; i++) {
		BP5758D_WriteByte(buffer[i]);
	}

	HAL_PIN_SetOutputValue(g_softI2C.pin_clk, true);
	usleep(BP5758D_DELAY);
	HAL_PIN_SetOutputValue(g_softI2C.pin_data, true);
	usleep(BP5758D_DELAY);
}


// see drv_bp5758d.h for sample values
// Also see here for Datasheet table:
// https://user-images.githubusercontent.com/19175445/193464004-d5e8072b-d7a8-4950-8f06-118c01796616.png
// https://imgur.com/a/VKM6jOb
static commandResult_t BP5758D_Current(const void *context, const char *cmd, const char *args, int flags){
	byte valRGB, valCW;
	Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	valRGB = Tokenizer_GetArgInteger(0);
	valCW = Tokenizer_GetArgInteger(1);
	// reinit bulb
	BP5758D_SetCurrent(valRGB,valCW);
	return CMD_RES_OK;
}

// startDriver BP5758D
// BP5758D_RGBCW FF00000000
//
// to init a current value at startup - short startup command
// backlog startDriver BP5758D; BP5758D_Current 14 14; 
void BP5758D_Init() {
	// default setting (applied only if none was applied earlier)
	CFG_SetDefaultLEDRemap(0, 1, 2, 3, 4);

	g_softI2C.pin_clk = PIN_FindPinIndexForRole(IOR_BP5758D_CLK,g_softI2C.pin_clk);
	g_softI2C.pin_data = PIN_FindPinIndexForRole(IOR_BP5758D_DAT,g_softI2C.pin_data);

	BP5758D_PreInit();

	//cmddetail:{"name":"BP5758D_RGBCW","args":"[HexColor]",
	//cmddetail:"descr":"Don't use it. It's for direct access of BP5758D driver. You don't need it because LED driver automatically calls it, so just use led_basecolor_rgb",
	//cmddetail:"fn":"CMD_LEDDriver_WriteRGBCW","file":"driver/drv_bp5758d.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("BP5758D_RGBCW", CMD_LEDDriver_WriteRGBCW, NULL);

	//cmddetail:{"name":"BP5758D_Map","args":"[Ch0][Ch1][Ch2][Ch3][Ch4]",
	//cmddetail:"descr":"Maps the RGBCW values to given indices of BP5758D channels. This is because BP5758D channels order is not the same for some devices. Some devices are using RGBCW order and some are using GBRCW, etc, etc. Example usage: BP5758D_Map 0 1 2 3 4",
	//cmddetail:"fn":"CMD_LEDDriver_Map","file":"driver/drv_bp5758d.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("BP5758D_Map", CMD_LEDDriver_Map, NULL);

	//cmddetail:{"name":"BP5758D_Current","args":"[MaxCurrentRGB][MaxCurrentCW]",
	//cmddetail:"descr":"Sets the maximum current limit for BP5758D driver, first value is for rgb and second for cw",
	//cmddetail:"fn":"BP5758D_Current","file":"driver/drv_bp5758d.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("BP5758D_Current", BP5758D_Current, NULL);

	// alias for LED_Map. In future we may want to migrate totally to shared LED_Map command.... 
	CMD_CreateAliasHelper("LED_Map", "BP5758D_Map");
}
