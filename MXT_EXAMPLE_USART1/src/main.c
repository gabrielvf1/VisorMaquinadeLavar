/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#include "asf.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ioport.h"
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "tfont.h"

//Testar
 #include "icones/powerbuttonoff.h"
 #include "icones/powerbuttonon.h"
 #include "icones/botao_next.h"
 #include "icones/botao_previous.h"
 #include "icones/unlocked.h"
 #include "icones/locked.h"
 #include "icones/trabalhando1.h"
 #include "icones/trabalhando2.h"

typedef struct {
	const uint8_t *data;
	uint16_t width;
	uint16_t height;
	uint8_t dataSize;
} tImage2;

#include "maquina1.h"
#include "calibri_36.h"

#define YEAR        2019
#define MOUNTH      4
#define DAY         8
#define WEEK        16
#define HOUR        17
#define MINUTE      16
#define SECOND      0

#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define LED_PIO       PIOA
#define LED_PIO_ID    ID_PIOA
#define LED_IDX       0
#define LED_IDX_MASK  (1u << LED_IDX)

#define BUT1_PIO      PIOC
#define BUT1_PIO_ID   ID_PIOC
#define BUT1_IDX  31
#define BUT1_IDX_MASK (1 << BUT1_IDX)

*define CLICOU 0x20
*define SOLTOU_1 0x84
*define SOLTOU_2 0x94
*define SOLTOU_3 0xc0
	
struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;
volatile Bool flag_lock = false;
volatile Bool flag_ligar = false;
volatile Bool porta_flag=false;
volatile Bool flag_porta_lock = false;
volatile Bool flag_apaga_tela = false;
bool locking_flag=0;
unsigned int hora;
unsigned int minuto, segundo;
unsigned int segundo_lock = 0;
int lock_seg;
t_ciclo *p_primeiro;
char tempo_string[32],segundo_string[32],minuto_string[32],hora_string[32],tempo_total_seg_string[32],tempo_total_min_string[32],tempo_total_hora_string[32];
int tempo_total_seg = 0;
int tempo_total_min = 0;
int tempo_total_hora = 0;
int contador_icone = 0;
int segundo_cd = 0;
int minuto_cd = 0;
int hora_cd = 0;
 
void io_init(void);

void RTC_init(void);
	
static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

t_ciclo *initMenuOrder(){
  c_rapido.previous = &c_enxague;
  c_rapido.next = &c_diario;

  c_diario.previous = &c_rapido;
  c_diario.next = &c_pesado;

  c_pesado.previous = &c_diario;
  c_pesado.next = &c_enxague;

  c_enxague.previous = &c_pesado;
  c_enxague.next = &c_centrifuga;

  c_centrifuga.previous = &c_enxague;
  c_centrifuga.next = &c_rapido;

  return(&c_diario);
}
apagaga_tempo(){
	
}
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		if(flag_ligar){
			segundo_cd-=1;
			if (segundo_cd==9){
				flag_apaga_tela = true;
			}
			if(segundo_cd<0){
				minuto_cd-=1;
				segundo_cd=59;
				if(minuto_cd<=1){
					hora_cd-=1;
					minuto_cd=59;
				}
			}
		}
		segundo +=1;
		segundo_lock += 1;
		if(segundo >= 59){
			minuto+=1;
			segundo = 0;
			if(minuto >=59){
				hora+=1;
				minuto = 0;
			}		
		}
	}	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	}
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_SECEN);

}

void write_text(int x, int y,  int tamanho_max_x, int tamanho_max_y , char *text){
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(x-2, y-2, tamanho_max_x+2, tamanho_max_y+2);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_string(x,y,text);
	
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	}

void draw_button_liga(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	if(!clicked) {
		write_text(2,2,150,15, "Desligado");
		ili9488_draw_pixmap(76, 30, powerbuttonoff.width, powerbuttonoff.height, powerbuttonoff.data);
	} else {
		write_text(2,2,150,15, "Ligado");
		write_text(40,360,320,380, "Lavagem em Progresso");
		ili9488_draw_pixmap(76, 30, powerbuttonon.width, powerbuttonon.height, powerbuttonon.data);
	}
	last_state = clicked;
}

void draw_button_lock(uint32_t flag) {
	static uint32_t last_state = 255; // undefined
	if(flag == last_state) return;
	
	if(flag) {
		write_text(220,2,320,15, "Locked");
		ili9488_draw_pixmap(240, 25, locked.width, locked.height, locked.data);
		} else {
		write_text(220,2,320,15, "Unlocked");
		ili9488_draw_pixmap(240, 25, unlocked.width, unlocked.height, unlocked.data);
	}
	last_state = flag;
}

void draw_button_cicle_previus(uint32_t flag) {
	static uint32_t last_state = 255; // undefined
	if(flag == last_state) return;
	if(flag) {
			
			p_primeiro = p_primeiro->previous;
			write_text(90,94,234,105, p_primeiro->nome);
			
			tempo_total_min = (p_primeiro->enxagueTempo*p_primeiro->enxagueQnt)+p_primeiro->centrifugacaoTempo;
			tempo_total_hora = (tempo_total_min / 60);
			tempo_total_min = tempo_total_min % 60;
			
			sprintf(tempo_total_seg_string,"%d",tempo_total_seg);
			write_text(290,216,315,232, tempo_total_seg_string);
					
			sprintf(tempo_total_min_string,"%d",tempo_total_min);
			write_text(258,216,281,232, tempo_total_min_string);
			write_text(282,216,283,232,":");
			
			sprintf(tempo_total_hora_string,"%d",tempo_total_hora);
			write_text(221,216,246,232, tempo_total_hora_string);
			write_text(246,216,248,232,":");
			ili9488_draw_pixmap(200, 115, p_primeiro->icone->width, p_primeiro->icone->height, p_primeiro->icone->data);
			flag = 0;
		}
	last_state = flag;
}

void draw_button_cicle_next(uint32_t flag) {
	static uint32_t last_state = 255; // undefined
	if(flag == last_state) return;
	if(flag) {
		p_primeiro = p_primeiro->next;
		write_text(90,94,234,105, p_primeiro->nome);
		
		tempo_total_min = (p_primeiro->enxagueTempo*p_primeiro->enxagueQnt)+p_primeiro->centrifugacaoTempo;
		tempo_total_hora = (tempo_total_min / 60);
		tempo_total_min = tempo_total_min % 60;
		
		sprintf(tempo_total_seg_string,"%d",tempo_total_seg);
		write_text(290,216,315,232, tempo_total_seg_string);
		
		sprintf(tempo_total_min_string,"%d",tempo_total_min);
		write_text(258,216,281,232, tempo_total_min_string);
		write_text(282,216,283,232,":");
		
		sprintf(tempo_total_hora_string,"%d",tempo_total_hora);
		write_text(221,216,246,232, tempo_total_hora_string);
		write_text(246,216,248,232,":");
		ili9488_draw_pixmap(200, 115, p_primeiro->icone->width, p_primeiro->icone->height, p_primeiro->icone->data);
		flag = 0;
	}
	last_state = flag;
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}


uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}
void making_border(int x,int y, int x1, int y1){
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_filled_rectangle(x, y, x1, y1);
	
}

void print_tempo(){
	if(flag_ligar){
		if(flag_apaga_tela){
			flag_apaga_tela = false;
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
			ili9488_draw_filled_rectangle(0, 257, 320,294);
		}
		sprintf(segundo_string,"%d",segundo_cd);
		font_draw_text(&calibri_36, segundo_string, 195, 258, 1);
		sprintf(minuto_string,"%d",minuto_cd);
		font_draw_text(&calibri_36, minuto_string, 142, 258, 1);
		font_draw_text(&calibri_36, ":", 185, 258, 1);
		sprintf(hora_string,"%d",hora_cd);
		font_draw_text(&calibri_36, hora_string, 90, 258, 1);
		font_draw_text(&calibri_36, ":", 125, 258, 1);
	}else{
		sprintf(segundo_string,"%d",segundo);
		font_draw_text(&calibri_36, segundo_string, 195, 258, 1);
		sprintf(minuto_string,"%d",minuto);
		font_draw_text(&calibri_36, minuto_string, 142, 258, 1);
		font_draw_text(&calibri_36, ":", 185, 258, 1);
		sprintf(hora_string,"%d",hora);
		font_draw_text(&calibri_36, hora_string, 90, 258, 1);
		font_draw_text(&calibri_36, ":", 125, 258, 1);
	}
}

void but1_callBack(){
	if(!flag_ligar){
		if (flag_porta_lock){
			write_text(2,300,320,320,"Porta Fechada");
			pio_clear(LED_PIO,LED_IDX_MASK);
		}else{
			write_text(2,300,320,320,"Porta Aberta");
			pio_set(LED_PIO,LED_IDX_MASK);
		}
		flag_porta_lock = !flag_porta_lock;	
	}
}

void update_screen(uint32_t tx, uint32_t ty, uint8_t status) {
	if(status==CLICOU && tx >= 20 && tx <= 80 && flag_lock==false  && flag_ligar==false) {
		if(ty >= 115 && ty <= 202 ){			
			draw_button_cicle_previus(1);
		}
	}
	if(status==CLICOU && tx >= 85 && tx <= 145 && flag_lock==false && flag_ligar==false) {
		if(ty >= 115 && ty <= 202 ){
			draw_button_cicle_next(1);			
		}
	}
	if(status==CLICOU && tx >= 76 && tx <= 125 && flag_lock==false && flag_porta_lock == false) {
		if(ty >= 30 && ty <= 78 ){
			flag_ligar = !flag_ligar;
			segundo_cd = tempo_total_seg;
			minuto_cd = tempo_total_min;
			hora_cd = tempo_total_hora;
			if (hora_cd>0){
				minuto_cd = 59;
				segundo_cd = 59;
				tempo_total_hora-=1;
			}else if(tempo_total_min>0){
				segundo_cd=59;
				minuto_cd-=1;
			}
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
			ili9488_draw_filled_rectangle(0, 257, 320,294);
			draw_button_liga(flag_ligar);
		}
	}
	if (tx >= 230 && tx <= 294){
		if(ty >= 20 && ty <= 84 ){
			if ((status ==SOLTOU_3 || status == SOLTOU_1 || status == SOLTOU_2) && locking_flag == false){
				//printf("AAAAAAAA");
				locking_flag = true;
				lock_seg = segundo_lock;
			}
		}
	}
	if(status==CLICOU && segundo_lock >= lock_seg+3){
		//printf("BBBBBBBBB");
		if (tx >= 230 && tx <= 294){
			if(ty >= 20 && ty <= 84 ){
				flag_lock = !flag_lock;
				locking_flag = false;
				segundo_lock = 0;
				draw_button_lock(flag_lock);
			}
		}
	}
	
}

void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
				touch_event.id, touch_event.x, touch_event.y,
				touch_event.status, conv_x, conv_y);
		update_screen(conv_x, conv_y,touch_event.status);

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	//pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callBack);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);
}

int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};
	
	RTC_init();
	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	draw_screen();
	io_init();
	p_primeiro = initMenuOrder();
		
	tempo_total_min = (p_primeiro->enxagueTempo*p_primeiro->enxagueQnt)+p_primeiro->centrifugacaoTempo;
	tempo_total_hora = (tempo_total_min / 60);
	tempo_total_min = tempo_total_min % 60;
	
	sprintf(tempo_total_seg_string,"%d",tempo_total_seg);
	write_text(290,216,315,232, tempo_total_seg_string);
	
	sprintf(tempo_total_min_string,"%d",tempo_total_min);
	write_text(258,216,281,232, tempo_total_min_string);
	write_text(282,216,283,232,":");
	
	sprintf(tempo_total_hora_string,"%d",tempo_total_hora);
	write_text(221,216,246,232, tempo_total_hora_string);
	write_text(246,216,248,232,":");
		
	write_text(2,2,200,15, "Desligado");
	write_text(2,94,105, 105, "Ciclo:");
	write_text(220,2,320,15, "Unlocked");
	write_text(90,94,234,105, p_primeiro->nome);
	write_text(2,300,320,320,"Porta Aberta");
	draw_button_lock(flag_lock);
	making_border(202,0,204,90);
	making_border(0,88,320,90);
	making_border(0,210 ,320 ,212);
	making_border(0,236 ,320 ,238);
	making_border(0,295 ,320 ,297);
	write_text(0,216,320,232, "Tempo Total Ciclo:");
	write_text(0,240,320,256, "Tempo Restante do Ciclo:");
	draw_button_liga(flag_ligar);
	draw_button_cicle_previus(1);
	draw_button_cicle_next(1);
	ili9488_draw_pixmap(85, 115, botao_next.width, botao_next.height, botao_next.data);
	ili9488_draw_pixmap(20, 115, botao_previous.width, botao_previous.height, botao_previous.data);
	ili9488_draw_pixmap(200, 115, p_primeiro->icone->width, p_primeiro->icone->height, p_primeiro->icone->data);

	  

	/* Initialize the mXT touch device */
	mxt_init(&device);
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	printf("\n\rmaXTouch data USART transmitter\n\r");

	while (true) {
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		if (flag_ligar){
			if(tempo_total_min == minuto && tempo_total_hora == hora && tempo_total_seg == segundo){
				flag_ligar = 0;
				minuto = 0;
				segundo = 0;
				hora = 0;
				print_tempo();
				draw_button_liga(flag_ligar);
				
			}else{
				print_tempo();
				if(segundo%2==0){
					ili9488_draw_pixmap(120, 380, trabalhando1.width, trabalhando1.height, trabalhando1.data);
					}else{
					ili9488_draw_pixmap(120, 380, trabalhando2.width, trabalhando2.height, trabalhando2.data);
				}
			}
		}else{
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
			ili9488_draw_filled_rectangle(0, 322, 320,480);
			segundo = 0;
			minuto = 0;
			hora = 0;
			print_tempo();
		}
	}
	return 0;
}
