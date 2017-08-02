/* -*- c++ -*- */
/*
 * Copyright 2010 Ettus Research LLC
 *
 */

//contains routines for loading programs from Flash. depends on Flash libraries.
//also contains routines for reading / writing EEPROM flags for the bootloader
#include <stdbool.h>
#include <string.h>
#include <bootloader_utils.h>
#include <spi_flash.h>
#include <i2c.h>
#include <memory_map.h>
#include <nonstdio.h>
#ifdef SPARTAN6
#include <xilinx_s6_icap.h>
#else
#include <xilinx_s3_icap.h>
#endif
#include <mdelay.h>
#include "spi.h"

#define BUTTON_PUSHED ((router_status->irqs & PIC_BUTTON) ? 0 : 1)

int is_valid_fpga_image(uint32_t addr) {
//	printf("is_valid_fpga_image(): starting with addr=%x...\n", addr);
	uint8_t imgbuf[64];
	spi_flash_read(addr, 64, imgbuf);
	//we're just looking for leading 0xFF padding, followed by the sync bytes 0xAA 0x99
	for(size_t i = 0; i<63; i++) {
		if(imgbuf[i] == 0xFF) continue;
		if(imgbuf[i] == 0xAA && imgbuf[i+1] == 0x99) {
			//printf("is_valid_fpga_image(): found valid FPGA image\n");
			return 1;
		}
	}
	
	return 0;
}

int is_valid_fw_image(uint32_t addr) {
	static const uint8_t fwheader[2] = {0x0b, 0x0b}; //just lookin for a jump to anywhere located at the reset vector
	uint8_t buf[2];
	spi_flash_read(addr, 2, buf);
	return memcmp(buf, fwheader, 2) == 0;
}

void start_program(void)
{
	//ignoring the addr now
	//all this does is tap that register
	*((volatile uint32_t *) SR_ADDR_BLDRDONE) = 1;
}

void do_the_bootload_thing(void) {
#ifdef NO_FLASH
	puts("Starting UmTRX without flash.");
	return;
#else
	spif_init(); //initialize SPI flash clock
	
	puts("SPI Flash has been initialized");

	bool production_image = find_safe_booted_flag();
	printf("Production image = %d\n", production_image);
	set_safe_booted_flag(0); //haven't booted yet
	
	if(BUTTON_PUSHED) { //see memory_map.h
		puts("Starting UmTRX in safe mode.");
		return;
	}
	
	if(!production_image) {
		puts("Checking for valid production FPGA image...");
		if(is_valid_fpga_image(PROD_FPGA_IMAGE_LOCATION_ADDR)) {
			puts("Valid production FPGA image found, booting..");
			set_safe_booted_flag(1);
			mdelay(300); //so serial output can finish
#ifdef SPARTAN6
			icap_s6_reload_fpga(PROD_FPGA_IMAGE_LOCATION_ADDR, SAFE_FPGA_IMAGE_LOCATION_ADDR);
#else
			icap_s3_reload_fpga(PROD_FPGA_IMAGE_LOCATION_ADDR);
#endif
		}
		puts("No valid production FPGA image found.\n");
		return;
	}
	if(is_valid_fw_image(PROD_FW_IMAGE_LOCATION_ADDR)) {
		puts("Valid production firmware found. Loading...");
		spi_flash_read(PROD_FW_IMAGE_LOCATION_ADDR, FW_IMAGE_SIZE_BYTES, (void *)RAM_BASE);
		puts("Starting image...");
		mdelay(300);
		start_program();
		puts("ERROR: Return from main!");
		//if this happens, though, the safest thing to do is reboot the whole FPGA and start over.
		mdelay(300);
#ifdef SPARTAN6
		icap_s6_reload_fpga(SAFE_FPGA_IMAGE_LOCATION_ADDR, SAFE_FPGA_IMAGE_LOCATION_ADDR);
#else
		icap_s3_reload_fpga(SAFE_FPGA_IMAGE_LOCATION_ADDR);
#endif
		return;
	}
	puts("No valid production firmware found!");
#endif
}
