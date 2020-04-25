/*
 * oled.c
 *
 *  Created on: 19.12.2017
 *      Author: daniel
 */


#include "oled.h"

const unsigned char Image1 [] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x70, 0x78, 0x7C, 0x7E, 0x7F, 0x77, 0x73, 0x71, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xFF, 0x7F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3F, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x3C, 0x1E, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Image2 [] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x40, 0x60, 0x60, 0x20, 0x30, 0x10, 0x18, 0x08, 0x08, 0x0C, 0x04, 0x06, 0x06, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0x04, 0x06, 0x06, 0x07, 0x07, 0x05, 0x05, 0x02, 0x02, 0x02, 0xC2, 0x32, 0x0B, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3C, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x40, 0x40, 0x40, 0x40, 0x40, 0x20, 0x30, 0x4E, 0x43, 0xC4, 0x84, 0x88, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x90, 0x8F, 0x9B, 0xE0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x86, 0x59, 0x21, 0x40, 0x40, 0xC0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x82, 0x82, 0x84, 0x4C, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x70, 0x18, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x81, 0xE1, 0xF9, 0xF9, 0xE1, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xE1, 0xF9, 0xF1, 0xE1, 0xC1, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x0C, 0x18, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xC0, 0xC0, 0x80, 0x00, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x02, 0x00, 0xC0, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0E, 0x38, 0x60, 0xC0, 0x80, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x7F, 0xFF, 0xFE, 0xFE, 0xFC, 0xF8, 0xF8, 0xC0, 0xC0, 0xC0, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xFC, 0xFC, 0xFE, 0x7E, 0x7F, 0x1F, 0x0F, 0x03, 0x00, 0x80, 0xC0, 0x60, 0x1C, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x06, 0x04, 0x0C, 0x08, 0x18, 0x10, 0x30, 0x31, 0x23, 0x63, 0x63, 0x67, 0x67, 0x67, 0x67, 0x67, 0x67, 0x67, 0x67, 0x60, 0x60, 0x60, 0x67, 0x27, 0x27, 0x33, 0x33, 0x11, 0x11, 0x18, 0x18, 0x08, 0x04, 0x06, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

};

const unsigned char FontSystem5x8[]=
{
		0x00,0x00,0x00,0x00,0x00, /* Space */
		0x00,0x00,0x4f,0x00,0x00, /* ! */
		0x00,0x07,0x00,0x07,0x00, /* " */
		0x14,0x7f,0x14,0x7f,0x14, /* # */
		0x24,0x2a,0x7f,0x2a,0x12, /* 0x */
		0x23,0x13,0x08,0x64,0x62, /* % */
		0x36,0x49,0x55,0x22,0x20, /* & */
		0x00,0x05,0x03,0x00,0x00, /* ' */
		0x00,0x1c,0x22,0x41,0x00, /* ( */
		0x00,0x41,0x22,0x1c,0x00, /* ) */
		0x14,0x08,0x3e,0x08,0x14, /* // */
		0x08,0x08,0x3e,0x08,0x08, /* + */
		0x50,0x30,0x00,0x00,0x00, /* , */
		0x08,0x08,0x08,0x08,0x08, /* - */
		0x00,0x60,0x60,0x00,0x00, /* . */
		0x20,0x10,0x08,0x04,0x02, /* / */
		0x3e,0x51,0x49,0x45,0x3e, /* 0 */
		0x00,0x42,0x7f,0x40,0x00, /* 1 */
		0x42,0x61,0x51,0x49,0x46, /* 2 */
		0x21,0x41,0x45,0x4b,0x31, /* 3 */
		0x18,0x14,0x12,0x7f,0x10, /* 4 */
		0x27,0x45,0x45,0x45,0x39, /* 5 */
		0x3c,0x4a,0x49,0x49,0x30, /* 6 */
		0x01,0x71,0x09,0x05,0x03, /* 7 */
		0x36,0x49,0x49,0x49,0x36, /* 8 */
		0x06,0x49,0x49,0x29,0x1e, /* 9 */
		0x00,0x36,0x36,0x00,0x00, /* : */
		0x00,0x56,0x36,0x00,0x00, /* ; */
		0x08,0x14,0x22,0x41,0x00, /* < */
		0x14,0x14,0x14,0x14,0x14, /* = */
		0x00,0x41,0x22,0x14,0x08, /* > */
		0x02,0x01,0x51,0x09,0x06, /* ? */
		0x3e,0x41,0x5d,0x55,0x1e, /* @ */
		0x7e,0x11,0x11,0x11,0x7e, /* A */
		0x7f,0x49,0x49,0x49,0x36, /* B */
		0x3e,0x41,0x41,0x41,0x22, /* C */
		0x7f,0x41,0x41,0x22,0x1c, /* D */
		0x7f,0x49,0x49,0x49,0x41, /* E */
		0x7f,0x09,0x09,0x09,0x01, /* F */
		0x3e,0x41,0x49,0x49,0x7a, /* G */
		0x7f,0x08,0x08,0x08,0x7f, /* H */
		0x00,0x41,0x7f,0x41,0x00, /* I */
		0x20,0x40,0x41,0x3f,0x01, /* J */
		0x7f,0x08,0x14,0x22,0x41, /* K */
		0x7f,0x40,0x40,0x40,0x40, /* L */
		0x7f,0x02,0x0c,0x02,0x7f, /* M */
		0x7f,0x04,0x08,0x10,0x7f, /* N */
		0x3e,0x41,0x41,0x41,0x3e, /* O */
		0x7f,0x09,0x09,0x09,0x06, /* P */
		0x3e,0x41,0x51,0x21,0x5e, /* Q */
		0x7f,0x09,0x19,0x29,0x46, /* R */
		0x26,0x49,0x49,0x49,0x32, /* S */
		0x01,0x01,0x7f,0x01,0x01, /* T */
		0x3f,0x40,0x40,0x40,0x3f, /* U */
		0x1f,0x20,0x40,0x20,0x1f, /* V */
		0x3f,0x40,0x38,0x40,0x3f, /* W */
		0x63,0x14,0x08,0x14,0x63, /* X */
		0x07,0x08,0x70,0x08,0x07, /* Y */
		0x61,0x51,0x49,0x45,0x43, /* Z */
		0x00,0x7f,0x41,0x41,0x00, /* [ */
		0x02,0x04,0x08,0x10,0x20, /* \ */
		0x00,0x41,0x41,0x7f,0x00, /* ] */
		0x04,0x02,0x01,0x02,0x04, /* ^ */
		0x40,0x40,0x40,0x40,0x40, /* _ */
		0x00,0x00,0x03,0x05,0x00, /* ` */
		0x20,0x54,0x54,0x54,0x78, /* a */
		0x7F,0x44,0x44,0x44,0x38, /* b */
		0x38,0x44,0x44,0x44,0x44, /* c */
		0x38,0x44,0x44,0x44,0x7f, /* d */
		0x38,0x54,0x54,0x54,0x18, /* e */
		0x04,0x04,0x7e,0x05,0x05, /* f */
		0x08,0x54,0x54,0x54,0x3c, /* g */
		0x7f,0x08,0x04,0x04,0x78, /* h */
		0x00,0x44,0x7d,0x40,0x00, /* i */
		0x20,0x40,0x44,0x3d,0x00, /* j */
		0x7f,0x10,0x28,0x44,0x00, /* k */
		0x00,0x41,0x7f,0x40,0x00, /* l */
		0x7c,0x04,0x7c,0x04,0x78, /* m */
		0x7c,0x08,0x04,0x04,0x78, /* n */
		0x38,0x44,0x44,0x44,0x38, /* o */
		0x7c,0x14,0x14,0x14,0x08, /* p */
		0x08,0x14,0x14,0x14,0x7c, /* q */
		0x7c,0x08,0x04,0x04,0x00, /* r */
		0x48,0x54,0x54,0x54,0x24, /* s */
		0x04,0x04,0x3f,0x44,0x44, /* t */
		0x3c,0x40,0x40,0x20,0x7c, /* u */
		0x1c,0x20,0x40,0x20,0x1c, /* v */
		0x3c,0x40,0x30,0x40,0x3c, /* w */
		0x44,0x28,0x10,0x28,0x44, /* x */
		0x0c,0x50,0x50,0x50,0x3c, /* y */
		0x44,0x64,0x54,0x4c,0x44, /* z */
		0x08,0x36,0x41,0x41,0x00, /* { */
		0x00,0x00,0x77,0x00,0x00, /* | */
		0x00,0x41,0x41,0x36,0x08, /* } */
		0x08,0x08,0x2a,0x1c,0x08, /* <- */
		0x08,0x1c,0x2a,0x08,0x08, /* -> */
		0xff,0xff,0xff,0xff,0xff, /*  */
};
/*--------------------------------------------------------------------------------*/
static uint8_t s_chDispalyBuffer[128][8];
/*--------------------------------------------------------------------------------*/
void OLED_Write_Byte(uint8_t chData, uint8_t chCmd)
{
	__OLED_CS_CLR();

	if (chCmd) {
		__OLED_DC_SET();
	} else {
		__OLED_DC_CLR();
	}
	while(!Chip_SSP_GetStatus(LPC_SSP1, SSP_STAT_TNF)); //TX FIFO not FULL
	Chip_SSP_SendFrame(LPC_SSP1, chData);
	while(Chip_SSP_GetStatus(LPC_SSP1, SSP_STAT_BSY)); //TX BUSY

	__OLED_DC_SET();
	__OLED_CS_SET();
}
/*--------------------------------------------------------------------------------*/
void OLED_Display_On(void)
{
	OLED_Write_Byte(0x8D, OLED_CMD);
	OLED_Write_Byte(0x14, OLED_CMD);
	OLED_Write_Byte(0xAF, OLED_CMD);
}
/*--------------------------------------------------------------------------------*/
void OLED_Display_Off(void)
{
	OLED_Write_Byte(0x8D, OLED_CMD);
	OLED_Write_Byte(0x10, OLED_CMD);
	OLED_Write_Byte(0xAE, OLED_CMD);
}
/*--------------------------------------------------------------------------------*/
void OLED_Refresh_Gram(void)
{
	for (uint8_t i = 0; i < 8; i ++) {
		OLED_Write_Byte(0xB0 + i, OLED_CMD);
		__SET_COL_START_ADDR();
		for (uint8_t j = 0; j < OLED_WIDTH; j ++) {
			OLED_Write_Byte(s_chDispalyBuffer[j][i], OLED_DAT);
		}
	}
}
/*--------------------------------------------------------------------------------*/
void OLED_Clear_Screen(uint8_t chFill)
{
	for (uint8_t i = 0; i < 8; i ++) {
		OLED_Write_Byte(0xB0 + i, OLED_CMD);
		__SET_COL_START_ADDR();
		for (uint8_t j = 0; j < OLED_WIDTH; j ++) {
			s_chDispalyBuffer[j][i] = chFill;
		}
	}
}
/*--------------------------------------------------------------------------------*/
void OLED_Draw_Point(uint8_t chXpos, uint8_t chYpos, uint8_t chPoint)
{
	uint8_t chPos, chBx, chTemp = 0;

	if (chXpos >= OLED_WIDTH || chYpos >= OLED_HEIGHT) {
		return;
	}
	chPos = chYpos / 8;
	chBx  = chYpos % 8;
	chTemp = 1 << (chBx);
	if (chPoint) {
		s_chDispalyBuffer[chXpos][chPos] |= chTemp;
	} else {
		s_chDispalyBuffer[chXpos][chPos] &= ~chTemp;
	}
}
/*--------------------------------------------------------------------------------*/
void OLED_Draw_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	uint8_t  x, y;
	int16_t addx, addy, dx, dy, P;

	dx = abs(x2 - x1);
	dy = abs(y2 - y1);
	x = x1;
	y = y1;
	addx = (x1 > x2) ? -1 : 1;
	addy = (y1 > y2) ? -1 : 1;

	if(dx >= dy){
		P = 2*dy - dx;
		for(int16_t i=0; i<=dx; i++){
			OLED_Draw_Point(x,y,1);
			if(P < 0){
				P += 2*dy;
				x += addx;
			}else{
				P += 2*(dy - dx);
				x += addx;
				y += addy;
			}
		}
	}else{
		P = 2*dx - dy;
		for(int16_t i=0; i<=dy; i++){
			OLED_Draw_Point(x,y,1);
			if(P < 0){
				P += 2*dx;
				y += addy;
			}else{
				P += 2*(dx - dy);
				x += addx;
				y += addy;
			}
		}
	}
}
/*--------------------------------------------------------------------------------*/
void OLED_Draw_Bitmap(const uint8_t *pchBmp)
{
	for (uint8_t i = 0; i < 8; i ++) {
		for (uint8_t j = 0; j < OLED_WIDTH; j ++) {
			s_chDispalyBuffer[j][i] = *pchBmp++;
		}
	}
	OLED_Refresh_Gram();
}
/*--------------------------------------------------------------------------------*/
void OLED_Puts(uint8_t x, uint8_t y, char *text)
{
	uint8_t i,j=0;
	char c;

	while(text[j])
	{
		c=text[j]-32;
		for (i = 0; i < 5; i ++) {
			s_chDispalyBuffer[x+i+(6*j)][y] = FontSystem5x8[5*c+i];
		}
		s_chDispalyBuffer[x+i+(6*j)][y]=0;
		j++;
	}
}
/*--------------------------------------------------------------------------------*/
void OLED_Init(void)
{
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, OLED_CS_GPIO,  OLED_CS_PIN,  true);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, OLED_RES_GPIO, OLED_RES_PIN, true);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, OLED_DC_GPIO,  OLED_DC_PIN,  true);

	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 20, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_20 connected to SCK1 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 22, (IOCON_FUNC2 | IOCON_MODE_PULLUP)); /* PIO1_22 connected to MOSI1 */

	Chip_SSP_Init(LPC_SSP);
	Chip_SSP_SetMaster(LPC_SSP, 1);
	Chip_SSP_SetClockRate(LPC_SSP, 0, 2);
	Chip_SSP_SetFormat(LPC_SSP, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA1_CPOL1);
	Chip_SSP_Enable(LPC_SSP);

	__OLED_CS_SET();   //CS set
	__OLED_DC_CLR();   //D/C reset
	__OLED_RES_SET();  //RES set

#if defined(SSD_1306)
	OLED_Write_Byte(0xAE, OLED_CMD);//--turn off oled panel
	OLED_Write_Byte(0x00, OLED_CMD);//--set low column address
	OLED_Write_Byte(0x10, OLED_CMD);//--set high column address
	OLED_Write_Byte(0x40, OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_Write_Byte(0x20, OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_Write_Byte(0x02, OLED_CMD);//
	OLED_Write_Byte(0x81, OLED_CMD);//--set contrast control register
	OLED_Write_Byte(0xCF, OLED_CMD);//--set SEG Output Current Brightness
	OLED_Write_Byte(0xA1, OLED_CMD);//--set SEG/Column Mapping
	OLED_Write_Byte(0xC8, OLED_CMD);//--set COM/Row Scan Direction (C0h / C8h)
	OLED_Write_Byte(0xA6, OLED_CMD);//--set normal display
	OLED_Write_Byte(0xA8, OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_Write_Byte(0x3F, OLED_CMD);//--set 1/64 duty
	OLED_Write_Byte(0xD3, OLED_CMD);//--set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_Write_Byte(0x00, OLED_CMD);//-not offset
	OLED_Write_Byte(0xD5, OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_Write_Byte(0x80, OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_Write_Byte(0xD9, OLED_CMD);//--set pre-charge period
	OLED_Write_Byte(0xF1, OLED_CMD);//--set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_Write_Byte(0xDA, OLED_CMD);//--set com pins hardware configuration
	OLED_Write_Byte(0x12, OLED_CMD);
	OLED_Write_Byte(0xDB, OLED_CMD);//--set vcomh
	OLED_Write_Byte(0x40, OLED_CMD);//--set VCOM Deselect Level
	OLED_Write_Byte(0x8D, OLED_CMD);//--set Charge Pump enable/disable
	OLED_Write_Byte(0x14, OLED_CMD);//--set(0x10) disable
	OLED_Write_Byte(0xA4, OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_Write_Byte(0xA6, OLED_CMD);// Disable Inverse Display On (0xa6/a7)
	OLED_Write_Byte(0xAF, OLED_CMD);//--turn on oled panel
#endif

#if defined(SH_1106)
	OLED_Write_Byte(0xAE, OLED_CMD);//--turn off oled panel
	OLED_Write_Byte(0x02, OLED_CMD);//--set low column address
	OLED_Write_Byte(0x10, OLED_CMD);//--set high column address
	OLED_Write_Byte(0x40, OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_Write_Byte(0xB0, OLED_CMD);//--set page address
	OLED_Write_Byte(0x81, OLED_CMD);//--set contrast control register
	OLED_Write_Byte(0xFF, OLED_CMD);//--set SEG Output Current Brightness
	OLED_Write_Byte(0xA1, OLED_CMD);//--set SEG/Column Mapping
	OLED_Write_Byte(0xC8, OLED_CMD);//--set COM/Row Scan Direction (C0h / C8h)
	OLED_Write_Byte(0xA8, OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_Write_Byte(0x3F, OLED_CMD);//--set 1/64 duty
	OLED_Write_Byte(0xD3, OLED_CMD);//--set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_Write_Byte(0x00, OLED_CMD);//-not offset
	OLED_Write_Byte(0xD5, OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_Write_Byte(0x80, OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_Write_Byte(0xD9, OLED_CMD);//--set pre-charge period
	OLED_Write_Byte(0x1F, OLED_CMD);//--set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_Write_Byte(0xDA, OLED_CMD);//--set com pins hardware configuration
	OLED_Write_Byte(0x12, OLED_CMD);
	OLED_Write_Byte(0xDB, OLED_CMD);//--set vcomh
	OLED_Write_Byte(0x40, OLED_CMD);//--set VCOM Deselect Level
	OLED_Write_Byte(0xAD, OLED_CMD);//--set Charge Pump enable/disable
	OLED_Write_Byte(0x8B, OLED_CMD);//--set external VCC
	OLED_Write_Byte(0xAF, OLED_CMD);//--turn on oled panel
#endif

	OLED_Clear_Screen(0x00);
}
