#ifndef _DDSM_CTRL_H
#define _DDSM_CTRL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DDSM_BAUDRATE 115200

#define TYPE_DDSM115  1
#define TYPE_DDSM210  2
#define TIME_BETWEEN_CMD 4
#define TIME0UT 4


class DDSM_CTRL{
public:
	DDSM_CTRL();

	virtual void clear_ddsm_buffer();
	virtual uint8_t crc8_update(uint8_t crc, uint8_t data);
	virtual int set_ddsm_type(int inputType);
	virtual int ddsm_id_check();
	virtual int ddsm_change_id(uint8_t id);
	virtual void ddsm_change_mode(uint8_t id, uint8_t mode);
	virtual void ddsm_ctrl(uint8_t id, int cmd, uint8_t act);
	virtual void ddsm_get_info(uint8_t id);
	virtual void ddsm_stop(uint8_t id);
	virtual int ddsm210_fb();
	virtual int ddsm115_fb();

private:
	const size_t packet_length;   
	uint8_t packet_move[10];
	uint8_t ddsm_type;
	bool get_info_flag;

public:
	HardwareSerial *pSerial;
	
	int speed_data;  // 115 210
	int current;     // 210
	int acceleration_time; // 210
	int temperature; // 115[info] 210

	int ddsm_mode;   // 115
	int ddsm_torque; // 115
	int ddsm_u8;     // 115[info]

	int32_t mileage; // 210[info]
	int ddsm_pos;    // 115 210[info]

	int fault_code;  // 115 210
};

#endif