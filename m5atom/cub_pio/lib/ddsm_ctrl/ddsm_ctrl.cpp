#include "ddsm_ctrl.h"

// DDSM_CTRL::DDSM_CTRL() {
// 	packet_length = 10;
// 	packet_move[10] = {0x01, 0x64, 0xff, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 0xda};
// 	ddsm_type = TYPE_DDSM115;
// 	get_info_flag = false;
// }
DDSM_CTRL::DDSM_CTRL() 
    : packet_length(10),  // Initialize const member in the initializer list
      ddsm_type(TYPE_DDSM115), 
      get_info_flag(false),
      pSerial(nullptr)
{
    packet_move[0] = 0x01;
    packet_move[1] = 0x64;
    packet_move[2] = 0xff;
    packet_move[3] = 0xce;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    packet_move[9] = 0xda;
}

// CRC-8/MAXIM
uint8_t DDSM_CTRL::crc8_update(uint8_t crc, uint8_t data) {
  uint8_t i;
  crc = crc ^ data;
  for (i = 0; i < 8; ++i) {
    if (crc & 0x01) {
      crc = (crc >> 1) ^ 0x8c;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

// config the type of ddsm.
int DDSM_CTRL::set_ddsm_type(int inputType) {
	if (inputType == 115 || inputType == TYPE_DDSM115) {
		ddsm_type = TYPE_DDSM115;
		return TYPE_DDSM115;
	} else if (inputType == 210 || inputType == TYPE_DDSM210) {
		ddsm_type = TYPE_DDSM210;
		return TYPE_DDSM210;
	}
	return -1;
}

// clear ddsm serial buffer.
void DDSM_CTRL::clear_ddsm_buffer() {
  while (pSerial->available() > 0) {
    pSerial->read();
  }
}

// feedback data from ddsm210
int DDSM_CTRL::ddsm210_fb() {
	unsigned long startTime = millis();

	while (pSerial->available() <10) {
		if (millis() - startTime >= TIME0UT) {
			return -1;
		}
	}

	uint8_t data[10];
	pSerial->readBytes(data, 10);

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]){
    return -1;
  }
  int feedback_type = data[1];
  uint8_t ID = data[0];

  if (feedback_type == 0x64) {
    speed_data = (data[2] << 8) | data[3];
    if (speed_data & 0x8000) {
      speed_data = -(0x10000 - speed_data);
    }

    current = (data[4] << 8) | data[5];
    if (current & 0x8000) {
      current = -(0x10000 - current);
    }

    acceleration_time = data[6];
    temperature = data[7];
    fault_code = data[8];
  } else if (feedback_type == 0x74) {
    mileage = (int32_t)((uint32_t)data[2] << 24 | (uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5]);
    ddsm_pos = (data[6] << 8) | data[7];
    fault_code = data[8];
  }
  return 1;
}

// feedback data from ddsm115.
int DDSM_CTRL::ddsm115_fb() {
	unsigned long startTime = millis();

	while (pSerial->available() <10) {
		if (millis() - startTime >= TIME0UT) {
			return -1;
		}
	}

  uint8_t data[10];
  Serial1.readBytes(data, 10);

  uint8_t ddsm_id = data[0];

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]){
    return -1;
  }

  ddsm_mode = data[1];

  ddsm_torque = (data[2] << 8) | data[3];
  if (ddsm_torque & 0x8000) {
    ddsm_torque = -(0x10000 - ddsm_torque);
  }

  speed_data = (data[4] << 8) | data[5];
  if (speed_data & 0x8000) {
    speed_data = -(0x10000 - speed_data);
  }

  if (get_info_flag) {
    get_info_flag = false;
    temperature = data[6];
    ddsm_u8 = data[7];
    fault_code = data[8];
  } else {
    ddsm_pos = (data[6] << 8) | data[7];
    fault_code = data[8];
  }
  return 1;
}


// check the ID of ddsm.
int DDSM_CTRL::ddsm_id_check() {
	packet_move[0] = 0xC8;
	packet_move[1] = 0x64;
	packet_move[2] = 0x00;
	packet_move[3] = 0x00;

	packet_move[4] = 0x00;
	packet_move[5] = 0x00;
	packet_move[6] = 0x00;
	packet_move[7] = 0x00;

	packet_move[8] = 0x00;
	packet_move[9] = 0xDE;
	pSerial->write(packet_move, packet_length);

	unsigned long startTime = millis();

	while (pSerial->available() <10) {
		if (millis() - startTime >= TIME0UT) {
			return -1;
		}
	}

	uint8_t data[10];
	pSerial->readBytes(data, 10);

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]){
    return -1;
  }
  int feedback_type = data[1];
  uint8_t ID = data[0];
  return ID;
}

int DDSM_CTRL::ddsm_change_id(uint8_t id) {
	packet_move[0] = 0xAA;

	packet_move[1] = 0x55;
	packet_move[2] = 0x53;

	packet_move[3] = id;

	packet_move[4] = 0x00;
	packet_move[5] = 0x00;
	packet_move[6] = 0x00;
	packet_move[7] = 0x00;
	packet_move[8] = 0x00;

	// CRC-8/MAXIM
	uint8_t crc = 0;
	for (size_t i = 0; i < packet_length - 1; ++i) {
		crc = crc8_update(crc, packet_move[i]);
	}
	packet_move[9] = crc;

	for (int i = 0;i < 5;i++) {
		pSerial->write(packet_move, packet_length);
		delay(TIME_BETWEEN_CMD);
	}

	ddsm_id_check();
	unsigned long startTime = millis();

	while (pSerial->available() <10) {
		if (millis() - startTime >= TIME0UT) {
			return -1;
		}
	}

	uint8_t data[10];
	pSerial->readBytes(data, 10);

  // CRC-8/MAXIM
  crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]){
    return -1;
  }
  int feedback_type = data[1];
  uint8_t ID = data[0];

  if (id != ID) {
  	return -1;
  } else {
  	return ID;
  }
}

// change mode
// ddsm115:
// 1 - current loop
// 2 - speed loop
// 3 - position loop

// ddsm210:
// 0 - open loop
// 2 - speed loop
// 3 - position loop
void DDSM_CTRL::ddsm_change_mode(uint8_t id, uint8_t mode) {
  if (ddsm_type == TYPE_DDSM115) {
    packet_move[0] = id;
    packet_move[1] = 0xA0;
    packet_move[2] = 0x00;
    packet_move[3] = 0x00;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    packet_move[9] = mode;
  } else if (ddsm_type == TYPE_DDSM210) {
    packet_move[0] = id;
    packet_move[1] = 0xA0;
    packet_move[2] = mode;
    packet_move[3] = 0x00; 
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
      crc = crc8_update(crc, packet_move[i]);
    }
    packet_move[9] = crc;
  }
  pSerial->write(packet_move, packet_length);
}

// --- DDSM115 ---
// current loop, cmd: -32767 ~ 32767 -> -8 ~ 8 A (ddsm115 max current < 2.7A)
// speed loop, cmd: -200 ~ 200 rpm
// position loop, cmd: 0 ~ 32767 -> 0 ~ 360°

// --- DDSM210 ---
// open loop, cmd: -32767 ~ 32767
// speed loop, cmd: -2100 ~ 2100 -> -210 ~ 210 rpm
// position loop, cmd: 0 ~ 32767 -> 0 ~ 360°

//    wherever the mode is set to position mode
//    the currently position is the 0 position and it moves to the goal position
//    at the direction as the shortest path.
void DDSM_CTRL::ddsm_ctrl(uint8_t id, int cmd, uint8_t act) {
  packet_move[0] = id;
  packet_move[1] = 0x64;

  packet_move[2] = (cmd >> 8) & 0xFF;
  packet_move[3] = cmd & 0xFF;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;

  packet_move[6] = act;
  packet_move[7] = 0x00;

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;

  pSerial->write(packet_move, packet_length);

  ddsm210_fb();
}

void DDSM_CTRL::ddsm_get_info(uint8_t id) {  
  packet_move[0] = id;

  if (ddsm_type == TYPE_DDSM115) {
    get_info_flag = true;
  }

  packet_move[1] = 0x74;

  packet_move[2] = 0x00;
  packet_move[3] = 0x00;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;

  packet_move[8] = 0x00;
  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;
  pSerial->write(packet_move, packet_length);
  if (ddsm_type == TYPE_DDSM115) {
  	ddsm115_fb();
  } else if (ddsm_type == TYPE_DDSM210) {
  	ddsm210_fb();
  }
}

void DDSM_CTRL::ddsm_stop(uint8_t id) {
	ddsm_ctrl(id, 0, 0);
  if (ddsm_type == TYPE_DDSM115) {
  	ddsm115_fb();
  } else if (ddsm_type == TYPE_DDSM210) {
  	ddsm210_fb();
  }
}