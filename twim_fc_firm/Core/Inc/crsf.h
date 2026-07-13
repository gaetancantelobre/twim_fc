#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#include <stdint.h>
#include <stdbool.h>

#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_MAX_FRAME_SIZE     64
#define CRSF_MAX_CHANNEL        16

typedef struct {
    uint16_t channels[CRSF_MAX_CHANNEL];
    bool is_connected;
    bool new_data;
} crsf_t;

void crsf_init(crsf_t *crsf);
void crsf_parse_byte(crsf_t *crsf, uint8_t byte);

#endif /* INC_CRSF_H_ */
