#include "crsf.h"
#include <string.h>

// CRSF packet structure
// [Sync] [Length] [Type] [Payload...] [CRC8]
// Length = Type + Payload + CRC8

#define CRSF_STATE_SYNC     0
#define CRSF_STATE_LENGTH   1
#define CRSF_STATE_TYPE     2
#define CRSF_STATE_PAYLOAD  3

static uint8_t crc8_calc(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

static void crsf_unpack_channels(crsf_t *crsf, const uint8_t *payload) {
    // RC_CHANNELS_PACKED payload is 22 bytes, encoding 16 channels, 11 bits each
    crsf->channels[0]  = (payload[0]       | payload[1] << 8)                          & 0x07FF;
    crsf->channels[1]  = (payload[1] >> 3  | payload[2] << 5)                          & 0x07FF;
    crsf->channels[2]  = (payload[2] >> 6  | payload[3] << 2  | payload[4] << 10)      & 0x07FF;
    crsf->channels[3]  = (payload[4] >> 1  | payload[5] << 7)                          & 0x07FF;
    crsf->channels[4]  = (payload[5] >> 4  | payload[6] << 4)                          & 0x07FF;
    crsf->channels[5]  = (payload[6] >> 7  | payload[7] << 1  | payload[8] << 9)       & 0x07FF;
    crsf->channels[6]  = (payload[8] >> 2  | payload[9] << 6)                          & 0x07FF;
    crsf->channels[7]  = (payload[9] >> 5  | payload[10] << 3)                         & 0x07FF;
    crsf->channels[8]  = (payload[10] >> 6 | payload[11] << 2 | payload[12] << 10)     & 0x07FF;
    crsf->channels[9]  = (payload[12] >> 1 | payload[13] << 7)                         & 0x07FF;
    crsf->channels[10] = (payload[13] >> 4 | payload[14] << 4)                         & 0x07FF;
    crsf->channels[11] = (payload[14] >> 7 | payload[15] << 1 | payload[16] << 9)      & 0x07FF;
    crsf->channels[12] = (payload[16] >> 2 | payload[17] << 6)                         & 0x07FF;
    crsf->channels[13] = (payload[17] >> 5 | payload[18] << 3)                         & 0x07FF;
    crsf->channels[14] = (payload[18] >> 6 | payload[19] << 2 | payload[20] << 10)     & 0x07FF;
    crsf->channels[15] = (payload[20] >> 1 | payload[21] << 7)                         & 0x07FF;
    
    crsf->is_connected = true;
    crsf->new_data = true;
}

void crsf_init(crsf_t *crsf) {
    memset(crsf, 0, sizeof(crsf_t));
}

void crsf_parse_byte(crsf_t *crsf, uint8_t byte) {
    static uint8_t state = CRSF_STATE_SYNC;
    static uint8_t length = 0;
    static uint8_t type = 0;
    static uint8_t payload[CRSF_MAX_FRAME_SIZE];
    static uint8_t payload_idx = 0;
    static uint8_t crc = 0;

    switch (state) {
        case CRSF_STATE_SYNC:
            if (byte == CRSF_SYNC_BYTE) {
                state = CRSF_STATE_LENGTH;
            }
            break;

        case CRSF_STATE_LENGTH:
            if (byte >= 2 && byte <= CRSF_MAX_FRAME_SIZE) {
                length = byte;
                state = CRSF_STATE_TYPE;
            } else {
                state = CRSF_STATE_SYNC;
            }
            break;

        case CRSF_STATE_TYPE:
            type = byte;
            crc = crc8_calc(0, byte);
            payload_idx = 0;
            if (length > 2) {
                state = CRSF_STATE_PAYLOAD;
            } else {
                // If length is 2, it means Type (1) + CRC (1)
                // This shouldn't happen for valid data packets, but just in case
                state = CRSF_STATE_SYNC;
            }
            break;

        case CRSF_STATE_PAYLOAD:
            if (payload_idx < length - 2) {
                payload[payload_idx++] = byte;
                crc = crc8_calc(crc, byte);
            } else {
                // The current byte is the CRC
                if (crc == byte) {
                    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
                        crsf_unpack_channels(crsf, payload);
                    }
                }
                state = CRSF_STATE_SYNC; // Ready for next packet
            }
            break;
            
        default:
            state = CRSF_STATE_SYNC;
            break;
    }
}
