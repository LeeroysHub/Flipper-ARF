#include "scher_khan.h"
#include "../blocks/custom_btn_i.h"
#include "../blocks/generic.h"

#define TAG "SubGhzProtocolScherKhan"

static const char* scher_khan_btn_name(uint8_t btn) {
    switch(btn) {
    case 0x1: return "Lock";
    case 0x2: return "Unlock";
    case 0x4: return "Trunk";
    case 0x8: return "Aux";
    default:  return "?";
    }
}

static uint8_t scher_khan_btn_to_custom(uint8_t btn) {
    switch(btn) {
    case 0x1: return SUBGHZ_CUSTOM_BTN_UP;
    case 0x2: return SUBGHZ_CUSTOM_BTN_DOWN;
    case 0x4: return SUBGHZ_CUSTOM_BTN_LEFT;
    case 0x8: return SUBGHZ_CUSTOM_BTN_RIGHT;
    default:  return SUBGHZ_CUSTOM_BTN_OK;
    }
}

static uint8_t scher_khan_custom_to_btn(uint8_t custom, uint8_t original_btn) {
    if(custom == SUBGHZ_CUSTOM_BTN_OK)    return original_btn;
    if(custom == SUBGHZ_CUSTOM_BTN_UP)    return 0x1;
    if(custom == SUBGHZ_CUSTOM_BTN_DOWN)  return 0x2;
    if(custom == SUBGHZ_CUSTOM_BTN_LEFT)  return 0x4;
    if(custom == SUBGHZ_CUSTOM_BTN_RIGHT) return 0x8;
    return original_btn;
}

static uint8_t scher_khan_get_btn_code(uint8_t original_btn) {
    return scher_khan_custom_to_btn(subghz_custom_btn_get(), original_btn);
}

static const SubGhzBlockConst subghz_protocol_scher_khan_const = {
    .te_short = 750,
    .te_long = 1100,
    .te_delta = 160,
    .min_count_bit_for_found = 35,
};

struct SubGhzProtocolDecoderScherKhan {
    SubGhzProtocolDecoderBase base;

    SubGhzBlockDecoder decoder;
    SubGhzBlockGeneric generic;

    uint16_t header_count;
    const char* protocol_name;
};

struct SubGhzProtocolEncoderScherKhan {
    SubGhzProtocolEncoderBase base;

    SubGhzProtocolBlockEncoder encoder;
    SubGhzBlockGeneric generic;

    const char* protocol_name;
};

typedef enum {
    ScherKhanDecoderStepReset = 0,
    ScherKhanDecoderStepCheckPreambula,
    ScherKhanDecoderStepSaveDuration,
    ScherKhanDecoderStepCheckDuration,
} ScherKhanDecoderStep;

static void subghz_protocol_scher_khan_check_remote_controller(
    SubGhzBlockGeneric* instance,
    const char** protocol_name);

const SubGhzProtocolDecoder subghz_protocol_scher_khan_decoder = {
    .alloc = subghz_protocol_decoder_scher_khan_alloc,
    .free = subghz_protocol_decoder_scher_khan_free,

    .feed = subghz_protocol_decoder_scher_khan_feed,
    .reset = subghz_protocol_decoder_scher_khan_reset,

    .get_hash_data = subghz_protocol_decoder_scher_khan_get_hash_data,
    .serialize = subghz_protocol_decoder_scher_khan_serialize,
    .deserialize = subghz_protocol_decoder_scher_khan_deserialize,
    .get_string = subghz_protocol_decoder_scher_khan_get_string,
};

const SubGhzProtocolEncoder subghz_protocol_scher_khan_encoder = {
    .alloc = subghz_protocol_encoder_scher_khan_alloc,
    .free = subghz_protocol_encoder_scher_khan_free,

    .deserialize = subghz_protocol_encoder_scher_khan_deserialize,
    .stop = subghz_protocol_encoder_scher_khan_stop,
    .yield = subghz_protocol_encoder_scher_khan_yield,
};

const SubGhzProtocol subghz_protocol_scher_khan = {
    .name = SUBGHZ_PROTOCOL_SCHER_KHAN_NAME,
    .type = SubGhzProtocolTypeDynamic,
    .flag = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM | SubGhzProtocolFlag_AM |
            SubGhzProtocolFlag_Decodable |
            SubGhzProtocolFlag_Load | SubGhzProtocolFlag_Save | SubGhzProtocolFlag_Send,

    .decoder = &subghz_protocol_scher_khan_decoder,
    .encoder = &subghz_protocol_scher_khan_encoder,
};

// ======================== ENCODER ========================

void* subghz_protocol_encoder_scher_khan_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolEncoderScherKhan* instance = malloc(sizeof(SubGhzProtocolEncoderScherKhan));

    instance->base.protocol = &subghz_protocol_scher_khan;
    instance->generic.protocol_name = instance->base.protocol->name;

    instance->encoder.repeat = 7;
    instance->encoder.size_upload = 256;
    instance->encoder.upload = malloc(instance->encoder.size_upload * sizeof(LevelDuration));
    instance->encoder.is_running = false;
    instance->encoder.front = 0;

    return instance;
}

void subghz_protocol_encoder_scher_khan_free(void* context) {
    furi_check(context);
    SubGhzProtocolEncoderScherKhan* instance = context;
    free(instance->encoder.upload);
    free(instance);
}

/**
 * Build the RF upload buffer matching the real Scher-Khan waveform.
 *
 * Real signal structure (from Boot_sh5.sub / Trunk_sh5.sub analysis):
 *
 *   Preamble:  6x pairs of ~750µs high / ~750µs low
 *   Header:    3x pairs of ~1500µs high / ~1500µs low
 *   Start bit: 1x pair  of ~750µs high / ~750µs low
 *   Data:      bit 0 = ~750µs  high / ~750µs  low
 *              bit 1 = ~1100µs high / ~1100µs low
 *   End:       ~1500µs high (marks frame end for decoder)
 */
static bool subghz_protocol_encoder_scher_khan_get_upload(
    SubGhzProtocolEncoderScherKhan* instance,
    uint8_t btn) {
    furi_check(instance);

    // For 51-bit dynamic: rebuild data with new button and incremented counter
    if(instance->generic.data_count_bit == 51) {
        if((instance->generic.cnt + 1) > 0xFFFF) {
            instance->generic.cnt = 0;
        } else {
            instance->generic.cnt += 1;
        }

        uint64_t upper = instance->generic.data & 0x7FFFFFFF0000ULL;
        upper = (upper & ~(0x0FULL << 24)) | ((uint64_t)(btn & 0x0F) << 24);
        instance->generic.data = upper | (instance->generic.cnt & 0xFFFF);
        instance->generic.btn = btn;
    }

    size_t index = 0;
    size_t needed = (6 * 2) + (3 * 2) + (1 * 2) + (instance->generic.data_count_bit * 2) + 1;
    if(needed > instance->encoder.size_upload) {
        FURI_LOG_E(TAG, "Upload buffer too small: need %zu, have %zu",
            needed, instance->encoder.size_upload);
        return false;
    }

    for(uint8_t i = 0; i < 6; i++) {
        instance->encoder.upload[index++] =
            level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_short);
        instance->encoder.upload[index++] =
            level_duration_make(false, (uint32_t)subghz_protocol_scher_khan_const.te_short);
    }

    for(uint8_t i = 0; i < 3; i++) {
        instance->encoder.upload[index++] =
            level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_short * 2);
        instance->encoder.upload[index++] =
            level_duration_make(false, (uint32_t)subghz_protocol_scher_khan_const.te_short * 2);
    }

    instance->encoder.upload[index++] =
        level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_short);
    instance->encoder.upload[index++] =
        level_duration_make(false, (uint32_t)subghz_protocol_scher_khan_const.te_short);

    for(uint8_t i = instance->generic.data_count_bit; i > 0; i--) {
        if(bit_read(instance->generic.data, i - 1)) {
            instance->encoder.upload[index++] =
                level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_long);
            instance->encoder.upload[index++] =
                level_duration_make(false, (uint32_t)subghz_protocol_scher_khan_const.te_long);
        } else {
            instance->encoder.upload[index++] =
                level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_short);
            instance->encoder.upload[index++] =
                level_duration_make(false, (uint32_t)subghz_protocol_scher_khan_const.te_short);
        }
    }

    instance->encoder.upload[index++] =
        level_duration_make(true, (uint32_t)subghz_protocol_scher_khan_const.te_short * 2);

    instance->encoder.size_upload = index;

    FURI_LOG_I(TAG, "Upload built: %zu entries, %d bits, btn=0x%02X, cnt=0x%04lX, data=0x%016llX",
        index, instance->generic.data_count_bit, instance->generic.btn,
        (unsigned long)instance->generic.cnt, instance->generic.data);

    return true;
}

static SubGhzProtocolStatus subghz_protocol_encoder_scher_khan_serialize_internal(
    SubGhzProtocolEncoderScherKhan* instance,
    FlipperFormat* flipper_format) {

    const char* pname = NULL;
    subghz_protocol_scher_khan_check_remote_controller(&instance->generic, &pname);

    SubGhzProtocolStatus ret = SubGhzProtocolStatusError;

    do {
        if(!flipper_format_insert_or_update_string_cstr(
               flipper_format, "Protocol", instance->generic.protocol_name)) {
            break;
        }

        uint32_t bits = instance->generic.data_count_bit;
        if(!flipper_format_insert_or_update_uint32(flipper_format, "Bit", &bits, 1)) {
            break;
        }

        char key_str[20];
        snprintf(key_str, sizeof(key_str), "%016llX", instance->generic.data);
        if(!flipper_format_insert_or_update_string_cstr(flipper_format, "Key", key_str)) {
            break;
        }

        if(!flipper_format_insert_or_update_uint32(
               flipper_format, "Serial", &instance->generic.serial, 1)) {
            break;
        }

        uint32_t temp = instance->generic.btn;
        if(!flipper_format_insert_or_update_uint32(flipper_format, "Btn", &temp, 1)) {
            break;
        }

        if(!flipper_format_insert_or_update_uint32(
               flipper_format, "Cnt", &instance->generic.cnt, 1)) {
            break;
        }

        ret = SubGhzProtocolStatusOk;
    } while(false);

    return ret;
}

SubGhzProtocolStatus
    subghz_protocol_encoder_scher_khan_deserialize(void* context, FlipperFormat* flipper_format) {
    furi_check(context);
    SubGhzProtocolEncoderScherKhan* instance = context;
    SubGhzProtocolStatus ret = SubGhzProtocolStatusError;

    flipper_format_rewind(flipper_format);

    do {
        FuriString* temp_str = furi_string_alloc();
        if(!flipper_format_read_string(flipper_format, "Protocol", temp_str)) {
            FURI_LOG_E(TAG, "Missing Protocol");
            furi_string_free(temp_str);
            break;
        }

        if(!furi_string_equal(temp_str, instance->base.protocol->name)) {
            FURI_LOG_E(TAG, "Wrong protocol %s != %s",
                furi_string_get_cstr(temp_str), instance->base.protocol->name);
            furi_string_free(temp_str);
            break;
        }
        furi_string_free(temp_str);

        uint32_t bit_count_temp;
        if(!flipper_format_read_uint32(flipper_format, "Bit", &bit_count_temp, 1)) {
            FURI_LOG_E(TAG, "Missing Bit");
            break;
        }
        instance->generic.data_count_bit = (uint8_t)bit_count_temp;

        temp_str = furi_string_alloc();
        if(!flipper_format_read_string(flipper_format, "Key", temp_str)) {
            FURI_LOG_E(TAG, "Missing Key");
            furi_string_free(temp_str);
            break;
        }

        const char* key_str = furi_string_get_cstr(temp_str);
        uint64_t key = 0;
        size_t str_len = strlen(key_str);
        size_t hex_pos = 0;
        for(size_t i = 0; i < str_len && hex_pos < 16; i++) {
            char c = key_str[i];
            if(c == ' ') continue;

            uint8_t nibble;
            if(c >= '0' && c <= '9') {
                nibble = c - '0';
            } else if(c >= 'A' && c <= 'F') {
                nibble = c - 'A' + 10;
            } else if(c >= 'a' && c <= 'f') {
                nibble = c - 'a' + 10;
            } else {
                FURI_LOG_E(TAG, "Invalid hex character: %c", c);
                furi_string_free(temp_str);
                break;
            }
            key = (key << 4) | nibble;
            hex_pos++;
        }
        furi_string_free(temp_str);

        if(hex_pos == 0) {
            FURI_LOG_E(TAG, "Invalid key length");
            break;
        }

        instance->generic.data = key;
        FURI_LOG_I(TAG, "Parsed key: 0x%016llX", instance->generic.data);

        if(instance->generic.data == 0) {
            FURI_LOG_E(TAG, "Key is zero after parsing!");
            break;
        }

        if(!flipper_format_read_uint32(flipper_format, "Serial", &instance->generic.serial, 1)) {
            instance->generic.serial = 0;
            FURI_LOG_I(TAG, "Serial not found, defaulting to 0");
        } else {
            FURI_LOG_I(TAG, "Read serial: 0x%08lX", instance->generic.serial);
        }

        uint32_t btn_temp;
        if(flipper_format_read_uint32(flipper_format, "Btn", &btn_temp, 1)) {
            instance->generic.btn = (uint8_t)btn_temp;
            FURI_LOG_I(TAG, "Read button: 0x%02X", instance->generic.btn);
        } else {
            instance->generic.btn = 0;
            FURI_LOG_I(TAG, "Button not found, defaulting to 0");
        }

        subghz_custom_btn_set_original(scher_khan_btn_to_custom(instance->generic.btn));
        subghz_custom_btn_set_max(4);

        uint32_t cnt_temp;
        if(flipper_format_read_uint32(flipper_format, "Cnt", &cnt_temp, 1)) {
            instance->generic.cnt = (uint16_t)cnt_temp;
            FURI_LOG_I(TAG, "Read counter: 0x%04lX", (unsigned long)instance->generic.cnt);
        } else {
            instance->generic.cnt = 0;
        }

        if(!flipper_format_read_uint32(
               flipper_format, "Repeat", (uint32_t*)&instance->encoder.repeat, 1)) {
            instance->encoder.repeat = 7;
            FURI_LOG_D(TAG, "Repeat not found, using default 7");
        }

        const char* pname = NULL;
        subghz_protocol_scher_khan_check_remote_controller(&instance->generic, &pname);
        instance->protocol_name = pname;

        uint8_t selected_btn = scher_khan_get_btn_code(instance->generic.btn);

        FURI_LOG_I(TAG,
            "Building upload: original_btn=0x%02X, selected_btn=0x%02X, bits=%d",
            instance->generic.btn, selected_btn, instance->generic.data_count_bit);

        if(!subghz_protocol_encoder_scher_khan_get_upload(instance, selected_btn)) {
            FURI_LOG_E(TAG, "Failed to generate upload");
            break;
        }

        subghz_protocol_encoder_scher_khan_serialize_internal(instance, flipper_format);

        instance->encoder.is_running = true;
        instance->encoder.front = 0;

        FURI_LOG_I(TAG, "Encoder ready: repeat=%u, size_upload=%zu",
            instance->encoder.repeat, instance->encoder.size_upload);

        ret = SubGhzProtocolStatusOk;
    } while(false);

    return ret;
}

void subghz_protocol_encoder_scher_khan_stop(void* context) {
    furi_check(context);
    SubGhzProtocolEncoderScherKhan* instance = context;
    instance->encoder.is_running = false;
}

LevelDuration subghz_protocol_encoder_scher_khan_yield(void* context) {
    SubGhzProtocolEncoderScherKhan* instance = context;

    if(instance->encoder.repeat == 0 || !instance->encoder.is_running) {
        instance->encoder.is_running = false;
        return level_duration_reset();
    }

    LevelDuration ret = instance->encoder.upload[instance->encoder.front];

    if(++instance->encoder.front == instance->encoder.size_upload) {
        instance->encoder.repeat--;
        instance->encoder.front = 0;
    }

    return ret;
}

// ======================== DECODER ========================

void* subghz_protocol_decoder_scher_khan_alloc(SubGhzEnvironment* environment) {
    UNUSED(environment);
    SubGhzProtocolDecoderScherKhan* instance = malloc(sizeof(SubGhzProtocolDecoderScherKhan));
    instance->base.protocol = &subghz_protocol_scher_khan;
    instance->generic.protocol_name = instance->base.protocol->name;

    return instance;
}

void subghz_protocol_decoder_scher_khan_free(void* context) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;
    free(instance);
}

void subghz_protocol_decoder_scher_khan_reset(void* context) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;
    instance->decoder.parser_step = ScherKhanDecoderStepReset;
}

void subghz_protocol_decoder_scher_khan_feed(void* context, bool level, uint32_t duration) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;

    switch(instance->decoder.parser_step) {
    case ScherKhanDecoderStepReset:
        if((level) && (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short * 2) <
                       subghz_protocol_scher_khan_const.te_delta)) {
            instance->decoder.parser_step = ScherKhanDecoderStepCheckPreambula;
            instance->decoder.te_last = duration;
            instance->header_count = 0;
        }
        break;
    case ScherKhanDecoderStepCheckPreambula:
        if(level) {
            if((DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short * 2) <
                subghz_protocol_scher_khan_const.te_delta) ||
               (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short) <
                subghz_protocol_scher_khan_const.te_delta)) {
                instance->decoder.te_last = duration;
            } else {
                instance->decoder.parser_step = ScherKhanDecoderStepReset;
            }
        } else if(
            (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short * 2) <
             subghz_protocol_scher_khan_const.te_delta) ||
            (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short) <
             subghz_protocol_scher_khan_const.te_delta)) {
            if(DURATION_DIFF(
                   instance->decoder.te_last, subghz_protocol_scher_khan_const.te_short * 2) <
               subghz_protocol_scher_khan_const.te_delta) {
                instance->header_count++;
                break;
            } else if(
                DURATION_DIFF(
                    instance->decoder.te_last, subghz_protocol_scher_khan_const.te_short) <
                subghz_protocol_scher_khan_const.te_delta) {
                if(instance->header_count >= 2) {
                    instance->decoder.parser_step = ScherKhanDecoderStepSaveDuration;
                    instance->decoder.decode_data = 0;
                    instance->decoder.decode_count_bit = 1;
                } else {
                    instance->decoder.parser_step = ScherKhanDecoderStepReset;
                }
            } else {
                instance->decoder.parser_step = ScherKhanDecoderStepReset;
            }
        } else {
            instance->decoder.parser_step = ScherKhanDecoderStepReset;
        }
        break;
    case ScherKhanDecoderStepSaveDuration:
        if(level) {
            if(duration >= (subghz_protocol_scher_khan_const.te_delta * 2UL +
                            subghz_protocol_scher_khan_const.te_long)) {
                instance->decoder.parser_step = ScherKhanDecoderStepReset;
                if(instance->decoder.decode_count_bit >=
                   subghz_protocol_scher_khan_const.min_count_bit_for_found) {
                    instance->generic.data = instance->decoder.decode_data;
                    instance->generic.data_count_bit = instance->decoder.decode_count_bit;
                    if(instance->base.callback)
                        instance->base.callback(&instance->base, instance->base.context);
                }
                instance->decoder.decode_data = 0;
                instance->decoder.decode_count_bit = 0;
                break;
            } else {
                instance->decoder.te_last = duration;
                instance->decoder.parser_step = ScherKhanDecoderStepCheckDuration;
            }

        } else {
            instance->decoder.parser_step = ScherKhanDecoderStepReset;
        }
        break;
    case ScherKhanDecoderStepCheckDuration:
        if(!level) {
            if((DURATION_DIFF(
                    instance->decoder.te_last, subghz_protocol_scher_khan_const.te_short) <
                subghz_protocol_scher_khan_const.te_delta) &&
               (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_short) <
                subghz_protocol_scher_khan_const.te_delta)) {
                subghz_protocol_blocks_add_bit(&instance->decoder, 0);
                instance->decoder.parser_step = ScherKhanDecoderStepSaveDuration;
            } else if(
                (DURATION_DIFF(
                     instance->decoder.te_last, subghz_protocol_scher_khan_const.te_long) <
                 subghz_protocol_scher_khan_const.te_delta) &&
                (DURATION_DIFF(duration, subghz_protocol_scher_khan_const.te_long) <
                 subghz_protocol_scher_khan_const.te_delta)) {
                subghz_protocol_blocks_add_bit(&instance->decoder, 1);
                instance->decoder.parser_step = ScherKhanDecoderStepSaveDuration;
            } else {
                instance->decoder.parser_step = ScherKhanDecoderStepReset;
            }
        } else {
            instance->decoder.parser_step = ScherKhanDecoderStepReset;
        }
        break;
    }
}

static void subghz_protocol_scher_khan_check_remote_controller(
    SubGhzBlockGeneric* instance,
    const char** protocol_name) {

    switch(instance->data_count_bit) {
    case 35:
        *protocol_name = "MAGIC CODE, Static";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    case 51:
        *protocol_name = "MAGIC CODE, Dynamic";
        instance->serial = ((instance->data >> 24) & 0xFFFFFF0) | ((instance->data >> 20) & 0x0F);
        instance->btn = (instance->data >> 24) & 0x0F;
        instance->cnt = instance->data & 0xFFFF;
        break;
    case 57:
        *protocol_name = "MAGIC CODE PRO/PRO2";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    case 63:
        *protocol_name = "MAGIC CODE, Response";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    case 64:
        *protocol_name = "MAGICAR, Response";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    case 81:
    case 82:
        *protocol_name = "MAGIC CODE PRO,\n Response";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    default:
        *protocol_name = "Unknown";
        instance->serial = 0;
        instance->btn = 0;
        instance->cnt = 0;
        break;
    }
}

uint8_t subghz_protocol_decoder_scher_khan_get_hash_data(void* context) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;
    return subghz_protocol_blocks_get_hash_data(
        &instance->decoder, (instance->decoder.decode_count_bit / 8) + 1);
}

SubGhzProtocolStatus subghz_protocol_decoder_scher_khan_serialize(
    void* context,
    FlipperFormat* flipper_format,
    SubGhzRadioPreset* preset) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;

    const char* pname = NULL;
    subghz_protocol_scher_khan_check_remote_controller(&instance->generic, &pname);

    SubGhzProtocolStatus ret = SubGhzProtocolStatusError;

    do {
        if(!flipper_format_write_header_cstr(
               flipper_format, "Flipper SubGhz Key File", 1)) {
            break;
        }
        if(preset != NULL) {
            if(!flipper_format_insert_or_update_uint32(
                   flipper_format, "Frequency", &preset->frequency, 1)) {
                break;
            }
            FuriString* preset_str = furi_string_alloc();
            subghz_block_generic_get_preset_name(
                furi_string_get_cstr(preset->name), preset_str);
            if(!flipper_format_insert_or_update_string_cstr(
                   flipper_format, "Preset", furi_string_get_cstr(preset_str))) {
                furi_string_free(preset_str);
                break;
            }
            furi_string_free(preset_str);
        }

        if(!flipper_format_insert_or_update_string_cstr(
               flipper_format, "Protocol", instance->generic.protocol_name)) {
            break;
        }

        uint32_t bits = instance->generic.data_count_bit;
        if(!flipper_format_insert_or_update_uint32(flipper_format, "Bit", &bits, 1)) {
            break;
        }

        char key_str[20];
        snprintf(key_str, sizeof(key_str), "%016llX", instance->generic.data);
        if(!flipper_format_insert_or_update_string_cstr(flipper_format, "Key", key_str)) {
            break;
        }

        if(!flipper_format_insert_or_update_uint32(
               flipper_format, "Serial", &instance->generic.serial, 1)) {
            break;
        }

        uint32_t temp = instance->generic.btn;
        if(!flipper_format_insert_or_update_uint32(flipper_format, "Btn", &temp, 1)) {
            break;
        }

        if(!flipper_format_insert_or_update_uint32(
               flipper_format, "Cnt", &instance->generic.cnt, 1)) {
            break;
        }

        ret = SubGhzProtocolStatusOk;
    } while(false);

    return ret;
}

SubGhzProtocolStatus
    subghz_protocol_decoder_scher_khan_deserialize(void* context, FlipperFormat* flipper_format) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;
    SubGhzProtocolStatus ret = SubGhzProtocolStatusError;

    flipper_format_rewind(flipper_format);

    do {
        FuriString* temp_str = furi_string_alloc();
        if(!flipper_format_read_string(flipper_format, "Protocol", temp_str)) {
            FURI_LOG_E(TAG, "Missing Protocol");
            furi_string_free(temp_str);
            break;
        }

        if(!furi_string_equal(temp_str, instance->base.protocol->name)) {
            FURI_LOG_E(TAG, "Wrong protocol %s != %s",
                furi_string_get_cstr(temp_str), instance->base.protocol->name);
            furi_string_free(temp_str);
            break;
        }
        furi_string_free(temp_str);

        uint32_t bit_count_temp;
        if(!flipper_format_read_uint32(flipper_format, "Bit", &bit_count_temp, 1)) {
            FURI_LOG_E(TAG, "Missing Bit");
            break;
        }
        instance->generic.data_count_bit = (uint8_t)bit_count_temp;

        temp_str = furi_string_alloc();
        if(!flipper_format_read_string(flipper_format, "Key", temp_str)) {
            FURI_LOG_E(TAG, "Missing Key");
            furi_string_free(temp_str);
            break;
        }

        const char* key_str = furi_string_get_cstr(temp_str);
        uint64_t key = 0;
        size_t str_len = strlen(key_str);
        size_t hex_pos = 0;
        for(size_t i = 0; i < str_len && hex_pos < 16; i++) {
            char c = key_str[i];
            if(c == ' ') continue;
            uint8_t nibble;
            if(c >= '0' && c <= '9') nibble = c - '0';
            else if(c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
            else if(c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
            else {
                FURI_LOG_E(TAG, "Invalid hex character: %c", c);
                furi_string_free(temp_str);
                break;
            }
            key = (key << 4) | nibble;
            hex_pos++;
        }
        furi_string_free(temp_str);

        if(hex_pos == 0) {
            FURI_LOG_E(TAG, "Invalid key length");
            break;
        }

        instance->generic.data = key;

        if(!flipper_format_read_uint32(flipper_format, "Serial", &instance->generic.serial, 1)) {
            instance->generic.serial = 0;
        }

        uint32_t btn_temp;
        if(flipper_format_read_uint32(flipper_format, "Btn", &btn_temp, 1)) {
            instance->generic.btn = (uint8_t)btn_temp;
        } else {
            instance->generic.btn = 0;
        }

        uint32_t cnt_temp;
        if(flipper_format_read_uint32(flipper_format, "Cnt", &cnt_temp, 1)) {
            instance->generic.cnt = (uint16_t)cnt_temp;
        } else {
            instance->generic.cnt = 0;
        }

        FURI_LOG_I(TAG, "Decoder deserialized");
        ret = SubGhzProtocolStatusOk;
    } while(false);

    return ret;
}

void subghz_protocol_decoder_scher_khan_get_string(void* context, FuriString* output) {
    furi_check(context);
    SubGhzProtocolDecoderScherKhan* instance = context;

    subghz_protocol_scher_khan_check_remote_controller(
        &instance->generic, &instance->protocol_name);

    subghz_custom_btn_set_original(scher_khan_btn_to_custom(instance->generic.btn));
    subghz_custom_btn_set_max(4);

    furi_string_cat_printf(
        output,
        "%s %dbit\r\n"
        "Key:0x%lX%08lX\r\n"
        "Sn:%07lX Btn:[%s]\r\n"
        "Cntr:%04lX\r\n"
        "Pt: %s\r\n",
        instance->generic.protocol_name,
        instance->generic.data_count_bit,
        (uint32_t)(instance->generic.data >> 32),
        (uint32_t)instance->generic.data,
        instance->generic.serial,
        scher_khan_btn_name(scher_khan_get_btn_code(instance->generic.btn)),
        instance->generic.cnt,
        instance->protocol_name);
}
