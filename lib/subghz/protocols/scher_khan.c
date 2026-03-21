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
};

typedef enum {
    ScherKhanDecoderStepReset = 0,
    ScherKhanDecoderStepCheckPreambula,
    ScherKhanDecoderStepSaveDuration,
    ScherKhanDecoderStepCheckDuration,
} ScherKhanDecoderStep;

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
    .alloc = NULL,
    .free = NULL,

    .deserialize = NULL,
    .stop = NULL,
    .yield = NULL,
};

const SubGhzProtocol subghz_protocol_scher_khan = {
    .name = SUBGHZ_PROTOCOL_SCHER_KHAN_NAME,
    .type = SubGhzProtocolTypeDynamic,
    .flag = SubGhzProtocolFlag_433 | SubGhzProtocolFlag_FM | SubGhzProtocolFlag_Decodable,

    .decoder = &subghz_protocol_scher_khan_decoder,
    .encoder = &subghz_protocol_scher_khan_encoder,
};

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
    return subghz_block_generic_deserialize(&instance->generic, flipper_format);
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
