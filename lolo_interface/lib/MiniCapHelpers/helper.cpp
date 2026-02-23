#include "helper.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/* ------------------- Print/Set Fields ------------------- */
bool setFieldByName(void* data, FieldDescriptor_t* fields, size_t fieldCount, const char* name, float value) {
    for (size_t i = 0; i < fieldCount; ++i) {
        if (strcasecmp(name, fields[i].name) == 0) {
            uint8_t* base = (uint8_t*)data + fields[i].offset;
            switch (fields[i].type) {
                case F_FLOAT:   ((float*)base)[0] = value; return true;
                case F_UINT8:   ((uint8_t*)base)[0] = (uint8_t)value; return true;
                case F_UINT16:  ((uint16_t*)base)[0] = (uint16_t)value; return true;
                case F_UINT32:  ((uint32_t*)base)[0] = (uint32_t)value; return true;
                case F_UINT64:  ((uint64_t*)base)[0] = (uint64_t)value; return true;
            }
        }
    }
    return false;
}

bool getFieldByName(void* data, FieldDescriptor_t* fields, size_t fieldCount, const char* name, char* outStr, size_t outStrLen) {
    for (size_t i = 0; i < fieldCount; ++i) {
        if (strcasecmp(name, fields[i].name) == 0) {
            uint8_t* base = (uint8_t*)data + fields[i].offset;
            switch (fields[i].type) {
                case F_FLOAT:
                    snprintf(outStr, outStrLen, "%.3f", ((float*)base)[0]);
                    return true;
                case F_UINT8:
                    snprintf(outStr, outStrLen, "%u", ((uint8_t*)base)[0]);
                    return true;
                case F_UINT16:
                    snprintf(outStr, outStrLen, "%u", ((uint16_t*)base)[0]);
                    return true;
                case F_UINT32:
                    snprintf(outStr, outStrLen, "%lu", (unsigned long)((uint32_t*)base)[0]);
                    return true;
                case F_UINT64:
                    snprintf(outStr, outStrLen, "%llu", (unsigned long long)((uint64_t*)base)[0]);
                    return true;
            }
        }
    }
    return false;
}


/* ------------------- Fill Fields ------------------- */
void fillFieldsRandom(void* data, FieldDescriptor_t* fields, size_t fieldCount) {
    for (size_t i = 0; i < fieldCount; i++) {
        uint8_t* base = (uint8_t*)data + fields[i].offset;
        for (size_t j = 0; j < fields[i].count; j++) {
            switch (fields[i].type) {
                case F_UINT8:
                    ((uint8_t*)base)[j] = rand() % 256;
                    break;
                case F_UINT16:
                    ((uint16_t*)base)[j] = rand() % 65536;
                    break;
                case F_UINT32:
                    ((uint32_t*)base)[j] = ((uint32_t)rand() << 16) | rand();
                    break;
                case F_UINT64:
                    ((uint64_t*)base)[j] = ((uint64_t)rand() << 32) | rand();
                    break;
                case F_FLOAT:
                    ((float*)base)[j] = ((float)(rand() % 10000)) / 100.0f;
                    break;
            }
        }
    }
}

/* ------------------- Unpack Fields ------------------- */
void unpackFieldsFromCom(CaptainInterFace* com, void* data, FieldDescriptor_t* fields, size_t fieldCount) {
    for (size_t i = 0; i < fieldCount; i++) {
        uint8_t* base = (uint8_t*)data + fields[i].offset;
        for (size_t j = 0; j < fields[i].count; j++) {
            switch (fields[i].type) {
                case F_UINT8:
                    ((uint8_t*)base)[j] = com->parse_byte();
                    break;
                case F_UINT16:
                    ((uint16_t*)base)[j] = com->parse_int();
                    break;
                case F_UINT32:
                    ((uint32_t*)base)[j] = com->parse_long();
                    break;
                case F_UINT64:
                    ((uint64_t*)base)[j] = com->parse_llong();
                    break;
                case F_FLOAT:
                    ((float*)base)[j] = com->parse_float();
                    break;
            }
        }
    }
}

// Generic: copy all fields from src to dst when names and types match
void mergeStructByFields(void* dst,
                         FieldDescriptor_t* dstFields, size_t dstCount,
                         const void* src,
                         FieldDescriptor_t* srcFields, size_t srcCount)
{
    for (size_t i = 0; i < srcCount; ++i) {
        FieldDescriptor_t& s = srcFields[i];

        // Find field with same name in dst
        FieldDescriptor_t* d = nullptr;
        for (size_t k = 0; k < dstCount; ++k) {
            if (strcmp(srcFields[i].name, dstFields[k].name) == 0) {
                d = &dstFields[k];
                break;
            }
        }
        if (!d) continue;                     // no matching field in dst
        if (d->type != s.type) continue;      // type mismatch, skip
        if (d->count != s.count) continue;    // array length mismatch, skip

        uint8_t* dstBase = (uint8_t*)dst + d->offset;
        const uint8_t* srcBase = (const uint8_t*)src + s.offset;

        switch (s.type) {
        case F_UINT8:
            for (size_t j = 0; j < s.count; ++j)
                ((uint8_t*)dstBase)[j] = ((const uint8_t*)srcBase)[j];
            break;
        case F_UINT16:
            for (size_t j = 0; j < s.count; ++j)
                ((uint16_t*)dstBase)[j] = ((const uint16_t*)srcBase)[j];
            break;
        case F_UINT32:
            for (size_t j = 0; j < s.count; ++j)
                ((uint32_t*)dstBase)[j] = ((const uint32_t*)srcBase)[j];
            break;
        case F_UINT64:
            for (size_t j = 0; j < s.count; ++j)
                ((uint64_t*)dstBase)[j] = ((const uint64_t*)srcBase)[j];
            break;
        case F_FLOAT:
            for (size_t j = 0; j < s.count; ++j)
                ((float*)dstBase)[j] = ((const float*)srcBase)[j];
            break;
        }
    }
}


inline bool IsSafetyFlagActive(uint64_t flags, FCSafetyFlags e)
{
    return (flags & (1ULL << e)) != 0;
}



