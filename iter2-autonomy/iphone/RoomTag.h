#pragma once
#include <stdint.h>
#include <simd/simd.h>

// Call only from the serial tag worker. Input is the native camera luma plane.
int room_tag_detect(uint8_t *pixels, int width, int height, int stride,
                    float fx, float fy, float cx, float cy, int preferred,
                    simd_float4x4 *camera_from_tag);
