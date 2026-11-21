#include "RoomTag.h"
#include "../../vendor/apriltag/apriltag_pose.h"
#include "../../vendor/apriltag/tag36h11.h"
#include <math.h>

int room_tag_detect(uint8_t *pixels, int width, int height, int stride,
                    float fx, float fy, float cx, float cy, int preferred,
                    simd_float4x4 *camera_from_tag)
{
    static apriltag_detector_t *detector;
    if (!detector) {
        detector = apriltag_detector_create();
        apriltag_detector_add_family_bits(detector, tag36h11_create(), 1);
        detector->quad_decimate = 2;
    }
    image_u8_t im = {width, height, stride, pixels};
    zarray_t *hits = apriltag_detector_detect(detector, &im);
    apriltag_detection_t *best = NULL;
    for (int i = 0; i < zarray_size(hits); i++) {
        apriltag_detection_t *d;
        zarray_get(hits, i, &d);
        if (d->hamming != 0 || d->decision_margin < 40) continue;
        if (!best || (d->id == preferred && best->id != preferred) ||
            (best->id != preferred && d->decision_margin > best->decision_margin)) best = d;
    }
    int id = -1;
    if (best) {
        apriltag_detection_info_t info = {best, 0.16, fx, fy, cx, cy};
        apriltag_pose_t pose;
        double error = estimate_tag_pose(&info, &pose);
        const double z = MATD_EL(pose.t, 2, 0);
        if (isfinite(error) && error < 0.002 && z > 0.15 && z < 4) {
            *camera_from_tag = matrix_identity_float4x4;
            for (int row = 0; row < 3; row++) {
                float sign = row == 0 ? 1 : -1; // optical -> ARKit camera axes
                for (int col = 0; col < 3; col++)
                    camera_from_tag->columns[col][row] = sign * MATD_EL(pose.R, row, col);
                camera_from_tag->columns[3][row] = sign * MATD_EL(pose.t, row, 0);
            }
            id = best->id;
        }
        matd_destroy(pose.R);
        matd_destroy(pose.t);
    }
    apriltag_detections_destroy(hits);
    return id;
}
