#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define DT (0.01)
#define FILESIZE (256*1024)

typedef struct {
    double x;
    double y;
    double z;
} Vector3;

const char* fileName = "accel.log";

void readOneEntry(FILE *f, Vector3 *res) {
    uint8_t buff[6] = {0};

    fread(buff, 6, 1, f);

    int16_t x = (int16_t)((uint16_t)buff[0] << 8 | buff[1]);
    int16_t y = (int16_t)((uint16_t)buff[2] << 8 | buff[3]);
    int16_t z = (int16_t)((uint16_t)buff[4] << 8 | buff[5]);

    res->x = x / 100.0;
    res->y = y / 100.0;
    res->z = z / 100.0;
}

int main(void) {
    Vector3 v_accum = {0};
    Vector3 r_accum = {0};

    Vector3 a_scratch = {0};
    Vector3 a_scratch_pre = {0};

    Vector3 v_scratch = {0};
    Vector3 v_scratch_pre = {0};

    FILE *log = fopen(fileName, "rb");

    if (!log) {
        printf("Failed to open file\n");
        return 1;
    }

    readOneEntry(log, &a_scratch_pre);

    for (size_t i = 1; i < FILESIZE/6; i++) {
        readOneEntry(log, &a_scratch);

        v_accum.x += ((a_scratch.x + a_scratch_pre.x)/2.0) * DT;
        v_accum.y += ((a_scratch.y + a_scratch_pre.y)/2.0) * DT;
        v_accum.z += ((a_scratch.z + a_scratch_pre.z)/2.0) * DT;

        v_scratch = v_accum;

        a_scratch_pre = a_scratch;

        r_accum.x += ((v_scratch.x + v_scratch_pre.x)/2.0) * DT;
        r_accum.y += ((v_scratch.y + v_scratch_pre.y)/2.0) * DT;
        r_accum.z += ((v_scratch.z + v_scratch_pre.z)/2.0) * DT;

        v_scratch_pre = v_scratch;

        printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", DT*i, a_scratch.x, a_scratch.y, a_scratch.z, v_accum.x, v_accum.y, v_accum.z, r_accum.x, r_accum.y, r_accum.z);
    }


    fclose(log);
    return 0;
}
