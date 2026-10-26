/* Crop a binary grid terrain STL to a rectangular XY area, keeping
 * triangles whose 3 vertices are inside the box. Usage:
 *   gcc -O2 -o crop_terrain_stl crop_terrain_stl.c
 *   ./crop_terrain_stl in.stl out.stl xmin xmax ymin ymax
 * Used to derive the ground-50-* collision meshes from ground-100-*.
 * The OBJ counterpart (visual mesh, keeps UVs/textures) is
 * crop_terrain_obj.c.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static int inside(double x, double y, double bx0, double bx1,
                  double by0, double by1)
{
    return x >= bx0 - 1e-6 && x <= bx1 + 1e-6 &&
           y >= by0 - 1e-6 && y <= by1 + 1e-6;
}

int main(int argc, char **argv)
{
    if (argc < 7) {
        fprintf(stderr, "usage: %s in.stl out.stl xmin xmax ymin ymax\n",
                argv[0]);
        return 1;
    }
    double bx0 = atof(argv[3]), bx1 = atof(argv[4]);
    double by0 = atof(argv[5]), by1 = atof(argv[6]);
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror("open stl"); return 1; }
    unsigned char hdr[80];
    if (fread(hdr, 1, 80, f) != 80) return 1;
    uint32_t n;
    if (fread(&n, 4, 1, f) != 1) return 1;
    FILE *o = fopen(argv[2], "wb");
    if (!o) { perror("open out"); return 1; }
    unsigned char ohdr[80];
    memset(ohdr, 0, 80);
    snprintf((char *)ohdr, 79, "vl400 cropped terrain collision mesh");
    fwrite(ohdr, 1, 80, o);
    uint32_t zero = 0;
    fwrite(&zero, 4, 1, o); /* count patched below */
    float v[12];
    uint16_t attr;
    uint32_t kept = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (fread(v, 4, 12, f) != 12) break;
        if (fread(&attr, 2, 1, f) != 1) break;
        int ok = 1;
        for (int j = 0; j < 3; j++) {
            if (!inside(v[3 + 3 * j], v[4 + 3 * j], bx0, bx1, by0, by1)) {
                ok = 0;
                break;
            }
        }
        if (!ok) continue;
        fwrite(v, 4, 12, o);
        uint16_t z16 = 0;
        fwrite(&z16, 2, 1, o);
        kept++;
    }
    fclose(f);
    fseek(o, 80, SEEK_SET);
    fwrite(&kept, 4, 1, o);
    fclose(o);
    fprintf(stderr, "stl: kept %u / %u triangles\n", kept, n);
    return 0;
}
