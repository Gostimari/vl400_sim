#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { char **l; size_t n, cap; } Arr;
static void apush(Arr *a, const char *s)
{
    if (a->n >= a->cap) {
        a->cap = a->cap ? a->cap * 2 : 65536;
        a->l = realloc(a->l, a->cap * sizeof(char *));
        if (!a->l) { fprintf(stderr, "OOM\n"); exit(2); }
    }
    char *p = malloc(strlen(s) + 1);
    if (!p) { fprintf(stderr, "OOM\n"); exit(2); }
    strcpy(p, s);
    a->l[a->n++] = p;
}

static double *vpx, *vpy;
static size_t vpcap;
static Arr V, VT, VN, F, O;   /* vertex lines, tex lines, normal lines,
                                  face lines, other (pre-face) lines */
static int *fkeep;

static int inside(double x, double y, double bx0, double bx1,
                  double by0, double by1)
{
    return x >= bx0 - 1e-6 && x <= bx1 + 1e-6 &&
           y >= by0 - 1e-6 && y <= by1 + 1e-6;
}

int main(int argc, char **argv)
{
    if (argc < 6) { fprintf(stderr, "usage: %s in.obj out.obj xmin xmax ymin ymax\n", argv[0]); return 1; }
    double bx0 = atof(argv[3]), bx1 = atof(argv[4]);
    double by0 = atof(argv[5]), by1 = atof(argv[6]);
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror("open"); return 1; }
    char buf[512];
    int seenF = 0;
    while (fgets(buf, sizeof(buf), f)) {
        if (strncmp(buf, "v ", 2) == 0) {
            double x, y, z;
            sscanf(buf + 2, "%lf %lf %lf", &x, &y, &z);
            if (V.n >= vpcap) {
                vpcap = vpcap ? vpcap * 2 : 65536;
                vpx = realloc(vpx, vpcap * sizeof(double));
                vpy = realloc(vpy, vpcap * sizeof(double));
                if (!vpx || !vpy) { fprintf(stderr, "OOM\n"); return 2; }
            }
            vpx[V.n] = x; vpy[V.n] = y;
            apush(&V, buf);
        } else if (strncmp(buf, "vt ", 3) == 0) {
            apush(&VT, buf);
        } else if (strncmp(buf, "vn ", 3) == 0) {
            apush(&VN, buf);
        } else if (buf[0] == 'f') {
            seenF = 1;
            int keep = 1;
            char *p = buf + 1;
            while (*p) {
                while (*p == ' ' || *p == '\t') p++;
                if (!*p || *p == '\n') break;
                char *end;
                long idx = strtol(p, &end, 10);
                if (end == p) break;
                if (idx > 0 && (size_t)idx <= V.n) {
                    if (!inside(vpx[idx-1], vpy[idx-1], bx0, bx1, by0, by1))
                        keep = 0;
                }
                p = end;
                while (*p && *p != ' ' && *p != '\t' && *p != '\n') p++;
            }
            if (F.n >= F.cap) {
                F.cap = F.cap ? F.cap * 2 : 65536;
                F.l = realloc(F.l, F.cap * sizeof(char *));
                fkeep = realloc(fkeep, F.cap * sizeof(int));
                if (!F.l || !fkeep) { fprintf(stderr, "OOM\n"); return 2; }
            }
            char *cp = malloc(strlen(buf) + 1);
            strcpy(cp, buf);
            F.l[F.n] = cp;
            fkeep[F.n] = keep;
            F.n++;
        } else if (!seenF) {
            apush(&O, buf);
        }
    }
    fclose(f);
    /* remap tables */
    int *vnew = calloc(V.n + 1, sizeof(int));
    int *vtnew = calloc(VT.n + 1, sizeof(int));
    int *vnnew = calloc(VN.n + 1, sizeof(int));
    if (!vnew || !vtnew || !vnnew) { fprintf(stderr, "OOM\n"); return 2; }
    /* mark used indices from kept faces */
    for (size_t i = 0; i < F.n; i++) {
        if (!fkeep[i]) continue;
        char *p = F.l[i] + 1;
        while (*p) {
            while (*p == ' ' || *p == '\t') p++;
            if (!*p || *p == '\n') break;
            char *tok = p;
            while (*p && *p != ' ' && *p != '\t' && *p != '\n') p++;
            char *tb = malloc(p - tok + 1);
            strncpy(tb, tok, p - tok);
            tb[p - tok] = 0;
            char *s1 = strtok(tb, "/");
            char *s2 = strtok(NULL, "/");
            char *s3 = strtok(NULL, "/");
            long a = s1 ? atol(s1) : 0, b = s2 ? atol(s2) : 0,
                 c = s3 ? atol(s3) : 0;
            if (a > 0 && (size_t)a <= V.n) vnew[a] = 1;
            if (b > 0 && (size_t)b <= VT.n) vtnew[b] = 1;
            if (c > 0 && (size_t)c <= VN.n) vnnew[c] = 1;
            free(tb);
        }
    }
    size_t nvout = 0, nvtout = 0, nvnout = 0;
    for (size_t i = 1; i <= V.n; i++) if (vnew[i]) vnew[i] = ++nvout;
    for (size_t i = 1; i <= VT.n; i++) if (vtnew[i]) vtnew[i] = ++nvtout;
    for (size_t i = 1; i <= VN.n; i++) if (vnnew[i]) vnnew[i] = ++nvnout;
    FILE *o = fopen(argv[2], "w");
    if (!o) { perror("open out"); return 1; }
    for (size_t i = 0; i < O.n; i++) fputs(O.l[i], o);
    for (size_t i = 0; i < V.n; i++) if (vnew[i + 1]) fputs(V.l[i], o);
    for (size_t i = 0; i < VT.n; i++) if (vtnew[i + 1]) fputs(VT.l[i], o);
    for (size_t i = 0; i < VN.n; i++) if (vnnew[i + 1]) fputs(VN.l[i], o);
    for (size_t i = 0; i < O.n; i++)
        if (strncmp(O.l[i], "usemtl", 6) == 0) fputs(O.l[i], o);
    size_t nfout = 0;
    for (size_t i = 0; i < F.n; i++) {
        if (!fkeep[i]) continue;
        fputc('f', o);
        char *p = F.l[i] + 1;
        while (*p) {
            while (*p == ' ' || *p == '\t') p++;
            if (!*p || *p == '\n') break;
            char *tok = p;
            while (*p && *p != ' ' && *p != '\t' && *p != '\n') p++;
            char *tb = malloc(p - tok + 1);
            strncpy(tb, tok, p - tok);
            tb[p - tok] = 0;
            char *s1 = strtok(tb, "/");
            char *s2 = strtok(NULL, "/");
            char *s3 = strtok(NULL, "/");
            long a = s1 ? atol(s1) : 0, b = s2 ? atol(s2) : 0,
                 c = s3 ? atol(s3) : 0;
            fputc(' ', o);
            if (a > 0 && (size_t)a <= V.n) fprintf(o, "%d", vnew[a]);
            if (b || c) {
                fputc('/', o);
                if (b > 0 && (size_t)b <= VT.n) fprintf(o, "%d", vtnew[b]);
                if (c) {
                    fputc('/', o);
                    if (c > 0 && (size_t)c <= VN.n) fprintf(o, "%d", vnnew[c]);
                }
            }
            free(tb);
            nfout++;
        }
        fputc('\n', o);
    }
    fclose(o);
    fprintf(stderr, "obj: %zu faces kept, %zu v, %zu vt, %zu vn\n",
            nfout / 3, nvout, nvtout, nvnout);
    return 0;
}
