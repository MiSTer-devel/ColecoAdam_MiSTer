// cmpframes: compare Verilator core frames (320x240 PPM) with ColEm frames (256x192 PPM)
// by mapping every pixel to its TMS9918 colour index using each emulator's palette.
//
//   cmpframes offset CORE.ppm COLEM.ppm
//       find where the 256x192 display sits inside the core frame
//   cmpframes score DX DY REF.ppm CAND.ppm [CAND.ppm ...]
//       compare REF against each candidate, print the best one
//       (images larger than 256x192 are cropped at DX,DY)
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <string>
#include <utility>
#include <vector>

struct Img {
    int w = 0, h = 0;
    std::vector<unsigned char> rgb;
};

static const int core_pal[16][3] = {
    {0, 0, 0}, {0, 0, 0}, {33, 200, 66}, {94, 220, 120}, {84, 85, 237}, {125, 118, 252},
    {212, 82, 77}, {66, 235, 245}, {252, 85, 84}, {255, 121, 120}, {212, 193, 84},
    {230, 206, 128}, {33, 176, 59}, {201, 91, 186}, {204, 204, 204}, {255, 255, 255}};
static const int colem_pal[16][3] = {
    {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00}, {0x24, 0xDA, 0x24}, {0x6D, 0xFF, 0x6D},
    {0x24, 0x24, 0xFF}, {0x48, 0x6D, 0xFF}, {0xB6, 0x24, 0x24}, {0x48, 0xDA, 0xFF},
    {0xFF, 0x24, 0x24}, {0xFF, 0x6D, 0x6D}, {0xDA, 0xDA, 0x24}, {0xDA, 0xDA, 0x91},
    {0x24, 0x91, 0x24}, {0xDA, 0x48, 0xB6}, {0xB6, 0xB6, 0xB6}, {0xFF, 0xFF, 0xFF}};

static bool readPPM(const char* path, Img& img) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    char magic[3] = {0};
    int maxv;
    if (fscanf(f, "%2s %d %d %d", magic, &img.w, &img.h, &maxv) != 4 || strcmp(magic, "P6") != 0) {
        fclose(f);
        return false;
    }
    fgetc(f);
    img.rgb.resize((size_t)img.w * img.h * 3);
    size_t n = fread(img.rgb.data(), 1, img.rgb.size(), f);
    fclose(f);
    return n == img.rgb.size();
}

// Colour index per pixel; 0 (transparent) and 1 (black) are the same colour on screen.
static std::vector<unsigned char> mapPalette(const Img& img, const int pal[16][3], int* inexact) {
    std::vector<unsigned char> out((size_t)img.w * img.h);
    *inexact = 0;
    for (size_t i = 0; i < out.size(); i++) {
        const unsigned char* p = &img.rgb[i * 3];
        int best = 1, best_d = 1 << 30;
        for (int c = 1; c < 16; c++) {
            int dr = p[0] - pal[c][0], dg = p[1] - pal[c][1], db = p[2] - pal[c][2];
            int d = dr * dr + dg * dg + db * db;
            if (d < best_d) { best_d = d; best = c; }
        }
        if (best_d) (*inexact)++;
        out[i] = (unsigned char)best;
    }
    return out;
}

// Core and ColEm frames use different RGB palettes, and a core frame may already be cropped
// to 256x192, so use whichever palette the image matches exactly.
static std::vector<unsigned char> indices(const Img& img, int* inexact) {
    int core_inexact, colem_inexact;
    std::vector<unsigned char> core = mapPalette(img, core_pal, &core_inexact);
    std::vector<unsigned char> colem = mapPalette(img, colem_pal, &colem_inexact);
    if (core_inexact <= colem_inexact) {
        *inexact = core_inexact;
        return core;
    }
    *inexact = colem_inexact;
    return colem;
}

// frame_NNNNN.ppm files in DIR, sorted by frame number
static std::vector<std::pair<int, std::string>> listFrames(const char* dir) {
    std::vector<std::pair<int, std::string>> out;
    DIR* d = opendir(dir);
    if (!d) return out;
    while (dirent* e = readdir(d)) {
        int n;
        if (sscanf(e->d_name, "frame_%d", &n) == 1 && strstr(e->d_name, ".ppm"))
            out.push_back(std::make_pair(n, std::string(dir) + "/" + e->d_name));
    }
    closedir(d);
    std::sort(out.begin(), out.end());
    return out;
}

static std::vector<unsigned char> crop(const std::vector<unsigned char>& idx, const Img& img, int dx, int dy) {
    if (img.w == 256 && img.h == 192) return idx;
    std::vector<unsigned char> out(256 * 192);
    for (int y = 0; y < 192; y++)
        for (int x = 0; x < 256; x++) out[y * 256 + x] = idx[(y + dy) * img.w + x + dx];
    return out;
}

struct Score {
    double match, fgmatch;
};

static Score score(const std::vector<unsigned char>& a, const std::vector<unsigned char>& b) {
    int hist[16] = {0};
    for (unsigned char c : b) hist[c]++;
    int bg = 1;
    for (int c = 1; c < 16; c++)
        if (hist[c] > hist[bg]) bg = c;
    long mism = 0, fg = 0;
    for (size_t i = 0; i < a.size(); i++) {
        if (a[i] != b[i]) mism++;
        if (a[i] != bg || b[i] != bg) fg++;
    }
    Score s;
    s.match = 1.0 - (double)mism / a.size();
    s.fgmatch = fg ? 1.0 - (double)mism / fg : 1.0;
    return s;
}

static void describe(const char* label, const std::vector<unsigned char>& a) {
    int hist[16] = {0}, colours = 0, dominant = 0;
    for (unsigned char c : a) hist[c]++;
    for (int c = 1; c < 16; c++) {
        if (hist[c]) colours++;
        if (hist[c] > dominant) dominant = hist[c];
    }
    printf(" %s_colours=%d %s_dominant=%.4f", label, colours, label, (double)dominant / a.size());
}

int main(int argc, char** argv) {
    if (argc >= 4 && !strcmp(argv[1], "offset")) {
        Img core, ref;
        if (!readPPM(argv[2], core) || !readPPM(argv[3], ref)) { fprintf(stderr, "cannot read input\n"); return 1; }
        int in1, in2;
        std::vector<unsigned char> ci = indices(core, &in1), ri = indices(ref, &in2);
        long best = -1;
        int bx = 0, by = 0;
        for (int dy = 0; dy + 192 <= core.h; dy++)
            for (int dx = 0; dx + 256 <= core.w; dx++) {
                long same = 0;
                for (int y = 0; y < 192; y++) {
                    const unsigned char* cr = &ci[(y + dy) * core.w + dx];
                    const unsigned char* rr = &ri[y * 256];
                    for (int x = 0; x < 256; x++) same += cr[x] == rr[x];
                }
                if (same > best) { best = same; bx = dx; by = dy; }
            }
        printf("dx=%d dy=%d match=%.4f core_inexact=%d ref_inexact=%d\n", bx, by, best / 49152.0, in1, in2);
        return 0;
    }

    if (argc >= 6 && !strcmp(argv[1], "score")) {
        int dx = atoi(argv[2]), dy = atoi(argv[3]);
        Img ref;
        if (!readPPM(argv[4], ref)) { fprintf(stderr, "cannot read %s\n", argv[4]); return 1; }
        int inexact;
        std::vector<unsigned char> a = crop(indices(ref, &inexact), ref, dx, dy);
        Score best = {-1, -1};
        const char* best_path = "";
        for (int i = 5; i < argc; i++) {
            Img cand;
            if (!readPPM(argv[i], cand)) continue;
            int cin;
            std::vector<unsigned char> b = crop(indices(cand, &cin), cand, dx, dy);
            Score s = score(a, b);
            if (s.match > best.match) { best = s; best_path = argv[i]; }
        }
        if (best.match < 0) { fprintf(stderr, "no readable candidates\n"); return 1; }
        printf("best=%s match=%.4f fgmatch=%.4f ref_inexact=%d", best_path, best.match, best.fgmatch, inexact);
        describe("ref", a);
        printf("\n");
        return 0;
    }

    if (argc >= 6 && !strcmp(argv[1], "align")) {
        // For every CORE frame, the best-matching REF frame anywhere in REFDIR
        // (ties go to the nearest frame number)
        int dx = atoi(argv[2]), dy = atoi(argv[3]);
        std::vector<std::pair<int, std::string>> core = listFrames(argv[4]), ref = listFrames(argv[5]);
        if (core.empty() || ref.empty()) { fprintf(stderr, "no frame_*.ppm files\n"); return 1; }
        std::vector<std::vector<unsigned char>> refIdx;
        for (const auto& r : ref) {
            Img im;
            if (!readPPM(r.second.c_str(), im)) { fprintf(stderr, "cannot read %s\n", r.second.c_str()); return 1; }
            int in;
            refIdx.push_back(crop(indices(im, &in), im, dx, dy));
        }
        for (const auto& c : core) {
            Img im;
            if (!readPPM(c.second.c_str(), im)) continue;
            int in;
            std::vector<unsigned char> a = crop(indices(im, &in), im, dx, dy);
            std::vector<size_t> order(ref.size());
            for (size_t i = 0; i < order.size(); i++) order[i] = i;
            std::stable_sort(order.begin(), order.end(), [&](size_t x, size_t y) {
                return std::abs(ref[x].first - c.first) < std::abs(ref[y].first - c.first);
            });
            long best = -1;
            size_t bi = 0;
            for (size_t i : order) {
                const std::vector<unsigned char>& b = refIdx[i];
                long m = 0;
                for (size_t p = 0; p < a.size() && (best < 0 || m < best); p++) m += a[p] != b[p];
                if (best < 0 || m < best) { best = m; bi = i; }
            }
            Score s = score(a, refIdx[bi]);
            printf("core=%d best=%d offset=%+d match=%.4f fgmatch=%.4f", c.first, ref[bi].first, ref[bi].first - c.first, s.match, s.fgmatch);
            describe("core", a);
            printf("\n");
        }
        return 0;
    }

    fprintf(stderr, "usage: cmpframes offset CORE.ppm COLEM.ppm\n"
                    "       cmpframes score DX DY REF.ppm CAND.ppm [CAND.ppm ...]\n"
                    "       cmpframes align DX DY COREDIR REFDIR\n");
    return 1;
}
