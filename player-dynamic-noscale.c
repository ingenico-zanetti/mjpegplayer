#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <jpeglib.h>

// ---------------- CONFIG ----------------

#define MAX_STREAMS 16
#define SCALE_DENOM 4

// ---------------- STRUCTS ----------------

typedef struct {
    int sock;
    unsigned char *tmp;
    int tmp_idx;
    int tmp_started;
    int index;
} stream_t;

typedef struct {
    int fd;
    uint32_t conn_id, crtc_id;
    drmModeModeInfo mode;
    uint8_t *map;
    uint32_t pitch;
} drm_dev_t;

// ---------------- TCP ----------------

int tcp_connect(const char *host, int port) {
    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        struct hostent *he = gethostbyname(host);
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        return -1;

    return sock;
}

int parse_tcp_url(const char *url, char *host, int *port) {
    if (strncmp(url, "tcp://", 6) != 0) return -1;

    const char *p = url + 6;
    const char *c = strchr(p, ':');

    strncpy(host, p, c - p);
    host[c - p] = 0;
    *port = atoi(c + 1);

    return 0;
}

// ---------------- MJPEG ----------------

int read_jpeg(stream_t *s, unsigned char **buf, unsigned long *size) {

    unsigned char c;

    while (recv(s->sock, &c, 1, 0) > 0) {

        if (!s->tmp_started) {
            if (c == 0xFF) {
                unsigned char next;
                if (recv(s->sock, &next, 1, 0) <= 0) break;

                if (next == 0xD8) {
                    s->tmp[0] = 0xFF;
                    s->tmp[1] = 0xD8;
                    s->tmp_idx = 2;
                    s->tmp_started = 1;
                }
            }
        } else {
            s->tmp[s->tmp_idx++] = c;

            if (s->tmp_idx > 2 &&
                s->tmp[s->tmp_idx-2] == 0xFF &&
                s->tmp[s->tmp_idx-1] == 0xD9) {

                *buf = s->tmp;
                *size = s->tmp_idx;

                s->tmp_idx = 0;
                s->tmp_started = 0;

                return 1;
            }
        }

        if (s->tmp_idx >= 1024*1024) {
            s->tmp_idx = 0;
            s->tmp_started = 0;
        }
    }

    return 0;
}

// ---------------- DRM ----------------

int drm_init(drm_dev_t *dev) {

    drmModeRes *res = drmModeGetResources(dev->fd);
    drmModeConnector *conn = drmModeGetConnector(dev->fd, res->connectors[0]);

    dev->conn_id = conn->connector_id;
    dev->mode = conn->modes[0];

    drmModeEncoder *enc = drmModeGetEncoder(dev->fd, conn->encoder_id);
    dev->crtc_id = enc->crtc_id;

    struct drm_mode_create_dumb creq = {0};
    creq.width = dev->mode.hdisplay;
    creq.height = dev->mode.vdisplay;
    creq.bpp = 32;

    ioctl(dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);

    uint32_t fb;
    drmModeAddFB(dev->fd, creq.width, creq.height,
        24, 32, creq.pitch, creq.handle, &fb);

    struct drm_mode_map_dumb mreq = {0};
    mreq.handle = creq.handle;
    ioctl(dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);

    dev->map = mmap(NULL, creq.size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        dev->fd, mreq.offset);

    dev->pitch = creq.pitch;

    drmModeSetCrtc(dev->fd, dev->crtc_id,
                   fb, 0, 0,
                   &dev->conn_id, 1, &dev->mode);

    return 0;
}

// ---------------- LAYOUT ----------------

void compute_layout(int total, int index,
                    drm_dev_t *drm,
                    int *ox, int *oy,
                    int *w, int *h)
{
    int W = drm->mode.hdisplay;
    int H = drm->mode.vdisplay;

    int hw = W/2, hh = H/2;

    // 1 flux
    if (total == 1) {
        *ox = 0; *oy = 0; *w = W; *h = H;
        return;
    }

    // 2-4
    if (total <= 4) {
        *ox = (index % 2) * hw;
        *oy = (index / 2) * hh;
        *w = hw;
        *h = hh;
        return;
    }

    // 5-7
    if (total <= 7) {
        if (index < 3) {
            *ox = (index % 2) * hw;
            *oy = (index == 2) ? hh : 0;
            *w = hw;
            *h = hh;
            return;
        }
        // subdiv last quadrant
        int sub = index - 3;
        int qx = 1, qy = 1;
        int sw = hw/2, sh = hh/2;

        *ox = qx*hw + (sub % 2)*sw;
        *oy = qy*hh + (sub / 2)*sh;
        *w = sw;
        *h = sh;
        return;
    }

    // 8-10
    if (total <= 10) {
        if (index < 2) {
            *ox = index * hw;
            *oy = 0;
            *w = hw;
            *h = hh;
            return;
        }

        int sub = index - 2;
        int quad = (sub < 4) ? 2 : 3;
        sub %= 4;

        int qx = quad % 2;
        int qy = quad / 2;

        int sw = hw/2, sh = hh/2;

        *ox = qx*hw + (sub%2)*sw;
        *oy = qy*hh + (sub/2)*sh;
        *w = sw;
        *h = sh;
        return;
    }

    // 11-13
    if (total <= 13) {
        if (index == 0) {
            *ox = 0; *oy = 0;
            *w = hw; *h = hh;
            return;
        }

        int sub = index - 1;
        int quad = 1 + sub / 4;

        sub %= 4;

        int qx = quad % 2;
        int qy = quad / 2;

        int sw = hw/2, sh = hh/2;

        *ox = qx*hw + (sub%2)*sw;
        *oy = qy*hh + (sub/2)*sh;
        *w = sw;
        *h = sh;
        return;
    }

    // >=14 -> 4x4
    int cw = W/4, ch = H/4;

    *ox = (index % 4)*cw;
    *oy = (index / 4)*ch;
    *w = cw;
    *h = ch;
}

// ---------------- DECODE ----------------

void decode(stream_t *s, drm_dev_t *drm,
            unsigned char *jpeg, unsigned long size,
            int total)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, jpeg, size);
    jpeg_read_header(&cinfo, TRUE);

    cinfo.scale_num = 1;
    cinfo.scale_denom = SCALE_DENOM;

    jpeg_start_decompress(&cinfo);

    int w = cinfo.output_width;
    int h = cinfo.output_height;
    int ch = cinfo.output_components;

    int ox, oy, cw, chh;
    compute_layout(total, s->index, drm, &ox, &oy, &cw, &chh);

    JSAMPARRAY buffer =
        (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
        JPOOL_IMAGE, w * ch, 1);

    int y = 0;

    while (cinfo.output_scanline < h && y < chh) {

        jpeg_read_scanlines(&cinfo, buffer, 1);

        uint8_t *dst = drm->map +
            (oy + y) * drm->pitch +
            ox * 4;

        for (int x = 0; x < w && x < cw; x++) {
            dst[x*4+0] = buffer[0][x*ch+2];
            dst[x*4+1] = buffer[0][x*ch+1];
            dst[x*4+2] = buffer[0][x*ch+0];
            dst[x*4+3] = 255;
        }

        y++;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
}

// ---------------- THREAD ----------------

typedef struct {
    stream_t *s;
    drm_dev_t *drm;
    int total;
} ctx_t;

void* thread(void *arg) {

    ctx_t *c = arg;

    while (1) {
        unsigned char *buf;
        unsigned long size;

        if (!read_jpeg(c->s, &buf, &size))
            break;

        decode(c->s, c->drm, buf, size, c->total);
    }

    return NULL;
}

// ---------------- MAIN ----------------

int main(int argc, char **argv) {

    if (argc < 2 || argc > 17) {
        printf("Usage: %s tcp://... (max16)\n", argv[0]);
        return -1;
    }

    int n = argc - 1;

    stream_t *streams = calloc(n, sizeof(stream_t));

    drm_dev_t drm = {0};
    drm.fd = open("/dev/dri/card1", O_RDWR);
    drm_init(&drm);

    pthread_t th[MAX_STREAMS];
    ctx_t ctx[MAX_STREAMS];

    for (int i = 0; i < n; i++) {

        char host[128];
        int port;

        parse_tcp_url(argv[i+1], host, &port);

        streams[i].sock = tcp_connect(host, port);
        streams[i].tmp = malloc(1024*1024);
        streams[i].index = i;

        ctx[i].s = &streams[i];
        ctx[i].drm = &drm;
        ctx[i].total = n;

        pthread_create(&th[i], NULL, thread, &ctx[i]);
    }

    for (int i = 0; i < n; i++)
        pthread_join(th[i], NULL);

    return 0;
}

