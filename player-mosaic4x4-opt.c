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

#define MAX_STREAMS 16
#define GRID_SIZE 4
#define SCALE_DENOM 4

// ---------------- STREAM ----------------

typedef struct {
    int sock;

    unsigned char *tmp;
    int tmp_idx;
    int tmp_started;

    int index;

} stream_t;

// ---------------- DRM ----------------

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
        if (!he) return -1;
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
    if (!c) return -1;

    strncpy(host, p, c - p);
    host[c - p] = 0;
    *port = atoi(c + 1);

    return 0;
}

// ---------------- MJPEG PARSER ----------------

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

// ---------------- DRM INIT ----------------

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

    uint32_t fb_id;
    drmModeAddFB(dev->fd, creq.width, creq.height,
        24, 32, creq.pitch, creq.handle, &fb_id);

    struct drm_mode_map_dumb mreq = {0};
    mreq.handle = creq.handle;
    ioctl(dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);

    dev->map = mmap(NULL, creq.size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        dev->fd, mreq.offset);

    dev->pitch = creq.pitch;

    drmModeSetCrtc(dev->fd, dev->crtc_id,
                   fb_id,
                   0, 0, &dev->conn_id, 1, &dev->mode);

    return 0;
}

// ---------------- DECODE DIRECT ----------------

void decode_to_framebuffer(stream_t *s, drm_dev_t *drm,
                           unsigned char *jpegBuf, unsigned long jpegSize)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, jpegBuf, jpegSize);
    jpeg_read_header(&cinfo, TRUE);

    cinfo.scale_num = 1;
    cinfo.scale_denom = SCALE_DENOM;

    jpeg_start_decompress(&cinfo);

    int w = cinfo.output_width;
    int h = cinfo.output_height;
    int ch = cinfo.output_components;

    int cell_w = drm->mode.hdisplay / GRID_SIZE;
    int cell_h = drm->mode.vdisplay / GRID_SIZE;

    int ox = (s->index % GRID_SIZE) * cell_w;
    int oy = (s->index / GRID_SIZE) * cell_h;

    JSAMPARRAY buffer =
        (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo,
        JPOOL_IMAGE, w * ch, 1);

    int y = 0;

    while (cinfo.output_scanline < h && y < cell_h) {

        jpeg_read_scanlines(&cinfo, buffer, 1);

        uint8_t *dst = drm->map +
            (oy + y) * drm->pitch +
            ox * 4;

        for (int x = 0; x < w && x < cell_w; x++) {

            dst[x*4+0] = buffer[0][x*ch+2]; // B
            dst[x*4+1] = buffer[0][x*ch+1]; // G
            dst[x*4+2] = buffer[0][x*ch+0]; // R
            dst[x*4+3] = 255;
        }

        y++;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
}

// ---------------- THREAD ----------------

typedef struct {
    stream_t *stream;
    drm_dev_t *drm;
} thread_ctx_t;

void* stream_thread(void *arg) {

    thread_ctx_t *ctx = arg;
    stream_t *s = ctx->stream;

    while (1) {

        unsigned char *buf;
        unsigned long size;

        if (!read_jpeg(s, &buf, &size))
            break;

        decode_to_framebuffer(s, ctx->drm, buf, size);
    }

    return NULL;
}

// ---------------- MAIN ----------------

int main(int argc, char **argv) {

    if (argc < 2 || argc > 17) {
        printf("Usage: %s tcp://h:p [...]\n", argv[0]);
        return -1;
    }

    int n = argc - 1;

    stream_t *streams = calloc(n, sizeof(stream_t));

    drm_dev_t drm = {0};
    drm.fd = open("/dev/dri/card1", O_RDWR);
    drm_init(&drm);

    pthread_t threads[MAX_STREAMS];
    thread_ctx_t ctx[MAX_STREAMS];

    for (int i = 0; i < n; i++) {

        char host[128];
        int port;

        parse_tcp_url(argv[i+1], host, &port);

        streams[i].sock = tcp_connect(host, port);
        streams[i].tmp = malloc(1024*1024);
        streams[i].index = i;

        ctx[i].stream = &streams[i];
        ctx[i].drm = &drm;

        pthread_create(&threads[i], NULL, stream_thread, &ctx[i]);
    }

    for (int i = 0; i < n; i++)
        pthread_join(threads[i], NULL);

    return 0;
}

