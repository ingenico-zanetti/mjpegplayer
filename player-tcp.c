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

#define SCALE_DENOM 4   // 1,2,4,8
#define MAX_QUEUE 8

// ---------------- QUEUE ----------------

typedef struct {
    void *items[MAX_QUEUE];
    int head, tail, count;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} queue_t;

void queue_init(queue_t *q) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mutex, NULL);
    pthread_cond_init(&q->cond, NULL);
}

void queue_push(queue_t *q, void *item) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == MAX_QUEUE)
        pthread_cond_wait(&q->cond, &q->mutex);

    q->items[q->tail] = item;
    q->tail = (q->tail + 1) % MAX_QUEUE;
    q->count++;

    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

void* queue_pop(queue_t *q) {
    pthread_mutex_lock(&q->mutex);
    while (q->count == 0)
        pthread_cond_wait(&q->cond, &q->mutex);

    void *item = q->items[q->head];
    q->head = (q->head + 1) % MAX_QUEUE;
    q->count--;

    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);

    return item;
}

// ---------------- FRAMES ----------------

typedef struct {
    unsigned char *data;
    unsigned long size;
} jpeg_frame_t;

typedef struct {
    unsigned char *data;
    int w;
    int h;
} raw_frame_t;

// ---------------- DRM ----------------

typedef struct {
    int fd;
    uint32_t conn_id;
    uint32_t crtc_id;
    drmModeModeInfo mode;

    struct {
        uint32_t fb_id;
        uint32_t handle;
        uint8_t *map;
        uint32_t pitch;
    } buf[2];

    int front, back;
} drm_dev_t;

int drm_init(drm_dev_t *dev) {
    drmModeRes *res = drmModeGetResources(dev->fd);
    drmModeConnector *conn = NULL;

    for (int i = 0; i < res->count_connectors; i++) {
        conn = drmModeGetConnector(dev->fd, res->connectors[i]);
        if (conn->connection == DRM_MODE_CONNECTED) break;
    }

    dev->conn_id = conn->connector_id;
    dev->mode = conn->modes[0];

    drmModeEncoder *enc = drmModeGetEncoder(dev->fd, conn->encoder_id);
    dev->crtc_id = enc->crtc_id;

    drmModeFreeEncoder(enc);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    for (int i = 0; i < 2; i++) {
        struct drm_mode_create_dumb creq = {0};
        creq.width  = dev->mode.hdisplay;
        creq.height = dev->mode.vdisplay;
        creq.bpp    = 32;

        ioctl(dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);

        drmModeAddFB(dev->fd, creq.width, creq.height,
                     24, 32, creq.pitch, creq.handle,
                     &dev->buf[i].fb_id);

        struct drm_mode_map_dumb mreq = {0};
        mreq.handle = creq.handle;
        ioctl(dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);

        dev->buf[i].map = mmap(NULL, creq.size,
                               PROT_READ | PROT_WRITE,
                               MAP_SHARED,
                               dev->fd, mreq.offset);

        dev->buf[i].pitch = creq.pitch;
        dev->buf[i].handle = creq.handle;
    }

    dev->front = 0;
    dev->back  = 1;

    drmModeSetCrtc(dev->fd, dev->crtc_id,
                   dev->buf[0].fb_id,
                   0, 0, &dev->conn_id, 1, &dev->mode);

    return 0;
}

void drm_flip(drm_dev_t *dev) {
    drmModePageFlip(dev->fd,
                    dev->crtc_id,
                    dev->buf[dev->back].fb_id,
                    DRM_MODE_PAGE_FLIP_EVENT,
                    NULL);

    int t = dev->front;
    dev->front = dev->back;
    dev->back = t;
}

// ---------------- TCP ----------------

int tcp_connect(const char *host, int port) {
    struct sockaddr_in addr;
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    if (sock < 0) return -1;

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
    if (strncmp(url, "tcp://", 6) != 0)
        return -1;

    const char *p = url + 6;
    const char *colon = strchr(p, ':');
    if (!colon) return -1;

    strncpy(host, p, colon - p);
    host[colon - p] = 0;

    *port = atoi(colon + 1);

    return 0;
}

// ---------------- MJPEG SOCKET ----------------

int read_jpeg_socket(int sock, unsigned char **buf, unsigned long *size) {
    static unsigned char tmp[2*1024*1024];
    int idx = 0, started = 0;
    unsigned char c;

    while (recv(sock, &c, 1, 0) > 0) {
        if (!started) {
            if (c == 0xFF) {
                unsigned char next;
                if (recv(sock, &next, 1, 0) <= 0) break;
                if (next == 0xD8) {
                    tmp[0] = 0xFF;
                    tmp[1] = 0xD8;
                    idx = 2;
                    started = 1;
                }
            }
        } else {
            tmp[idx++] = c;

            if (idx > 2 &&
                tmp[idx-2] == 0xFF &&
                tmp[idx-1] == 0xD9) {

                *buf = malloc(idx);
                memcpy(*buf, tmp, idx);
                *size = idx;
                return 1;
            }
        }

        if (idx >= sizeof(tmp)) {
            idx = 0;
            started = 0;
        }
    }

    return 0;
}

// ---------------- THREADS ----------------

queue_t jpeg_q;
queue_t raw_q;

typedef struct {
    int sock;
} reader_ctx_t;

// reader
void* reader_thread(void *arg) {
    reader_ctx_t *ctx = arg;

    while (1) {
        jpeg_frame_t *f = malloc(sizeof(*f));

        if (!read_jpeg_socket(ctx->sock, &f->data, &f->size)) {
            free(f);
            break;
        }

        queue_push(&jpeg_q, f);
    }
    return NULL;
}

// decoder
void* decoder_thread(void *arg) {
    while (1) {
        jpeg_frame_t *jf = queue_pop(&jpeg_q);

        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_decompress(&cinfo);

        jpeg_mem_src(&cinfo, jf->data, jf->size);
        jpeg_read_header(&cinfo, TRUE);

        cinfo.scale_num = 1;
        cinfo.scale_denom = SCALE_DENOM;

        jpeg_start_decompress(&cinfo);

        int w = cinfo.output_width;
        int h = cinfo.output_height;
        int ch = cinfo.output_components;

        raw_frame_t *rf = malloc(sizeof(*rf));
        rf->w = w;
        rf->h = h;
        rf->data = malloc(w * h * 4);

        JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)
            ((j_common_ptr)&cinfo, JPOOL_IMAGE, w * ch, 1);

        unsigned char *dst = rf->data;

        while (cinfo.output_scanline < h) {
            jpeg_read_scanlines(&cinfo, buffer, 1);

            for (int x = 0; x < w; x++) {
                unsigned char r = buffer[0][x*ch+0];
                unsigned char g = buffer[0][x*ch+1];
                unsigned char b = buffer[0][x*ch+2];

                dst[x*4+0] = b;
                dst[x*4+1] = g;
                dst[x*4+2] = r;
                dst[x*4+3] = 255;
            }
            dst += w * 4;
        }

        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);

        free(jf->data);
        free(jf);

        queue_push(&raw_q, rf);
    }
    return NULL;
}

// display
void* display_thread(void *arg) {
    drm_dev_t *drm = arg;

    while (1) {
        raw_frame_t *f = queue_pop(&raw_q);

        uint8_t *dst = drm->buf[drm->back].map;

        int minw = f->w < drm->mode.hdisplay ? f->w : drm->mode.hdisplay;
        int minh = f->h < drm->mode.vdisplay ? f->h : drm->mode.vdisplay;

        for (int y = 0; y < minh; y++) {
            memcpy(dst + y * drm->buf[drm->back].pitch,
                   f->data + y * f->w * 4,
                   minw * 4);
        }

        drm_flip(drm);

        free(f->data);
        free(f);
    }
    return NULL;
}

// ---------------- MAIN ----------------

int main(int argc, char **argv) {

    if (argc < 2) {
        printf("Usage: %s tcp://host:port\n", argv[0]);
        return -1;
    }

    char host[256];
    int port;

    if (parse_tcp_url(argv[1], host, &port) < 0) {
        printf("Invalid URL\n");
        return -1;
    }

    int sock = tcp_connect(host, port);
    if (sock < 0) {
        perror("connect");
        return -1;
    }

    drm_dev_t drm = {0};
    drm.fd = open("/dev/dri/card1", O_RDWR);
    drm_init(&drm);

    queue_init(&jpeg_q);
    queue_init(&raw_q);

    pthread_t t1, t2, t3;

    reader_ctx_t ctx = { .sock = sock };

    pthread_create(&t1, NULL, reader_thread, &ctx);
    pthread_create(&t2, NULL, decoder_thread, NULL);
    pthread_create(&t3, NULL, display_thread, &drm);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    pthread_join(t3, NULL);

    close(sock);

    return 0;
}

