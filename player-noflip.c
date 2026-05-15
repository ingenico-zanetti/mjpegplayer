#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

#include <turbojpeg.h>

// ---------------- CONFIG ----------------

#define MAX_QUEUE 8

// ---------------- FRAME STRUCT ----------------

typedef struct {
    unsigned char *data;
    unsigned long size;
} jpeg_frame_t;

typedef struct {
    unsigned char *data;
    int width;
    int height;
} raw_frame_t;

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

// init + double buffer
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
        creq.width = dev->mode.hdisplay;
        creq.height = dev->mode.vdisplay;
        creq.bpp = 32;

        ioctl(dev->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq);

        drmModeAddFB(dev->fd, creq.width, creq.height,
                     24, 32, creq.pitch, creq.handle, &dev->buf[i].fb_id);

        struct drm_mode_map_dumb mreq = {0};
        mreq.handle = creq.handle;
        ioctl(dev->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq);

        dev->buf[i].map = mmap(0, creq.size,
                               PROT_READ | PROT_WRITE, MAP_SHARED,
                               dev->fd, mreq.offset);

        dev->buf[i].pitch = creq.pitch;
        dev->buf[i].handle = creq.handle;
    }

    dev->front = 0;
    dev->back = 1;

    drmModeSetCrtc(dev->fd, dev->crtc_id,
                   dev->buf[0].fb_id,
                   0, 0, &dev->conn_id, 1, &dev->mode);

    return 0;
}

// flip
void drm_flip(drm_dev_t *dev) {
    drmModePageFlip(dev->fd,
                    dev->crtc_id,
                    dev->buf[dev->back].fb_id,
                    DRM_MODE_PAGE_FLIP_EVENT,
                    NULL);

    // swap buffers
    int tmp = dev->front;
    dev->front = dev->back;
    dev->back = tmp;
}

// ---------------- MJPEG PARSER ----------------

int read_jpeg(FILE *fp, unsigned char **buf, unsigned long *size) {
    static unsigned char tmp[2*1024*1024];
    int c, idx = 0, start = 0;

    while ((c = fgetc(fp)) != EOF) {
        if (!start) {
            if (c == 0xFF && fgetc(fp) == 0xD8) {
                tmp[0] = 0xFF;
                tmp[1] = 0xD8;
                idx = 2;
                start = 1;
            }
        } else {
            tmp[idx++] = c;
            if (tmp[idx-2] == 0xFF && tmp[idx-1] == 0xD9) {
                *buf = malloc(idx);
                memcpy(*buf, tmp, idx);
                *size = idx;
                return 1;
            }
        }
    }
    return 0;
}

// ---------------- THREADS ----------------

queue_t jpeg_q;
queue_t raw_q;

// producer
void* reader_thread(void *arg) {
    FILE *fp = stdin;

    while (1) {
        jpeg_frame_t *f = malloc(sizeof(*f));

        if (!read_jpeg(fp, &f->data, &f->size)) break;

        queue_push(&jpeg_q, f);
    }
    return NULL;
}

// decoder
void* decoder_thread(void *arg) {
    tjhandle tj = tjInitDecompress();
#if 0
    int numScalingFactors = -1;
    tjscalingfactor *scalingFactors = tj3GetScalingFactors(&numScalingFactors);

    int i = 0;
    while(i < numScalingFactors){
	    fprintf(stderr, "Available scaling factor %d from %d is num=%d/denom=%d" "\n", i, numScalingFactors, scalingFactors[i].num, scalingFactors[i].denom);
	    i++;
    }
#endif
    while (1) {
        jpeg_frame_t *jf = queue_pop(&jpeg_q);

        int w, h, subsamp, cs;
        tjDecompressHeader3(tj, jf->data, jf->size, &w, &h, &subsamp, &cs);

        // scaling
        tjscalingfactor sf = { 1, 2};
        int sw = TJSCALED(w, sf);
        int sh = TJSCALED(h, sf);

        raw_frame_t *rf = malloc(sizeof(*rf));
        rf->width = sw;
        rf->height = sh;
        rf->data = malloc(sw * sh * 4);

        tjDecompress2(tj, jf->data, jf->size,
                      rf->data, sw, 0, sh,
                      TJPF_BGRA,
                      TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);

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

        int minw = f->width < drm->mode.hdisplay ? f->width : drm->mode.hdisplay;
        int minh = f->height < drm->mode.vdisplay ? f->height : drm->mode.vdisplay;

        for (int y = 0; y < minh; y++) {
            memcpy(dst + y * drm->buf[drm->back].pitch,
                   f->data + y * f->width * 4,
                   minw * 4);
        }

        drm_flip(drm);

        free(f->data);
        free(f);
    }
    return NULL;
}

// ---------------- MAIN ----------------

int main() {
    drm_dev_t drm = {0};
    drm.fd = open("/dev/dri/card1", O_RDWR);

    drm_init(&drm);

    queue_init(&jpeg_q);
    queue_init(&raw_q);

    pthread_t t1, t2, t3;

    pthread_create(&t1, NULL, reader_thread, NULL);
    pthread_create(&t2, NULL, decoder_thread, NULL);
    pthread_create(&t3, NULL, display_thread, &drm);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    pthread_join(t3, NULL);

    return 0;
}

