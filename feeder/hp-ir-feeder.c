/*
 * hp-ir-feeder.c - On-demand IR Camera & WLED2 Bridge for Howdy
 *
 * Connects the physical Intel IPU3 OV7251 NIR sensor (/dev/video2)
 * to the V4L2 loopback device (/dev/video20).
 *
 * Automatically detects when a consumer (Howdy/OpenCV) opens /dev/video20,
 * pulses the TPS68470 WLED2 infrared emitter, streams and unpacks
 * IPU3 raw-10 frames into BGR24 frames, and turns off WLED2 and streaming
 * as soon as the consumer disconnects.
 *
 * Hardware safety:
 * - WLED2 is pulsed with kernel auto-timeout (max 3000 ms).
 * - Immediate hardware shutoff (wled_off) on reader exit or SIGTERM.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/inotify.h>
#include <linux/videodev2.h>

#define SENSOR_DEV      "/dev/video2"
#define LOOPBACK_DEV    "/dev/video20"
#define WLED_PULSE_SYSFS "/sys/bus/i2c/devices/i2c-INT3472:05/wled_pulse"
#define WLED_OFF_SYSFS   "/sys/bus/i2c/devices/i2c-INT3472:05/wled_off"

#define FRAME_WIDTH     640
#define FRAME_HEIGHT    480
#define RAW_STRIDE      832
#define RAW_FRAME_BYTES (RAW_STRIDE * FRAME_HEIGHT)  /* 399360 bytes */
#define OUT_FRAME_BYTES (FRAME_WIDTH * FRAME_HEIGHT * 3) /* 921600 bytes */
#define NUM_BUFFERS     4
#define WLED_PULSE_MS   3000

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_session_active = 0;
static uint8_t g_lut[1024];

static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
    g_session_active = 0;
}

static void init_lut(double gain) {
    for (int i = 0; i < 1024; i++) {
        double v = ((double)i / 4.0) * gain;
        if (v > 255.0) v = 255.0;
        if (v < 0.0) v = 0.0;
        g_lut[i] = (uint8_t)(v + 0.5);
    }
}

static void wled_pulse(int ms) {
    int fd = open(WLED_PULSE_SYSFS, O_WRONLY);
    if (fd >= 0) {
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%d\n", ms);
        if (len > 0) {
            ssize_t w = write(fd, buf, len);
            (void)w;
        }
        close(fd);
    }
}

static void wled_off(void) {
    int fd = open(WLED_OFF_SYSFS, O_WRONLY);
    if (fd >= 0) {
        ssize_t w = write(fd, "1\n", 2);
        (void)w;
        close(fd);
    }
}

/* Fast IPU3 Raw-10 to BGR24 unpacking with LUT */
static void unpack_ipu3_to_bgr(const uint8_t * __restrict raw, uint8_t * __restrict bgr) {
    for (int y = 0; y < FRAME_HEIGHT; y++) {
        const uint8_t *line = raw + y * RAW_STRIDE;
        uint8_t *out = bgr + y * (FRAME_WIDTH * 3);
        
        int x = 0;
        for (int c = 0; c < 26 && x < FRAME_WIDTH; c++) {
            const uint8_t *chunk = line + (c << 5); /* c * 32 */
            for (int p = 0; p < 25 && x < FRAME_WIDTH; p++, x++) {
                int bit_offset = p * 10;
                int byte_idx = bit_offset >> 3;
                int shift = bit_offset & 7;
                
                uint16_t val10 = ((chunk[byte_idx] | ((uint16_t)chunk[byte_idx + 1] << 8)) >> shift) & 0x3ff;
                uint8_t val8 = g_lut[val10];
                
                int out_idx = x * 3;
                out[out_idx + 0] = val8;
                out[out_idx + 1] = val8;
                out[out_idx + 2] = val8;
            }
        }
    }
}

static int open_and_init_sensor(void **buffers, size_t *buf_lens) {
    /* Ensure media link is enabled and sensor controls optimal for NIR */
    int mr = system("media-ctl -d /dev/media0 -l '\"hp-ov7251-ir-test 2-0060\":0 -> \"ipu3-csi2 2\":0[1]' 2>/dev/null");
    (void)mr;
    int cr = system("v4l2-ctl -d /dev/v4l-subdev6 --set-ctrl 'gain=256,exposure=1704' 2>/dev/null");
    (void)cr;

    int fd = open(SENSOR_DEV, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("[hp-ir-feeder] open SENSOR_DEV");
        return -1;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = FRAME_WIDTH;
    fmt.fmt.pix_mp.height = FRAME_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_IPU3_Y10;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("[hp-ir-feeder] SENSOR VIDIOC_S_FMT");
        close(fd);
        return -1;
    }

    struct v4l2_requestbuffers req = {0};
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = NUM_BUFFERS;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("[hp-ir-feeder] SENSOR VIDIOC_REQBUFS");
        close(fd);
        return -1;
    }

    for (int i = 0; i < NUM_BUFFERS; i++) {
        struct v4l2_plane p[1] = {{0}};
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        b.length = 1;
        b.m.planes = p;
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) {
            perror("[hp-ir-feeder] SENSOR VIDIOC_QUERYBUF");
            close(fd);
            return -1;
        }
        buf_lens[i] = p[0].length;
        buffers[i] = mmap(NULL, buf_lens[i], PROT_READ | PROT_WRITE, MAP_SHARED, fd, p[0].m.mem_offset);
        if (buffers[i] == MAP_FAILED) {
            perror("[hp-ir-feeder] SENSOR mmap");
            close(fd);
            return -1;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) < 0) {
            perror("[hp-ir-feeder] SENSOR VIDIOC_QBUF");
            close(fd);
            return -1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("[hp-ir-feeder] SENSOR VIDIOC_STREAMON");
        close(fd);
        return -1;
    }

    return fd;
}

static void close_sensor(int fd, void **buffers, size_t *buf_lens) {
    if (fd < 0) return;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (buffers[i] && buffers[i] != MAP_FAILED) {
            munmap(buffers[i], buf_lens[i]);
            buffers[i] = NULL;
        }
    }
    close(fd);
}

static int open_loopback(void) {
    int fd = open(LOOPBACK_DEV, O_WRONLY);
    if (fd < 0) {
        perror("[hp-ir-feeder] open LOOPBACK_DEV");
        return -1;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = FRAME_WIDTH;
    fmt.fmt.pix.height = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = FRAME_WIDTH * 3;
    fmt.fmt.pix.sizeimage = OUT_FRAME_BYTES;
    fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

    ioctl(fd, VIDIOC_S_FMT, &fmt);

    /* Write 1 initial blank frame to prime the loopback buffer queue */
    uint8_t *blank = calloc(1, OUT_FRAME_BYTES);
    if (blank) {
        ssize_t w = write(fd, blank, OUT_FRAME_BYTES);
        (void)w;
        free(blank);
    }

    return fd;
}

static void drain_inotify(int ifd) {
    if (ifd < 0) return;
    char buf[1024];
    while (read(ifd, buf, sizeof(buf)) > 0) {}
}

/* Streams frames until consumer disconnects, timeout, or signal */
static void run_streaming_session(int inotify_fd, int loopback_fd, double max_seconds) {
    printf("[hp-ir-feeder] Starting capture session (max %.1fs)...\n", max_seconds);
    
    /* Turn on WLED2 */
    wled_pulse(WLED_PULSE_MS);

    void *buffers[NUM_BUFFERS] = {0};
    size_t buf_lens[NUM_BUFFERS] = {0};
    int sfd = open_and_init_sensor(buffers, buf_lens);
    if (sfd < 0) {
        wled_off();
        return;
    }

    uint8_t *bgr_buf = malloc(OUT_FRAME_BYTES);
    if (!bgr_buf) {
        close_sensor(sfd, buffers, buf_lens);
        wled_off();
        return;
    }

    struct timespec start_ts, last_pulse_ts;
    clock_gettime(CLOCK_MONOTONIC, &start_ts);
    last_pulse_ts = start_ts;

    g_session_active = 1;
    int frame_count = 0;

    while (g_running && g_session_active) {
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        double elapsed = (now_ts.tv_sec - start_ts.tv_sec) + (now_ts.tv_nsec - start_ts.tv_nsec) / 1e9;
        if (max_seconds > 0 && elapsed >= max_seconds) {
            printf("[hp-ir-feeder] Session time limit reached (%.2fs).\n", elapsed);
            break;
        }

        /* Refresh WLED2 pulse every 1.5 seconds */
        double since_pulse = (now_ts.tv_sec - last_pulse_ts.tv_sec) + (now_ts.tv_nsec - last_pulse_ts.tv_nsec) / 1e9;
        if (since_pulse >= 1.5) {
            wled_pulse(WLED_PULSE_MS);
            last_pulse_ts = now_ts;
        }

        /* Check for inotify events (reader closed) */
        if (inotify_fd >= 0) {
            struct pollfd ipfd = {.fd = inotify_fd, .events = POLLIN};
            int ir = poll(&ipfd, 1, 0);
            if (ir > 0 && (ipfd.revents & POLLIN)) {
                char ibuf[512];
                ssize_t n = read(inotify_fd, ibuf, sizeof(ibuf));
                ssize_t pos = 0;
                while (pos + sizeof(struct inotify_event) <= (size_t)n) {
                    struct inotify_event *ev = (struct inotify_event *)(ibuf + pos);
                    if (ev->mask & (IN_CLOSE_NOWRITE | IN_CLOSE_WRITE)) {
                        printf("[hp-ir-feeder] Reader closed /dev/video20 (mask=0x%x).\n", ev->mask);
                        g_session_active = 0;
                        break;
                    }
                    pos += sizeof(struct inotify_event) + ev->len;
                }
                if (!g_session_active) break;
            }
        }

        /* Wait for frame from sensor */
        struct pollfd pfd = {.fd = sfd, .events = POLLIN};
        int pr = poll(&pfd, 1, 200);
        if (pr <= 0) {
            if (pr < 0 && errno == EINTR) continue;
            continue;
        }

        struct v4l2_plane p[1] = {{0}};
        struct v4l2_buffer b = {0};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = p;

        if (ioctl(sfd, VIDIOC_DQBUF, &b) < 0) {
            if (errno == EAGAIN) continue;
            perror("[hp-ir-feeder] VIDIOC_DQBUF");
            break;
        }

        /* Unpack frame */
        unpack_ipu3_to_bgr(buffers[b.index], bgr_buf);

        /* Write to loopback */
        ssize_t written = write(loopback_fd, bgr_buf, OUT_FRAME_BYTES);
        if (written < 0) {
            if (errno == EPIPE || errno == EBADF) {
                printf("[hp-ir-feeder] Loopback write broken pipe (reader disconnected).\n");
                ioctl(sfd, VIDIOC_QBUF, &b);
                break;
            }
        }

        /* Requeue buffer */
        if (ioctl(sfd, VIDIOC_QBUF, &b) < 0) {
            perror("[hp-ir-feeder] VIDIOC_QBUF");
            break;
        }

        frame_count++;
    }

    /* Shutdown hardware cleanly */
    wled_off();
    close_sensor(sfd, buffers, buf_lens);
    free(bgr_buf);
    g_session_active = 0;

    drain_inotify(inotify_fd);
    printf("[hp-ir-feeder] Capture session ended cleanly. Frames streamed: %d\n", frame_count);
}

int main(int argc, char **argv) {
    setlinebuf(stdout);
    setlinebuf(stderr);

    bool daemon_mode = true;
    double oneshot_secs = 0.0;
    double gain = 1.25; /* Default soft boost */

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--oneshot") && i + 1 < argc) {
            daemon_mode = false;
            oneshot_secs = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--gain") && i + 1 < argc) {
            gain = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--help")) {
            printf("Usage: hp-ir-feeder [--oneshot <seconds>] [--gain <1.0-4.0>]\n");
            return 0;
        }
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    signal(SIGPIPE, SIG_IGN);

    init_lut(gain);
    printf("[hp-ir-feeder] Initialized (gain=%.2f)\n", gain);

    int lfd = open_loopback();
    if (lfd < 0) {
        fprintf(stderr, "[hp-ir-feeder] Failed to open loopback device %s\n", LOOPBACK_DEV);
        return 1;
    }

    if (!daemon_mode) {
        run_streaming_session(-1, lfd, oneshot_secs > 0.0 ? oneshot_secs : 5.0);
        close(lfd);
        return 0;
    }

    /* Daemon Mode: Watch /dev/video20 with inotify */
    int ifd = inotify_init1(IN_NONBLOCK);
    if (ifd < 0) {
        perror("[hp-ir-feeder] inotify_init1");
        close(lfd);
        return 1;
    }

    int wd = inotify_add_watch(ifd, LOOPBACK_DEV, IN_OPEN | IN_CLOSE_NOWRITE | IN_CLOSE_WRITE);
    if (wd < 0) {
        perror("[hp-ir-feeder] inotify_add_watch " LOOPBACK_DEV);
        close(ifd);
        close(lfd);
        return 1;
    }

    drain_inotify(ifd);
    printf("[hp-ir-feeder] Daemon listening on %s (inotify wd=%d)...\n", LOOPBACK_DEV, wd);

    while (g_running) {
        struct pollfd pfd = {.fd = ifd, .events = POLLIN};
        int ret = poll(&pfd, 1, 1000);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        char buf[512];
        ssize_t n = read(ifd, buf, sizeof(buf));
        if (n <= 0) continue;

        ssize_t pos = 0;
        bool opened = false;
        while (pos + sizeof(struct inotify_event) <= (size_t)n) {
            struct inotify_event *ev = (struct inotify_event *)(buf + pos);
            if (ev->mask & IN_OPEN) {
                opened = true;
            }
            pos += sizeof(struct inotify_event) + ev->len;
        }

        if (opened && g_running) {
            printf("[hp-ir-feeder] IN_OPEN detected on %s!\n", LOOPBACK_DEV);
            /* Max 25.0 seconds per session (Howdy enrollment can take up to 15s) */
            run_streaming_session(ifd, lfd, 25.0);
        }
    }

    wled_off();
    inotify_rm_watch(ifd, wd);
    close(ifd);
    close(lfd);
    printf("[hp-ir-feeder] Daemon terminated cleanly.\n");
    return 0;
}
