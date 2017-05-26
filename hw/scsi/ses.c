/*
 * SCSI Enclosure Service emulation
 *
 */

//#define DEBUG_SCSI

#ifdef DEBUG_SCSI
#define DPRINTF(fmt, ...) \
do { printf("scsi-enclosure: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/scsi/scsi.h"
#include "block/scsi.h"
#include "sysemu/sysemu.h"
#include "sysemu/block-backend.h"
#include "sysemu/blockdev.h"
#include "hw/block/block.h"
#include "sysemu/dma.h"
#include "qemu/cutils.h"

#ifdef __linux
#include <scsi/sg.h>
#endif

#define SCSI_MAX_INQUIRY_LEN        256
#define SCSI_MAX_MODE_LEN           256

typedef struct SCSISESState SCSISESState;

typedef struct SCSISESReq {
    SCSIRequest req;
    /* Both sector and sector_count are in terms of qemu 512 byte blocks.  */
    uint32_t buflen;
    struct iovec iov;
    QEMUIOVector qiov;
} SCSISESReq;

struct SCSISESState
{
    SCSIDevice qdev;
    uint32_t features;
    uint16_t port_index;
    QEMUBH *bh;
    char *version;
    char *serial;
    char *vendor;
    char *product;
};

static void scsi_free_request(SCSIRequest *req)
{
}

/* Helper function for command completion with sense.  */
static void scsi_check_condition(SCSISESReq *r, SCSISense sense)
{
    DPRINTF("Command complete tag=0x%x sense=%d/%d/%d\n",
            r->req.tag, sense.key, sense.asc, sense.ascq);
    scsi_req_build_sense(&r->req, sense);
    scsi_req_complete(&r->req, CHECK_CONDITION);
}

static int scsi_disk_emulate_inquiry(SCSIRequest *req, uint8_t *outbuf)
{
    return 0;
}

static int scsi_disk_emulate_mode_sense(SCSISESReq *r, uint8_t *outbuf)
{
    return 0;
}

#if 0
static void scsi_ses_emulate_read_data(SCSIRequest *req)
{
    SCSISESReq *r = DO_UPCAST(SCSISESReq, req, req);

    /* This also clears the sense buffer for REQUEST SENSE.  */
    scsi_req_complete(&r->req, GOOD);
}
#endif

static void scsi_disk_emulate_mode_select(SCSISESReq *r, uint8_t *inbuf)
{
}

static void scsi_ses_emulate_write_data(SCSIRequest *req)
{
    SCSISESReq *r = DO_UPCAST(SCSISESReq, req, req);

    if (r->iov.iov_len) {
        int buflen = r->iov.iov_len;
        DPRINTF("Write buf_len=%d\n", buflen);
        r->iov.iov_len = 0;
        scsi_req_data(&r->req, buflen);
        return;
    }

    switch (req->cmd.buf[0]) {
    case MODE_SELECT:
    case MODE_SELECT_10:
        /* This also clears the sense buffer for REQUEST SENSE.  */
        scsi_disk_emulate_mode_select(r, r->iov.iov_base);
        break;

    default:
        abort();
    }
}

static int32_t scsi_ses_emulate_command(SCSIRequest *req, uint8_t *buf)
{
    SCSISESReq *r = DO_UPCAST(SCSISESReq, req, req);
    SCSISESState *s = DO_UPCAST(SCSISESState, qdev, req->dev);
    uint8_t *outbuf = NULL;
    int buflen;

    switch (req->cmd.buf[0]) {
    case INQUIRY:
    case MODE_SENSE:
    case MODE_SENSE_10:
    case REQUEST_SENSE:
    case RECEIVE_DIAGNOSTIC:
    case SEND_DIAGNOSTIC:
        break;

    default:
        break;
    }

    if (req->cmd.xfer > 65536) {
        goto illegal_request;
    }

    r->buflen = MAX(4096, req->cmd.xfer);

    if (!r->iov.iov_base) {
        r->iov.iov_base = blk_blockalign(s->qdev.conf.blk, r->buflen);
        buflen = req->cmd.xfer;
        outbuf = r->iov.iov_base;
        memset(outbuf, 0, r->buflen);
    }

    switch (req->cmd.buf[0]) {
    case TEST_UNIT_READY:
        assert(blk_is_available(s->qdev.conf.blk));
        break;
    case INQUIRY:
        buflen = scsi_disk_emulate_inquiry(req, outbuf);
        if (buflen < 0) {
            goto illegal_request;
        }
        break;
    case MODE_SENSE:
    case MODE_SENSE_10:
        buflen = scsi_disk_emulate_mode_sense(r, outbuf);
        if (buflen < 0) {
            goto illegal_request;
        }
        break;
    case REQUEST_SENSE:
        /* Just return "NO SENSE".  */
        buflen = scsi_build_sense(NULL, 0, outbuf, r->buflen,
                                  (req->cmd.buf[1] & 1) == 0);
        if (buflen < 0) {
            goto illegal_request;
        }
        break;
    case MODE_SELECT:
        DPRINTF("Mode Select(6) (len %lu)\n", (long)r->req.cmd.xfer);
        break;
    case MODE_SELECT_10:
        DPRINTF("Mode Select(10) (len %lu)\n", (long)r->req.cmd.xfer);
        break;
    case RECEIVE_DIAGNOSTIC:
        break;
    case SEND_DIAGNOSTIC:
        break;
    default:
        DPRINTF("Unknown SCSI command (%2.2x=%s)\n", buf[0],
                scsi_command_name(buf[0]));
        scsi_check_condition(r, SENSE_CODE(INVALID_OPCODE));
        return 0;
    }
    assert(!r->req.aiocb);
    r->iov.iov_len = MIN(r->buflen, req->cmd.xfer);
    if (r->iov.iov_len == 0) {
        scsi_req_complete(&r->req, GOOD);
    }
    if (r->req.cmd.mode == SCSI_XFER_TO_DEV) {
        assert(r->iov.iov_len == req->cmd.xfer);
        return -r->iov.iov_len;
    } else {
        return r->iov.iov_len;
    }

illegal_request:
    if (r->req.status == -1) {
        scsi_check_condition(r, SENSE_CODE(INVALID_FIELD));
    }
    return 0;
}

static void scsi_ses_reset(DeviceState *dev)
{
}

static void scsi_ses_realize(SCSIDevice *dev, Error **errp)
{
    SCSISESState *s = DO_UPCAST(SCSISESState, qdev, dev);

    s->qdev.type = TYPE_ENCLOSURE;
    if (!s->product) {
        s->product = g_strdup("Virtual SCSI Enclosure Device");
    }

    if (!s->version) {
        s->version = g_strdup(qemu_hw_version());
    }

    if (!s->vendor)  {
        s->vendor = g_strdup("QEMU");
    }
}

static const SCSIReqOps scsi_ses_emulate_reqops = {
    .size         = sizeof(SCSISESReq),
    .free_req     = scsi_free_request,
    .send_command = scsi_ses_emulate_command,
    //.read_data    = scsi_ses_emulate_read_data,
    .write_data   = scsi_ses_emulate_write_data,
    //.get_buf      = scsi_get_buf,
};

static const SCSIReqOps *const scsi_disk_reqops_dispatch[256] = {
    [TEST_UNIT_READY]                 = &scsi_ses_emulate_reqops,
    [INQUIRY]                         = &scsi_ses_emulate_reqops,
    [MODE_SENSE]                      = &scsi_ses_emulate_reqops,
    [MODE_SENSE_10]                   = &scsi_ses_emulate_reqops,
    [MODE_SELECT]                     = &scsi_ses_emulate_reqops,
    [MODE_SELECT_10]                  = &scsi_ses_emulate_reqops,
    [RECEIVE_DIAGNOSTIC]              = &scsi_ses_emulate_reqops,
    [SEND_DIAGNOSTIC]                 = &scsi_ses_emulate_reqops,
};

static SCSIRequest *scsi_new_request(SCSIDevice *d, uint32_t tag, uint32_t lun,
                                     uint8_t *buf, void *hba_private)
{
    SCSISESState *s = DO_UPCAST(SCSISESState, qdev, d);
    SCSIRequest *req;
    const SCSIReqOps *ops;
    uint8_t command;

    command = buf[0];
    ops = scsi_disk_reqops_dispatch[command];
    if (!ops) {
        ops = &scsi_ses_emulate_reqops;
    }
    req = scsi_req_alloc(ops, &s->qdev, tag, lun, hba_private);

#ifdef DEBUG_SCSI
    DPRINTF("Command: lun=%d tag=0x%x data=0x%02x", lun, tag, buf[0]);
    {
        int i;
        for (i = 1; i < scsi_cdb_length(buf); i++) {
            printf(" 0x%02x", buf[i]);
        }
        printf("\n");
    }
#endif

    return req;
}

#define DEFINE_SCSI_SES_PROPERTIES()                                \
    DEFINE_PROP_STRING("ver", SCSISESState, version),               \
    DEFINE_PROP_STRING("serial", SCSISESState, serial),             \
    DEFINE_PROP_STRING("vendor", SCSISESState, vendor),             \
    DEFINE_PROP_STRING("product", SCSISESState, product)

static Property scsi_ses_properties[] = {
    DEFINE_SCSI_SES_PROPERTIES(),
    DEFINE_PROP_UINT64("wwn", SCSISESState, qdev.wwn, 0),
    DEFINE_PROP_UINT64("port_wwn", SCSISESState, qdev.port_wwn, 0),
    DEFINE_PROP_UINT16("port_index", SCSISESState, port_index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void scsi_ses_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SCSIDeviceClass *sc = SCSI_DEVICE_CLASS(klass);

    sc->realize      = scsi_ses_realize;
    sc->alloc_req    = scsi_new_request;
    dc->fw_name = "ses";
    dc->desc = "Virtual SCSI Enclosure Service";
    dc->reset = scsi_ses_reset;
    dc->props = scsi_ses_properties;
}

static const TypeInfo scsi_ses_info = {
    .name          = "ses",
    .parent        = TYPE_SCSI_DEVICE,
    .instance_size = sizeof(SCSISESState),
    .class_init    = scsi_ses_class_initfn,
};

static void scsi_enclosure_register_types(void)
{
    type_register_static(&scsi_ses_info);
}

type_init(scsi_enclosure_register_types)
