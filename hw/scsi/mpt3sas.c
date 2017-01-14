#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "sysemu/dma.h"
#include "sysemu/block-backend.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "qemu/iov.h"
#include "hw/scsi/scsi.h"
#include "block/scsi.h"
#include "mpt3sas.h"

#include "hw/scsi/mpi/mpi2_type.h"
#include "hw/scsi/mpi/mpi2.h"
#include "hw/scsi/mpi/mpi2_cnfg.h"
#include "hw/scsi/mpi/mpi2_init.h"
#include "hw/scsi/mpi/mpi2_ioc.h"

#include "trace/control.h"
#include "qemu/log.h"

#define NAA_LOCALLY_ASSIGNED_ID 0x3ULL
#define IEEE_COMPANY_LOCALLY_ASSIGNED 0x525400

#define TYPE_MPT3SAS3008   "lsisas3008"

#define MPT3SAS_LSI3008_PRODUCT_ID  \
    (MPI25_FW_HEADER_PID_FAMILY_3108_SAS |   \
     MPI2_FW_HEADER_PID_PROD_TARGET_INITIATOR_SCSI |    \
     MPI2_FW_HEADER_PID_TYPE_SAS)

#define MPT3SAS(obj) \
    OBJECT_CHECK(MPT3SASState, (obj), TYPE_MPT3SAS3008)


#define DEBUG_MPT3SAS   1

static uint32_t ioc_reset_sequence[] = {
    MPI2_WRSEQ_1ST_KEY_VALUE,
    MPI2_WRSEQ_2ND_KEY_VALUE,
    MPI2_WRSEQ_3RD_KEY_VALUE,
    MPI2_WRSEQ_4TH_KEY_VALUE,
    MPI2_WRSEQ_5TH_KEY_VALUE,
    MPI2_WRSEQ_6TH_KEY_VALUE};

static const int mpi2_request_sizes[] = {
    [MPI2_FUNCTION_SCSI_IO_REQUEST]     = sizeof(Mpi2SCSIIORequest_t),
    [MPI2_FUNCTION_SCSI_TASK_MGMT]      = sizeof(Mpi2IOCFactsRequest_t),
    [MPI2_FUNCTION_IOC_INIT]            = sizeof(Mpi2IOCInitRequest_t),
    [MPI2_FUNCTION_CONFIG]              = sizeof(Mpi2ConfigRequest_t),
    [MPI2_FUNCTION_PORT_FACTS]          = sizeof(Mpi2PortFactsRequest_t),
    [MPI2_FUNCTION_PORT_ENABLE]         = sizeof(Mpi2PortEnableRequest_t),
    [MPI2_FUNCTION_EVENT_NOTIFICATION]  = sizeof(Mpi2EventNotificationRequest_t),
    [MPI2_FUNCTION_EVENT_ACK]           = sizeof(Mpi2EventAckRequest_t),
    [MPI2_FUNCTION_FW_DOWNLOAD]         = sizeof(Mpi2FWDownloadRequest),
    //[MPI2_FUNCTION_TARGET_ASSIST]       = sizeof(Mpi2TargetAssistRequest_t),
};
#if 0
static inline void trace_mpt3sas_all(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    if (trace_event_get_state(TRACE_MPT3SAS_ALL)) {
        qemu_log(fmt, ap);
    }
    va_end(ap);
}
#endif

#ifdef DEBUG_MPT3SAS
#define DPRINTF(fmt, ...) \
    do { qemu_log_mask(LOG_TRACE, "mpt3sas: " fmt, ##__VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

//#define MPT3SAS_FLAG_USE_MSI    0
//#define MPT3SAS_MASK_USE_MSI    (1 << MPT3SAS_FLAG_USE_MSI)

#define MPT3SAS_FLAG_USE_MSIX   1
#define MPT3SAS_MASK_USE_MSIX   (1 << MPT3SAS_FLAG_USE_MSIX)

const char *register_description[] = {
    [MPI2_DOORBELL_OFFSET] = "DOORBELL",
    [MPI2_WRITE_SEQUENCE_OFFSET] = "WRITE SEQUENCE",
    [MPI2_HOST_DIAGNOSTIC_OFFSET] = "HOST DIAGNOSTIC",
    [MPI2_DIAG_RW_DATA_OFFSET] = "DIAG_RW_DATA_OFFSET",
    [MPI2_HOST_INTERRUPT_STATUS_OFFSET] = "HOST INTERRUPT STATUS",
    [MPI2_HOST_INTERRUPT_MASK_OFFSET] = "HOST INTERRUPT MASK", 
    [MPI2_DCR_DATA_OFFSET] = "DCR DATA",
    [MPI2_DCR_ADDRESS_OFFSET] = "DCR ADDRESS",
    [MPI2_REPLY_FREE_HOST_INDEX_OFFSET] = "REPLY FREE HOST INDEX",
    [MPI2_REPLY_POST_HOST_INDEX_OFFSET] = "REPLY POST HOST INDEX",
    [MPI25_SUP_REPLY_POST_HOST_INDEX_OFFSET] = "SUP REPLY POST HOST INDEX",
    [MPI2_HCB_SIZE_OFFSET] = "HCB SIZE",
    [MPI2_HCB_ADDRESS_LOW_OFFSET] = "HCB ADDRESS LOW",
    [MPI2_HCB_ADDRESS_HIGH_OFFSET] = "HCB ADDRESS HIGH",
    [MPI26_SCRATCHPAD0_OFFSET] = "SCRATCHPAD0",
    [MPI26_SCRATCHPAD1_OFFSET] = "SCRATCHPAD1",
    [MPI26_SCRATCHPAD2_OFFSET] = "SCRATCHPAD2",
    [MPI26_SCRATCHPAD3_OFFSET] = "SCRATCHPAD3",
    [MPI2_REQUEST_DESCRIPTOR_POST_LOW_OFFSET] = "REQUEST DESCRIPTOR POST LOW",
    [MPI2_REQUEST_DESCRIPTOR_POST_HIGH_OFFSET] = "REQUEST DESCRIPTOR POST HIGH",
    [MPI26_ATOMIC_REQUEST_DESCRIPTOR_POST_OFFSET] = "ATOMIC REQUEST DESCRIPTOR POST", 
};

static void mpt3sas_update_interrupt(MPT3SASState *s)
{
    PCIDevice *pci =(PCIDevice *)s;
    
    //uint32_t state = s->intr_status & ~(s->intr_mask | MPI2_HIS_IOP_DOORBELL_STATUS);
    uint32_t state = s->intr_status & ~(s->intr_mask | MPI2_HIS_SYS2IOC_DB_STATUS);
    //DPRINTF("%s interrupt state 0x%x\n", __func__, state);
    pci_set_irq(pci, !!state);
}

static void mpt3sas_interrupt_status_write(MPT3SASState *s)
{
//    DPRINTF("%s:%d doorbell state %d\n", __func__, __LINE__, s->doorbell_state);
    switch (s->doorbell_state) {
        case DOORBELL_NONE:
        case DOORBELL_WRITE:
            //s->intr_status &= ~MPI2_HIS_DOORBELL_INTERRUPT;
            s->intr_status &= ~MPI2_HIS_IOC2SYS_DB_STATUS;
            break;
        case DOORBELL_READ:
            //assert(s->intr_status & MPI2_HIS_DOORBELL_INTERRUPT);
            assert(s->intr_status & MPI2_HIS_IOC2SYS_DB_STATUS);
            if (s->doorbell_reply_idx == s->doorbell_reply_size) {
                DPRINTF("%s:%d Change doorbell state from %d to DOOBELL_NONE\n", __func__, __LINE__, s->doorbell_state);
                s->doorbell_state = DOORBELL_NONE;
            }
            break;
        default:
            abort();
    }

 //   DPRINTF("%s:%d doorbell state %d\n", __func__, __LINE__, s->doorbell_state);
    mpt3sas_update_interrupt(s);
}

static void mpt3sas_set_fault(MPT3SASState *s, uint32_t code)
{
    if ((s->state & MPI2_IOC_STATE_FAULT) == 0)
        s->state = MPI2_IOC_STATE_FAULT | code;
}

static void mpt3sas_reply(MPT3SASState *s, MPI2DefaultReply_t *reply)
{
    if (s->doorbell_state == DOORBELL_WRITE) {
        DPRINTF("%s:%d Change doorbell state from DOORBELL_WRITE to DOORBELL_READ\n", __func__, __LINE__);
        s->doorbell_state = DOORBELL_READ;
        s->doorbell_reply_idx = 0;
        s->doorbell_reply_size = reply->MsgLength * 2;
        memcpy(s->doorbell_reply, reply, s->doorbell_reply_size * 2);
        s->intr_status |= MPI2_HIS_IOC2SYS_DB_STATUS;
        mpt3sas_update_interrupt(s);
    }
}

static void mpt3sas_handle_ioc_facts(MPT3SASState *s,Mpi2IOCFactsRequest_t *req)
{
    Mpi2IOCFactsReply_t reply;

    DPRINTF("----------> Handle IOC FACTS.\n");
    memset(&reply, 0, sizeof(reply));
    reply.MsgVersion = 0x0205;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.HeaderVersion = 0x2a00;
    reply.IOCNumber = 1;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.IOCExceptions = 0x1FF; //Report all errors.
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;
    reply.IOCLogInfo = 0;
    reply.MaxChainDepth = MPT3SAS_MAX_CHAIN_DEPTH;
    reply.WhoInit = s->who_init;
    reply.NumberOfPorts = MPT3SAS_NUM_PORTS;
    reply.MaxMSIxVectors = MPT3SAS_MAX_MSIX_VECTORS;
    reply.RequestCredit = MPT3SAS_MAX_OUTSTANDING_REQUESTS;
    reply.ProductID = MPT3SAS_LSI3008_PRODUCT_ID;
    reply.IOCCapabilities = 0x0; //TODO
    //reply.FWVersion = 0x20000000;
    reply.IOCRequestFrameSize = 2048; //TODO
    reply.IOCMaxChainSegmentSize = 0;
    reply.MaxInitiators = 1;
    reply.MaxTargets = s->max_devices;
    reply.MaxSasExpanders = 0x1;
    reply.MaxEnclosures = 0x1;
    reply.ProtocolFlags = MPI2_IOCFACTS_PROTOCOL_SCSI_INITIATOR | MPI2_IOCFACTS_PROTOCOL_SCSI_TARGET;
    reply.HighPriorityCredit = 64; //TODO
    reply.MaxReplyDescriptorPostQueueDepth = 8192; //TODO
    reply.ReplyFrameSize = 128; //TODO
    reply.MaxVolumes = 0x0; //TODO
    reply.MaxDevHandle = s->max_devices;
    reply.MaxPersistentEntries = 0x0; //TODO
    reply.CurrentHostPageSize = 0x0; //TODO

    mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_port_facts(MPT3SASState *s, Mpi2PortFactsRequest_t *req)
{
    DPRINTF("---------------- Handle PORT FACTS\n");
    Mpi2PortFactsReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.PortNumber = req->PortNumber;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;
    reply.IOCLogInfo = 0x0;
    if (req->PortNumber < MPT3SAS_NUM_PORTS) {
        reply.PortType = MPI2_PORTFACTS_PORTTYPE_SAS_PHYSICAL;
        reply.MaxPostedCmdBuffers = 128; //TODO
    }

    mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_ioc_init(MPT3SASState *s, Mpi2IOCInitRequest_t *req)
{
    Mpi2IOCInitReply_t reply;
    DPRINTF("---------------- Handle IOC INIT\n");

    s->who_init = req->WhoInit;
    s->host_page_size = req->HostPageSize;
    s->host_msix_vectors = req->HostMSIxVectors;
    s->system_request_frame_size = req->SystemRequestFrameSize;
    s->reply_descriptor_post_queue_depth = req->ReplyDescriptorPostQueueAddress;
    s->reply_free_queue_depth = req->ReplyFreeQueueDepth;
    s->sense_buffer_address_hi = (hwaddr)req->SenseBufferAddressHigh;
    s->system_reply_address_hi = (hwaddr)req->SystemReplyAddressHigh;
    s->system_request_frame_base_address = (hwaddr)req->SystemRequestFrameBaseAddress;
    s->reply_descriptor_post_queue_address = (hwaddr)req->ReplyDescriptorPostQueueAddress;
    s->reply_free_queue_address = (hwaddr)req->ReplyFreeQueueAddress;

    if (s->state == MPI2_IOC_STATE_READY) {
        s->state = MPI2_IOC_STATE_OPERATIONAL;
    }

    memset(&reply, 0, sizeof(reply));
    reply.WhoInit = s->who_init;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;
    reply.IOCLogInfo = 0x0;
    
    mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
}

//static void mpt3sas_handle_event_notification(MPT3SASState *s, Mpi2EventNotificationRequest_t *req)
//{
//    DPRINTF("---------------> Handle EVENT NOTIFICATION \n");
//    Mpi2EventNotificationReply_t reply;
//
//    memset(&reply, 0, sizeof(reply));
//    reply.EventDataLength = sizeof(reply.EventData) / 4;
//    reply.MsgLength = sizeof(reply) / 4;
//    reply.Function = req->Function;
//    reply.MsgFlags = req->MsgFlags;
//    reply.Event = MPI2_EVENT_EVENT_CHANGE;
//
//    mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
//}

static void mpt3sas_handle_message(MPT3SASState *s, MPI2RequestHeader_t *req)
{
    uint8_t i = 0;

    uint32_t *msg = (uint32_t *)req;
    DPRINTF("----DOORBELL MESSAGE-------\n");
    qemu_log_mask(LOG_TRACE, "\toffset:data\n");
    for (i = 0; i < s->doorbell_cnt; i++) {
        qemu_log_mask(LOG_TRACE, "\t[0x%02x]:%08x\n", i*4, le32_to_cpu(msg[i]));
    }
    switch (req->Function) {
        case MPI2_FUNCTION_SCSI_IO_REQUEST:
            DPRINTF("** NOT IMPLEMENTED SCSI_IO_REQUEST **\n");
            break;
        case MPI2_FUNCTION_SCSI_TASK_MGMT:
            DPRINTF("** NOT IMPLEMENTED SCSI_TASK_MGMT **\n");
            break;
        case MPI2_FUNCTION_IOC_INIT:
            mpt3sas_handle_ioc_init(s, (Mpi2IOCInitRequest_t *)req);
            break;
        case MPI2_FUNCTION_CONFIG:
            DPRINTF("** NOT IMPLEMENTED CONFIG **\n");
            break;
        case MPI2_FUNCTION_IOC_FACTS:
            mpt3sas_handle_ioc_facts(s, (Mpi2IOCFactsRequest_t *)req);
            break;
        case MPI2_FUNCTION_PORT_FACTS:
            mpt3sas_handle_port_facts(s, (Mpi2PortFactsRequest_t *)req);
            break;
        case MPI2_FUNCTION_PORT_ENABLE:
            DPRINTF("** NOT IMPLEMENTED PORT_ENABLE **\n");
            break;
       // case MPI2_FUNCTION_EVENT_NOTIFICATION:
       //     mpt3sas_handle_event_notification(s, (Mpi2EventNotificationRequest_t *)req);
       //     break;
        case MPI2_FUNCTION_EVENT_ACK:
            DPRINTF("** NOT IMPLEMENTED EVENT_ACK **\n");
            break;
        case MPI2_FUNCTION_FW_DOWNLOAD:
            DPRINTF("** NOT IMPLEMENTED FW_DOWNLOAD **\n");
            break;
        default:
            DPRINTF("** UNSUPPORTED FUNCTION: 0x%x ** \n", req->Function);
            mpt3sas_set_fault(s, MPI2_IOCSTATUS_INVALID_FUNCTION);
            break;
    }
}

static void mpt3sas_soft_reset(MPT3SASState *s)
{
    DPRINTF("%s:%d\n", __func__, __LINE__);
    uint32_t save_mask;

    save_mask = s->intr_mask;
    s->intr_mask = MPI2_HIM_RESET_IRQ_MASK | MPI2_HIM_DIM | MPI2_HIM_DIM;
    mpt3sas_update_interrupt(s);

    qbus_reset_all(&s->bus.qbus);
    s->intr_status = 0;
    s->intr_mask = save_mask;

    s->state = MPI2_IOC_STATE_READY;
}

static void mpt3sas_hard_reset(MPT3SASState *s)
{
    s->state = MPI2_IOC_STATE_RESET;
    mpt3sas_soft_reset(s);
    s->intr_mask = MPI2_HIM_RESET_IRQ_MASK | MPI2_HIM_DIM | MPI2_HIM_DIM;
    s->max_devices = MPT3SAS_NUM_PORTS;
    s->max_buses = 1;
}

static uint32_t mpt3sas_doorbell_read(MPT3SASState *s)
{
    uint32_t retval = 0;

    retval = (s->who_init << MPI2_DOORBELL_WHO_INIT_SHIFT) & MPI2_DOORBELL_WHO_INIT_MASK;

    retval |= s->state;
    switch (s->doorbell_state) {
        case DOORBELL_NONE:
            break;
        case DOORBELL_WRITE:
            retval |= MPI2_DOORBELL_USED;
            break;
        case DOORBELL_READ:
            retval &= ~MPI2_DOORBELL_DATA_MASK;
            assert(s->intr_status & MPI2_HIS_IOC2SYS_DB_STATUS);
            assert(s->doorbell_reply_idx <=s->doorbell_reply_size);

            DPRINTF("%s:%d doorbell reply index %d, doorbell reply size: 0x%x\n", __func__, __LINE__, s->doorbell_reply_idx, s->doorbell_reply_size);
            retval |= MPI2_DOORBELL_USED;
            if (s->doorbell_reply_idx < s->doorbell_reply_size) {
                //retval |= le16_to_cpu(s->doorbell_reply[s->doorbell_reply_idx++]);
                retval |= s->doorbell_reply[s->doorbell_reply_idx++];
            }
            break;
        default:
            abort();
    }

    return retval;
}

static void mpt3sas_doorbell_write(MPT3SASState *s, uint32_t val)
{
    uint8_t function;

    if (s->doorbell_state == DOORBELL_WRITE) {
        DPRINTF("%s:%d Request in doorbell val 0x%x, doorbell index %d\n", __func__, __LINE__, val, s->doorbell_idx);
        if (s->doorbell_idx < s->doorbell_cnt) {
            s->doorbell_msg[s->doorbell_idx++] = cpu_to_le32(val);
            if (s->doorbell_idx == s->doorbell_cnt) {
                mpt3sas_handle_message(s, (MPI2RequestHeader_t *)s->doorbell_msg);
            }
        }
        return;
    }

    function = (val & MPI2_DOORBELL_FUNCTION_MASK) >> MPI2_DOORBELL_FUNCTION_SHIFT;
    switch(function) {
        case MPI2_FUNCTION_IOC_MESSAGE_UNIT_RESET:
            mpt3sas_soft_reset(s);
            break;
        case MPI2_FUNCTION_HANDSHAKE:
            s->doorbell_state = DOORBELL_WRITE;
            s->doorbell_idx = 0;
            s->doorbell_cnt = (val & MPI2_DOORBELL_ADD_DWORDS_MASK) >> MPI2_DOORBELL_ADD_DWORDS_SHIFT;
            s->intr_status |= MPI2_HIS_IOC2SYS_DB_STATUS; 
            DPRINTF("%s HANDSHAKE function,doorbell count %d, doorbell state %d\n", __func__, s->doorbell_cnt, s->doorbell_state);
            mpt3sas_update_interrupt(s);
            break;
        default:
            DPRINTF("%s ****** unhandled doorbell function 0x%x\n", __func__, function);
            break;
    }
}

static uint64_t mpt3sas_mmio_read(void *opaque, hwaddr addr,
        unsigned size)
{
    //MPT3SASState *s = opaque,
    uint32_t ret = 0;

    MPT3SASState *s = opaque;


    DPRINTF("%s:%d Read register [ %s ], size = %d\n", __func__, __LINE__,
            register_description[addr & ~3], size);

    switch (addr & ~3) {
        case MPI2_DOORBELL_OFFSET:
            ret = mpt3sas_doorbell_read(s);
            break;
        case MPI2_HOST_DIAGNOSTIC_OFFSET:
            ret = s->host_diag;
            break;
        case MPI2_DIAG_RW_DATA_OFFSET:
            break;
        case MPI2_HOST_INTERRUPT_STATUS_OFFSET:
            ret = s->intr_status;
            break;
        case MPI2_HOST_INTERRUPT_MASK_OFFSET:
            ret = s->intr_mask;
            break;
        case MPI2_DCR_DATA_OFFSET:
            break;
        case MPI2_DCR_ADDRESS_OFFSET:
            break;
        case MPI2_REPLY_FREE_HOST_INDEX_OFFSET:
            break;
        case MPI2_REPLY_POST_HOST_INDEX_OFFSET:
            break;
        case MPI25_SUP_REPLY_POST_HOST_INDEX_OFFSET:
            break;
        case MPI2_HCB_SIZE_OFFSET:
            ret = s->hcb_size;
            break;
        case MPI2_HCB_ADDRESS_LOW_OFFSET:
            break;
        case MPI2_HCB_ADDRESS_HIGH_OFFSET:
            break;
        case MPI26_SCRATCHPAD0_OFFSET:
            break;
        case MPI26_SCRATCHPAD1_OFFSET:
            break;
        case MPI26_SCRATCHPAD2_OFFSET:
            break;
        case MPI26_SCRATCHPAD3_OFFSET:
            break;
        case MPI2_REQUEST_DESCRIPTOR_POST_LOW_OFFSET:
            break;
        case MPI2_REQUEST_DESCRIPTOR_POST_HIGH_OFFSET:
            break;
        case MPI26_ATOMIC_REQUEST_DESCRIPTOR_POST_OFFSET:
            break;
        default:
              DPRINTF("%s:%d Unknown offset 0x%lx\n", __func__, __LINE__, addr & ~3);
              break;
    }
    DPRINTF("%s:%d Register [ %s ] returned value 0x%x\n", __func__, __LINE__, register_description[addr & ~3], ret);
    return ret;
}

static void mpt3sas_mmio_write(void *opaque, hwaddr addr,
        uint64_t val, unsigned size)
{
    MPT3SASState *s = opaque;
    DPRINTF("%s:%d Write register [ %s ], val = 0x%lx, size = %d\n", __func__, __LINE__,
            register_description[addr], val, size);
    switch (addr) {
        case MPI2_DOORBELL_OFFSET:
            mpt3sas_doorbell_write(s, val);
            break;
        case MPI2_WRITE_SEQUENCE_OFFSET:
            if (ioc_reset_sequence[s->ioc_reset] == val) {
                s->ioc_reset++;
            } else if (val == MPI2_WRSEQ_FLUSH_KEY_VALUE){
                s->ioc_reset = 0;
                s->host_diag = 0;
            } else {
                abort();
            }

            if (s->ioc_reset == 6) {
                s->host_diag = MPI2_DIAG_DIAG_WRITE_ENABLE; 
            }
            break;
        case MPI2_HOST_DIAGNOSTIC_OFFSET:
            if ((s->host_diag & MPI2_DIAG_DIAG_WRITE_ENABLE) &&
                val & MPI2_DIAG_RESET_ADAPTER) {
                s->host_diag |= MPI2_DIAG_RESET_ADAPTER;
                mpt3sas_soft_reset(s);
                s->ioc_reset = 0;
                s->host_diag = 0;
                s->hcb_size = 0x40000; //PCI WINDOW SIZE
            }
            break;
        case MPI2_DIAG_RW_DATA_OFFSET:
            break;
        case MPI2_HOST_INTERRUPT_STATUS_OFFSET:
            mpt3sas_interrupt_status_write(s);
            break;
        case MPI2_HOST_INTERRUPT_MASK_OFFSET:
            s->intr_mask = val & (MPI2_HIM_RIM | MPI2_HIM_DIM | MPI2_HIM_RESET_IRQ_MASK);
            mpt3sas_update_interrupt(s);
            break;
        case MPI2_DCR_DATA_OFFSET:
            break;
        case MPI2_DCR_ADDRESS_OFFSET:
            break;
        case MPI2_REPLY_FREE_HOST_INDEX_OFFSET:
            s->reply_free_host_index = val;
            break;
        case MPI2_REPLY_POST_HOST_INDEX_OFFSET:
            s->reply_post_host_index = val;
            break;
        case MPI25_SUP_REPLY_POST_HOST_INDEX_OFFSET:
            break;
        case MPI2_HCB_SIZE_OFFSET:
            s->hcb_size = val;
            break;
        case MPI2_HCB_ADDRESS_LOW_OFFSET:
            break;
        case MPI2_HCB_ADDRESS_HIGH_OFFSET:
            break;
        case MPI26_SCRATCHPAD0_OFFSET:
              break;
        case MPI26_SCRATCHPAD1_OFFSET:
              break;
        case MPI26_SCRATCHPAD2_OFFSET:
            break;
        case MPI26_SCRATCHPAD3_OFFSET:
            break;
        case MPI2_REQUEST_DESCRIPTOR_POST_LOW_OFFSET:
            s->request_descriptor_post = (hwaddr)(val & 0xffffffff);
            break;
        case MPI2_REQUEST_DESCRIPTOR_POST_HIGH_OFFSET:
            s->request_descriptor_post |= (val & 0xffffffff) << 32;
            break;
        case MPI26_ATOMIC_REQUEST_DESCRIPTOR_POST_OFFSET:
            break;
        default:
              DPRINTF("%s:%d Unknown offset 0x%lx\n", __func__, __LINE__, addr);
              break;
    }

}

static const MemoryRegionOps mpt3sas_mmio_ops = {
    .read = mpt3sas_mmio_read,
    .write = mpt3sas_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static const MemoryRegionOps mpt3sas_port_ops = {
    .read = mpt3sas_mmio_read,
    .write = mpt3sas_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

//static uint64_t mpt3sas_diag_read(void *opaque, hwaddr addr,
//        unsigned size)
//{
//    DPRINTF("%s:%d addr = 0x%lx, size = %d\n", __func__, __LINE__,
//            addr, size);
//    return 0;
//}
//
//static void mpt3sas_diag_write(void *opaque, hwaddr addr,
//        uint64_t val, unsigned size)
//{
//    DPRINTF("%s:%d addr = 0x%lx, val = 0x%lx, size = %d\n", __func__, __LINE__,
//            addr, val, size);
//}
//
//static const MemoryRegionOps mpt3sas_diag_ops = {
//    .read = mpt3sas_diag_read,
//    .write = mpt3sas_diag_write,
//    .endianness = DEVICE_LITTLE_ENDIAN,
//    .impl = {
//        .min_access_size = 4,
//        .max_access_size = 4,
//    }
//};

static QEMUSGList *mpt3sas_get_sg_list(SCSIRequest *sreq)
{
    return NULL;
}

static void mpt3sas_command_complete(SCSIRequest *sreq,
        uint32_t status, size_t resid)
{
}

static void mpt3sas_request_cancelled(SCSIRequest *sreq)
{
}

#if 0
# TODO: not sure what are used for.
static void mpt3sas_save_request(QEMUFile *f, SCSIRequest *sreq)
{
}


static void *mpt3sas_load_request(QEMUFile *f, SCSIRequest *sreq)
{
    return NULL;
}
#endif

static const struct SCSIBusInfo mpt3sas_scsi_info = {
    .tcq = true,
    .max_target = MPT3SAS_NUM_PORTS,
    .max_lun = 1,

    .get_sg_list = mpt3sas_get_sg_list,
    .complete = mpt3sas_command_complete,
    .cancel = mpt3sas_request_cancelled,
//    .save_request = mpt3sas_save_request,
//    .load_request = mpt3sas_load_request,

};

static void mpt3sas_scsi_init(PCIDevice *dev, Error **errp)
{
    DeviceState *d = DEVICE(dev);
    MPT3SASState *s = MPT3SAS(dev);

    DPRINTF("%s:%d: initialize start.\n", __func__, __LINE__);
    dev->config[PCI_LATENCY_TIMER] = 0;
    dev->config[PCI_INTERRUPT_PIN] = 0x01;
    memory_region_init_io(&s->mmio_io, OBJECT(s), &mpt3sas_mmio_ops, s,
            "mpt3sas-mmio", 0x10000);
    memory_region_init_io(&s->port_io, OBJECT(s), &mpt3sas_port_ops, s,
            "mpt3sas-io", 256);
//    memory_region_init_io(&s->diag_io, OBJECT(s), &mpt3sas_diag_ops, s,
//            "mpt3sas-diag", 0x10000);

    // nentries - 15 ??
    // table_bar_nr 0x1
    // table_offset 0x2000??
    // pba_bar_nr 0x1
    // pba_offset 0x3800 ??
    // cap_pos 0x68 ??
    if (s->msix_available  &&
        !msix_init(dev, 15, &s->mmio_io, 0x1, 0x2000,
            &s->mmio_io, 0x1, 0x3800, 0x68)) {
        DPRINTF("Initialize msix ok.\n");
        s->msix_in_use = true;
    }

    if (pci_is_express(dev)) {
        pcie_endpoint_cap_init(dev, 0xa0);
    }

    // bar0 for IO space, size: 256 bytes
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->port_io);

    // bar1 for memory io space, size: 64K
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY |
                                 PCI_BASE_ADDRESS_MEM_TYPE_64, &s->mmio_io);

    // bar2 for memory io space ester_bar
    pci_register_bar(dev, 3, PCI_BASE_ADDRESS_SPACE_MEMORY |
                                 PCI_BASE_ADDRESS_MEM_TYPE_64, &s->diag_io);


    if (s->msix_available) {
        msix_vector_use(dev, 0);
    }

    if (!s->sas_addr) {
        s->sas_addr = ((NAA_LOCALLY_ASSIGNED_ID << 24) |
                       IEEE_COMPANY_LOCALLY_ASSIGNED) << 36;
        s->sas_addr |= (pci_bus_num(dev->bus) << 16);
        s->sas_addr |= (PCI_SLOT(dev->devfn) << 8);
        s->sas_addr |= PCI_FUNC(dev->devfn);
    }

    scsi_bus_new(&s->bus, sizeof(s->bus), &dev->qdev, &mpt3sas_scsi_info, NULL);

    if (!d->hotplugged) {
        scsi_bus_legacy_handle_cmdline(&s->bus, errp);
    }

    // Do the hardreset
    mpt3sas_hard_reset(s);
}

static void mpt3sas_scsi_uninit(PCIDevice *dev)
{
}

static void mpt3sas_reset(DeviceState *dev)
{
}

static Property mpt3sas_properties[] = {
    DEFINE_PROP_UINT64("sas_address", MPT3SASState, sas_addr, 0),
    DEFINE_PROP_BIT("use_msix", MPT3SASState, msix_available, 0, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void mpt3sas3008_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = mpt3sas_scsi_init;
    pc->exit = mpt3sas_scsi_uninit;
    pc->romfile = 0;
    pc->vendor_id = MPI2_MFGPAGE_VENDORID_LSI;
    pc->device_id = MPI25_MFGPAGE_DEVID_SAS3008;
    pc->subsystem_vendor_id = MPI2_MFGPAGE_VENDORID_LSI;
    pc->subsystem_id = 0x8000;
    pc->class_id = PCI_CLASS_STORAGE_SCSI;
    pc->is_express = true;
    dc->props = mpt3sas_properties;
    dc->reset = mpt3sas_reset;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "LSI SAS 3008";
}

static const TypeInfo mpt3sas_info = {
    .name = TYPE_MPT3SAS3008,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MPT3SASState),
    .class_init = mpt3sas3008_class_init,
};

static void mpt3sas_register_types(void)
{
    type_register(&mpt3sas_info);
}

type_init(mpt3sas_register_types)
