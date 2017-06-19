#ifndef __MPT3SAS_H__
#define __MPT3SAS_H__

#include "hw/scsi/mpi/mpi2_type.h"
#include "hw/scsi/mpi/mpi2.h"
#include "hw/scsi/mpi/mpi2_cnfg.h"
#include "hw/scsi/mpi/mpi2_init.h"
#include "hw/scsi/mpi/mpi2_ioc.h"
#include "hw/scsi/mpi/mpi2_cnfg.h"
#include "hw/scsi/mpi/mpi2_sas.h"

#define MPT3SAS_NUM_PHYS 8

#define MPT3SAS_MAX_CHAIN_DEPTH     128
#define MPT3SAS_MAX_MSIX_VECTORS    16
#define MPT3SAS_MAX_OUTSTANDING_REQUESTS    9680 //max outstanding requests held by driver
#define MPT3SAS_MAX_REPLY_DESCRIPTOR_QUEUE_DEPTH 19440
#define MPT3SAS_REQUEST_FRAME_SIZE   32
#define MPT3SAS_REPLY_FRAME_SIZE      32
#define MPT3SAS_EXPANDER_COUNT  2

#define MPT3SAS_REQUEST_QUEUE_DEPTH MPT3SAS_MAX_OUTSTANDING_REQUESTS

#define MPT3SAS_IOC_HANDLE_START            (0x1)
#define MPT3SAS_EXPANDER_HANDLE_START       (MPT3SAS_IOC_HANDLE_START + MPT3SAS_NUM_PHYS)
//#define MPT3SAS_ATTACHED_DEV_HANDLE_START   (MPT3SAS_IOC_HANDLE_START + MPT3SAS_NUM_PORTS + 1)
#define MPT3SAS_ATTACHED_DEV_HANDLE_START   (MPT3SAS_EXPANDER_HANDLE_START + MPT3SAS_EXPANDER_COUNT)

#define MPT3SAS_EXPANDER_PORT_START_PHY  (20)
#define MPT3SAS_EXPANDER_NUM_PHYS   (28)

#define MPT3SAS_ENCLOSURE_HANDLE_START (0x1)

#define MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR   (0x500056b3e16208ff)

#define MPT3SAS_DRIVE_DEFAULT_SAS_ADDR      (0x51866da05a753a00)

#define MPT3SAS_IOC_ENCLOSURE_HANDLE (0x1)
#define MPT3SAS_EXPANDER_ENCLOSURE_HANDLE (0x2)

#define MPT3SAS_IOC_NUM_SLOTS   MPT3SAS_NUM_PHYS
#define MPT3SAS_EXPANDER_NUM_SLOTS  (MPT3SAS_EXPANDER_NUM_PHYS - MPT3SAS_NUM_PHYS)

typedef struct MPT3SASState MPT3SASState;

enum {
    DOORBELL_NONE,
    DOORBELL_WRITE,
    DOORBELL_READ
};

typedef struct {
    Notifier notifier;
    MPT3SASState *s;
    uint16_t smid;
    uint8_t msix_index;
    Mpi2SCSITaskManagementReply_t *reply;
} MPT3SASCancelNotifier;

typedef struct {
    uint32_t ioc_index;
    uint32_t host_index;
    hwaddr base;
} MPT3SASReplyPost;

typedef struct MPT3SASRequest {
    Mpi25SCSIIORequest_t scsi_io;
    SCSIRequest *sreq;
    QEMUSGList qsg;
    MPT3SASState *dev;
    uint16_t smid; //copy of request smid
    uint8_t msix_index;
    QTAILQ_ENTRY(MPT3SASRequest) next;
} MPT3SASRequest;

typedef struct MPT3SASEventData {
    uint32_t length;
    uint8_t data[1];
} MPT3SASEventData;

typedef struct MPT3SASEvent {
    uint8_t event_type;
    void *data; // for MPT3SASEventData
    QTAILQ_ENTRY(MPT3SASEvent) next;
} MPT3SASEvent;

typedef struct MPT3SASEventQueue {
    QemuCond cond;
    QemuMutex mutex;
    QemuThread thread;
    bool exit;
    QTAILQ_HEAD(, MPT3SASEvent) events;
} MPT3SASEventQueue;

typedef struct Mpi2ManufacturingPage10_t {
    MPI2_CONFIG_PAGE_HEADER Header;
    U8 OEMIdentifier;
    U8 Reserved1;
    U16 Reserved2;
    U32 Reserved3;
    U32 GenericFlags0;
    U32 GenericFlags1;
    U32 Reserved4;
    U32 OEMSpecificFlags0;
    U32 OEMSpecificFlags1;
    U32 Reserved5[18];
} Mpi2ManufacturingPage10_t;


struct MPT3SASState {
    PCIDevice dev;
    MemoryRegion mmio_io;
    MemoryRegion port_io;
    MemoryRegion diag_io;
    QEMUBH *request_bh;

    QEMUBH *event_handler;
    uint16_t event;

    uint32_t max_devices;
    uint32_t max_buses;

    uint32_t msix_available;
    bool msix_in_use;

    /* interrupt register */
    uint32_t intr_status;
    uint32_t intr_mask;

    /* Doorbell Register*/
    //IOC State -> RESET, READY, OPERATIONAL, FAULT
    uint32_t state;
    uint8_t who_init;
    uint8_t doorbell_state;

    uint32_t doorbell_msg[256];
    int doorbell_idx;
    int doorbell_cnt;

    uint16_t doorbell_reply[256];
    int doorbell_reply_idx;
    int doorbell_reply_size;

    uint8_t ioc_reset;
    uint32_t host_diag;

    uint32_t hcb_size;

    // Request queues

    //maintained by ioc internal, tracks the head of the reply queue,
    //the host maintained the tail of the reply queue.
    uint32_t reply_free_ioc_index;  // head of reply free queue
    uint32_t reply_free_host_index; // tail of reply free queue

    MPT3SASReplyPost reply_post[MPT3SAS_MAX_MSIX_VECTORS];

    // maintained internally, private to IOC
    uint64_t request_descriptor_post[MPT3SAS_REQUEST_QUEUE_DEPTH + 1];
    uint32_t request_descriptor_post_head;
    uint32_t request_descriptor_post_tail;

    // host configured settings through IOCInit
    uint8_t host_page_size;
    uint8_t host_msix_vectors;
    uint16_t system_request_frame_size;
    uint16_t reply_descriptor_post_queue_depth;
    uint16_t reply_free_queue_depth;
    uint32_t sense_buffer_address_hi;
    uint32_t system_reply_address_hi;
    hwaddr system_request_frame_base_address;
    hwaddr reply_descriptor_post_queue_address;
    hwaddr reply_free_queue_address;
    uint64_t time_stamp;

    uint64_t sas_address;

    MPT3SASRequest *completed_queue[MPT3SAS_MAX_REPLY_DESCRIPTOR_QUEUE_DEPTH + 1];
    uint32_t completed_queue_head;
    uint32_t completed_queue_tail;
    uint64_t completed_commands;

    MPT3SASEventQueue *event_queue;

    struct {
        uint32_t count;
        uint32_t all_phys;
        uint32_t upstream_phys;
        uint32_t downstream_phys;
        uint32_t downstream_start_phy;
        uint32_t upstream_start_phy;

    } expander;

    SCSIBus bus;
    QTAILQ_HEAD(, MPT3SASRequest) pending;
    QEMUBH *completed_request_bh;
};

#endif
