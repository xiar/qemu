#ifndef __MPT3SAS_H__
#define __MPT3SAS_H__

#include "hw/scsi/mpi/mpi2_type.h"
#include "hw/scsi/mpi/mpi2.h"
#include "hw/scsi/mpi/mpi2_cnfg.h"
#include "hw/scsi/mpi/mpi2_init.h"
#include "hw/scsi/mpi/mpi2_ioc.h"
#include "hw/scsi/mpi/mpi2_cnfg.h"
#include "hw/scsi/mpi/mpi2_sas.h"

#define MPT3SAS_NUM_PORTS   8

#define MPT3SAS_MAX_CHAIN_DEPTH     128
#define MPT3SAS_MAX_MSIX_VECTORS    8
#define MPT3SAS_MAX_OUTSTANDING_REQUESTS    9680 //max outstanding requests held by driver
#define MPT3SAS_MAX_REPLY_DESCRIPTOR_QUEUE_DEPTH 19440

#define MPT3SAS_REQUEST_FRAME_SIZE   32
#define MPT3SAS_REPLY_FRAME_SIZE      32

#define MPT3SAS_REQUEST_QUEUE_DEPTH MPT3SAS_MAX_OUTSTANDING_REQUESTS
#define MPT3SAS_REPLY_QUEUE_DEPTH   128


typedef struct MPT3SASState MPT3SASState;

enum {
    DOORBELL_NONE,
    DOORBELL_WRITE,
    DOORBELL_READ
};

typedef struct {
    Notifier notifier;
    MPT3SASState *s;
    Mpi2SCSITaskManagementReply_t *reply;
} MPT3SASCancelNotifier;

typedef struct MPT3SASRequest {
    Mpi25SCSIIORequest_t scsi_io;
    SCSIRequest *sreq;
    QEMUSGList qsg;
    MPT3SASState *dev;
    uint16_t smid; //copy of request smid
    QTAILQ_ENTRY(MPT3SASRequest) next;
} MPT3SASRequest;

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
    uint64_t sas_addr;

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

    uint32_t reply_post_ioc_index;  // tail of reply post queue
    uint32_t reply_post_host_index; // head of reply post queue

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

    // internal use
    uint16_t controller_dev_handle;
    Mpi2SasIOUnitPage0_t sas_iounit_pg0;

    uint64_t sas_address;

    SCSIBus bus;
    QTAILQ_HEAD(, MPT3SASRequest) pending;
};

#endif
