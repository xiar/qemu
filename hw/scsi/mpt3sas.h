#ifndef __MPT3SAS_H__
#define __MPT3SAS_H__

#define MPT3SAS_NUM_PORTS   8

#define MPT3SAS_MAX_CHAIN_DEPTH     64
#define MPT3SAS_MAX_MSIX_VECTORS    8
#define MPT3SAS_MAX_OUTSTANDING_REQUESTS    64 //max outstanding requests held by driver

typedef struct MPT3SASState MPT3SASState;

enum {
    DOORBELL_NONE,
    DOORBELL_WRITE,
    DOORBELL_READ
};

struct MPT3SASState {
    PCIDevice dev;
    MemoryRegion mmio_io;
    MemoryRegion port_io;
    MemoryRegion diag_io;

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

    SCSIBus bus;
};

#endif
