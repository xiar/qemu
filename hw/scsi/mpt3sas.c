/*
 * QEMU LSI SAS3008 Host Bus Adapter emulation
 * Based on the QEMU Megaraid and MPTSAS emulator 
 *
 * Copyright (c) 2017 DellEMC 
 *
 * Author: Robert Xia
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

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
#include "trace/control.h"
#include "qemu/log.h"
#include "trace.h"

#define NAA_LOCALLY_ASSIGNED_ID 0x3ULL
#define IEEE_COMPANY_LOCALLY_ASSIGNED 0x51866d

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
    [MPI2_FUNCTION_SCSI_IO_REQUEST]     = sizeof(Mpi25SCSIIORequest_t),
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

#ifdef DEBUG_MPT3SAS
#define DPRINTF(fmt, ...) \
    do { struct timeval _now; gettimeofday(&_now, NULL); qemu_log_mask(LOG_TRACE, "[%zd.%06zd] mpt3sas: " fmt, (size_t)_now.tv_sec, (size_t)_now.tv_usec, ##__VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

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

//_________________LSI SAS3008 CONFIG PAGE____________________________
//
typedef struct MPT3SASConfigPage {
    uint8_t number;
    uint8_t type;
    size_t (*mpt_config_build)(MPT3SASState *s, uint8_t **data, int address);
}MPT3SASConfigPage;

static uint8_t device_handle_to_phy(uint16_t dev_handle)
{
    if (dev_handle < MPT3SAS_NUM_PORTS + 1) {
        return 0;
    }

    return dev_handle - (MPT3SAS_NUM_PORTS + 1);
}

static SCSIDevice *mpt3sas_phy_get_device(MPT3SASState *s, uint32_t i, uint16_t *dev_handle)
{
    SCSIDevice *d = scsi_device_find(&s->bus, 0, i, 0);
    if (dev_handle) {
        *dev_handle = d ? i + 1 + MPT3SAS_NUM_PORTS : 0;
    }

    return d;
}

static void mpt3sas_free_request(MPT3SASRequest *req)
{
    MPT3SASState *s = req->dev;

    if (req->sreq) {
        req->sreq->hba_private = NULL;
        scsi_req_unref(req->sreq);
        req->sreq = NULL;
        QTAILQ_REMOVE(&s->pending, req, next);
    }
    qemu_sglist_destroy(&req->qsg);
    g_free(req);
}

static void __attribute__((unused)) mpt3sas_print_scsi_devices(SCSIBus *bus)
{
    BusChild *kid;
    QTAILQ_FOREACH_REVERSE(kid, &bus->qbus.children, ChildrenHead, sibling) {
        DeviceState *qdev = kid->child;
        SCSIDevice *dev = SCSI_DEVICE(qdev);
        DPRINTF("%s:%d SCSI Device (%p) channel %d, scsi id %d lun %d\n",
                __func__, __LINE__, dev, dev->channel, dev->id, dev->lun);
    }
}
static int mpt3sas_scsi_device_find(MPT3SASState *s, uint16_t dev_handle,
        uint8_t *lun, SCSIDevice **sdev)
{
    int target;


    if (!dev_handle || dev_handle == 0xffff) {
        return MPI2_IOCSTATUS_SCSI_INVALID_DEVHANDLE;
    }

    target = device_handle_to_phy(dev_handle);

    if (target >= s->max_devices) {
        return MPI2_IOCSTATUS_SCSI_DEVICE_NOT_THERE;
    }

    *sdev = scsi_device_find(&s->bus, 0, target, lun[1]);
    if (!*sdev) {
        return MPI2_IOCSTATUS_SCSI_DEVICE_NOT_THERE;
    }

    return 0;
}

//TODO: Most the config pages need to be configured again for making the host driver works.
// currently just for make host linux driver initialization working
static size_t mpt3sas_config_manufacturing_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ManufacturingPage0_t man_pg0;

    memset(&man_pg0, 0, sizeof(man_pg0));
    man_pg0.Header.PageVersion = MPI2_MANUFACTURING0_PAGEVERSION;
    man_pg0.Header.PageLength = sizeof(man_pg0) / 4;
    man_pg0.Header.PageNumber = 0x0;
    man_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;
    strcpy((void *)man_pg0.ChipName, "LSI MPT3 Fusion");
    strcpy((void *)man_pg0.ChipRevision, "3.0");
    strcpy((void *)man_pg0.BoardName, "LSI 3008"); 
    strcpy((void *)man_pg0.BoardAssembly, "LSI");
    strcpy((void *)man_pg0.BoardTracerNumber, "112233445566");
    if (data) {
        *data = g_malloc(sizeof(man_pg0));
        memcpy(*data, &man_pg0, sizeof(man_pg0));
    }
    return sizeof(man_pg0);
}

static size_t mpt3sas_config_manufacturing_11(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ConfigPageHeader_t *man_pg11 = NULL;
    uint32_t page_len = sizeof(Mpi2ConfigPageHeader_t) + 0x40; 

    man_pg11 = g_malloc(page_len);
    memset(man_pg11, 0, page_len);
    man_pg11->PageVersion = MPI2_MANUFACTURING11_PAGEVERSION;
    man_pg11->PageNumber = 0xb;
    man_pg11->PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;
    man_pg11->PageLength = page_len / 4;
    // EEDPTagMode
    *((uint8_t *)man_pg11 + 0x9) = 0x1;
    if (data) {
        *data = (uint8_t *)man_pg11;
    }
    return page_len;
}

static size_t mpt3sas_config_bios_2(MPT3SASState *s, uint8_t **data, int address)
{
    MPI2_CONFIG_PAGE_BIOS_2 bios_pg2;

    memset(&bios_pg2, 0, sizeof(bios_pg2));
    bios_pg2.Header.PageVersion = MPI2_BIOSPAGE2_PAGEVERSION;
    bios_pg2.Header.PageNumber = 0x2;
    bios_pg2.Header.PageType = MPI2_CONFIG_PAGETYPE_BIOS;
    bios_pg2.Header.PageLength = sizeof(bios_pg2) / 4;
    bios_pg2.ReqBootDeviceForm = 0x5;
    bios_pg2.ReqAltBootDeviceForm = 0x5;
    bios_pg2.CurrentBootDeviceForm = 0x5;
    if (data) {
        *data = g_malloc(sizeof(bios_pg2));
        memcpy(*data, &bios_pg2, sizeof(bios_pg2));
    }
    return sizeof(bios_pg2);
}

static size_t mpt3sas_config_bios_3(MPT3SASState *s, uint8_t **data, int address)
{
   Mpi2BiosPage3_t  bios_pg3;

   memset(&bios_pg3, 0, sizeof(bios_pg3));
   bios_pg3.Header.PageVersion = MPI2_BIOSPAGE3_PAGEVERSION;
   bios_pg3.Header.PageNumber = 0x3;
   bios_pg3.Header.PageType = MPI2_CONFIG_PAGETYPE_BIOS;
   bios_pg3.Header.PageLength = sizeof(bios_pg3) / 4;
   bios_pg3.GlobalFlags = MPI2_BIOSPAGE3_FLAGS_HOOK_INT_40_DISABLE | MPI2_BIOSPAGE3_FLAGS_VERBOSE_ENABLE;
   bios_pg3.BiosVersion = 0x2030101;
   if (data) {
       *data = g_malloc(sizeof(bios_pg3));
       memcpy(*data, &bios_pg3, sizeof(bios_pg3));
   }
   return sizeof(bios_pg3);
}

static size_t mpt3sas_config_ioc_8(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOCPage8_t ioc_pg8;

    memset(&ioc_pg8, 0, sizeof(ioc_pg8));
    ioc_pg8.Header.PageVersion = MPI2_IOCPAGE8_PAGEVERSION;
    ioc_pg8.Header.PageNumber = 0x8;
    ioc_pg8.Header.PageType = MPI2_CONFIG_PAGETYPE_IOC;
    ioc_pg8.Header.PageLength = sizeof(ioc_pg8) / 4;
    ioc_pg8.NumDevsPerEnclosure = 0x0; //Default value, TODO
    ioc_pg8.MaxPersistentEntries = 0x0;
    ioc_pg8.MaxNumPhysicalMappedIDs = 0x8;
    if (data) {
       *data = g_malloc(sizeof(ioc_pg8));
       memcpy(*data, &ioc_pg8, sizeof(ioc_pg8));
    }
    return sizeof(ioc_pg8);
}

static size_t mpt3sas_config_io_unit_0(MPT3SASState *s, uint8_t **data, int address)
{
    PCIDevice *pci = PCI_DEVICE(s);
    Mpi2IOUnitPage0_t iounit_pg0;

    memset(&iounit_pg0, 0, sizeof(iounit_pg0));
    iounit_pg0.Header.PageVersion = MPI2_IOUNITPAGE0_PAGEVERSION;
    iounit_pg0.Header.PageNumber = 0x0;
    iounit_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;
    iounit_pg0.Header.PageLength = sizeof(iounit_pg0) / 4;
    iounit_pg0.UniqueValue = 0x53504D554D4553LL | (uint64_t)pci->devfn << 56;
    if (data) {
        *data = g_malloc(sizeof(iounit_pg0));
        memcpy(*data, &iounit_pg0, sizeof(iounit_pg0));
    }

    return sizeof(iounit_pg0);
}

static size_t mpt3sas_config_io_unit_1(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOUnitPage1_t iounit_pg1;

    memset(&iounit_pg1, 0, sizeof(iounit_pg1));
    iounit_pg1.Header.PageVersion = MPI2_IOUNITPAGE1_PAGEVERSION;
    iounit_pg1.Header.PageNumber = 0x1;
    iounit_pg1.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;
    iounit_pg1.Header.PageLength = sizeof(iounit_pg1) / 4;
    iounit_pg1.Flags = 0x41;
    if (data) {
        *data = g_malloc(sizeof(iounit_pg1));
        memcpy(*data, &iounit_pg1, sizeof(iounit_pg1));
    }
    
    return sizeof(iounit_pg1);
}

static size_t mpt3sas_config_io_unit_8(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOUnitPage8_t iounit_pg8;

    memset(&iounit_pg8, 0, sizeof(iounit_pg8));
    iounit_pg8.Header.PageVersion = MPI2_IOUNITPAGE8_PAGEVERSION;
    iounit_pg8.Header.PageNumber = 0x8;
    iounit_pg8.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;
    iounit_pg8.NumSensors = 0x8;
    if (data) {
        *data = g_malloc(sizeof(iounit_pg8));
        memcpy(*data, &iounit_pg8, sizeof(iounit_pg8));
    }

    return sizeof(iounit_pg8);
}

static size_t mpt3sas_config_sas_io_unit_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasIOUnitPage0_t *sas_iounit_pg0 = NULL;
    uint32_t i = 0;
    size_t page_len = 0;

    page_len = offsetof(Mpi2SasIOUnitPage0_t, PhyData) + sizeof(Mpi2SasIOUnit0PhyData_t) * MPT3SAS_NUM_PORTS;
    sas_iounit_pg0 = g_malloc(page_len);
    memset(sas_iounit_pg0, 0, page_len);
    sas_iounit_pg0->Header.PageVersion = MPI2_SASIOUNITPAGE0_PAGEVERSION;
    sas_iounit_pg0->Header.PageNumber = 0x0;
    sas_iounit_pg0->Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_iounit_pg0->Header.ExtPageLength = page_len / 4;
    sas_iounit_pg0->Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT;
    sas_iounit_pg0->NumPhys = MPT3SAS_NUM_PORTS;

    for (i = 0; i < MPT3SAS_NUM_PORTS; i++) {
        uint16_t dev_handle = 0;
        SCSIDevice *dev = mpt3sas_phy_get_device(s, i, &dev_handle);

        sas_iounit_pg0->PhyData[i].Port = i;
        sas_iounit_pg0->PhyData[i].PortFlags = MPI2_SASIOUNIT0_PORTFLAGS_AUTO_PORT_CONFIG;
        sas_iounit_pg0->PhyData[i].PhyFlags = 0x0;
        sas_iounit_pg0->PhyData[i].NegotiatedLinkRate = dev ? MPI2_SAS_NEG_LINK_RATE_6_0 : MPI2_SAS_NEG_LINK_RATE_NEGOTIATION_FAILED; 
        sas_iounit_pg0->PhyData[i].ControllerPhyDeviceInfo = dev ? MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH | MPI2_SAS_DEVICE_INFO_SSP_TARGET | MPI2_SAS_DEVICE_INFO_END_DEVICE : MPI2_SAS_DEVICE_INFO_NO_DEVICE;

        sas_iounit_pg0->PhyData[i].AttachedDevHandle = dev ? dev_handle : 0x0;
        sas_iounit_pg0->PhyData[i].ControllerDevHandle = s->controller_dev_handle;
    }

    if (data) {
        *data = (uint8_t *)sas_iounit_pg0;
    } 

    return page_len; //sizeof(sas_iounit_pg0);
}

static size_t mpt3sas_config_sas_io_unit_1(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasIOUnitPage1_t *sas_iounit_pg1 = NULL;
    uint32_t page_len = 0;
    uint32_t i = 0;

    page_len = offsetof(Mpi2SasIOUnitPage1_t, PhyData) + sizeof(Mpi2SasIOUnit1PhyData_t) * MPT3SAS_NUM_PORTS;
    sas_iounit_pg1 = g_malloc(page_len);
    memset(sas_iounit_pg1, 0, page_len);
    sas_iounit_pg1->Header.PageVersion = MPI2_SASIOUNITPAGE1_PAGEVERSION;
    sas_iounit_pg1->Header.PageNumber = 0x1;
    sas_iounit_pg1->Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_iounit_pg1->Header.ExtPageLength = page_len / 4;
    sas_iounit_pg1->Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT;
    sas_iounit_pg1->NumPhys = MPT3SAS_NUM_PORTS;

    for (i = 0; i < MPT3SAS_NUM_PORTS; i++) {
        sas_iounit_pg1->PhyData[i].Port = i;
        sas_iounit_pg1->PhyData[i].PortFlags = 0x1;
        sas_iounit_pg1->PhyData[i].PhyFlags = 0x0;
        sas_iounit_pg1->PhyData[i].MaxMinLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0 << 4 | MPI2_SAS_NEG_LINK_RATE_1_5;
        sas_iounit_pg1->PhyData[i].ControllerPhyDeviceInfo = MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH | MPI2_SAS_DEVICE_INFO_SSP_TARGET; 
    }
    if (data) {
        *data = (uint8_t *)sas_iounit_pg1;
    } 

    return page_len;
}

static size_t mpt3sas_config_sas_device_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasDevicePage0_t sas_device_pg0;
    uint16_t handle;
    uint8_t phy_id = 0;
    SCSIDevice  *d = NULL;

    //if ((address & MPI2_SAS_DEVICE_PGAD_FORM_MASK) == MPI2_SAS_DEVICE_PGAD_FORM_HANDLE) {
    //    DPRINTF("This is SAS Device 0 PageAddress Format.\n");
    //}

    handle = address & MPI2_SAS_DEVICE_PGAD_HANDLE_MASK;

    phy_id = device_handle_to_phy(handle);
    d = scsi_device_find(&s->bus, 0, phy_id, 0);

    memset(&sas_device_pg0, 0, sizeof(sas_device_pg0));
    sas_device_pg0.Header.PageVersion = MPI2_SASDEVICE0_PAGEVERSION;
    sas_device_pg0.Header.PageNumber = 0x0;
    sas_device_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_device_pg0.Header.ExtPageLength = sizeof(sas_device_pg0) / 4;
    sas_device_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_DEVICE;
    sas_device_pg0.Slot = 0;
    sas_device_pg0.EnclosureHandle = 0x0; //ignore now.
    if (handle == s->controller_dev_handle) {
        sas_device_pg0.SASAddress = s->sas_address;
    } else {
        sas_device_pg0.SASAddress = d ? s->sas_address + phy_id + 1: 0x0;
    }
    sas_device_pg0.ParentDevHandle = d ? s->controller_dev_handle : 0x0;
    sas_device_pg0.PhyNum = phy_id;
    sas_device_pg0.AccessStatus = MPI2_SAS_DEVICE0_ASTATUS_NO_ERRORS;
    sas_device_pg0.DevHandle = handle;
    if (handle == s->controller_dev_handle) {
        sas_device_pg0.DeviceInfo = MPI2_SAS_DEVICE_INFO_SSP_INITIATOR;
    } else {
        sas_device_pg0.DeviceInfo = d ? MPI2_SAS_DEVICE_INFO_SSP_TARGET | MPI2_SAS_DEVICE_INFO_END_DEVICE | MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH: MPI2_SAS_DEVICE_INFO_NO_DEVICE;
    }

    sas_device_pg0.Flags = d ? MPI2_SAS_DEVICE0_FLAGS_DEVICE_PRESENT : 0x0 ;

    if (data) {
        *data = g_malloc(sizeof(sas_device_pg0));
        memcpy(*data, &sas_device_pg0, sizeof(sas_device_pg0));
    }
    return sizeof(sas_device_pg0);
}

static size_t mpt3sas_config_sas_phy_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasPhyPage0_t sas_phy_pg0;
    uint32_t phy_number = address & MPI2_SAS_PHY_PGAD_PHY_NUMBER_MASK; 
    SCSIDevice  *d = scsi_device_find(&s->bus, 0, phy_number, 0);

    memset(&sas_phy_pg0, 0, sizeof(sas_phy_pg0));
    sas_phy_pg0.Header.PageVersion = MPI2_SASPHY0_PAGEVERSION;
    sas_phy_pg0.Header.PageNumber = 0x0;
    sas_phy_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_phy_pg0.Header.ExtPageLength = sizeof(sas_phy_pg0) / 4;
    sas_phy_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_PHY;
    if (d) {
        sas_phy_pg0.OwnerDevHandle = s->controller_dev_handle;
        sas_phy_pg0.AttachedDevHandle = phy_number + 1 + MPT3SAS_NUM_PORTS;
        sas_phy_pg0.AttachedPhyIdentifier = phy_number;
        sas_phy_pg0.HwLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0 << 4 | MPI2_SAS_NEG_LINK_RATE_1_5;
        sas_phy_pg0.NegotiatedLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0;

    }

    if (data) {
        *data = g_malloc(sizeof(sas_phy_pg0));
        memcpy(*data, &sas_phy_pg0, sizeof(sas_phy_pg0));
    }
    return sizeof(sas_phy_pg0);
}

static const MPT3SASConfigPage mpt3sas_config_pages[] = {
    {
        0, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_0,
    },
    {
        11, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_11,
    },
    {
        2, MPI2_CONFIG_PAGETYPE_BIOS,
        mpt3sas_config_bios_2,
    },
    {
        3, MPI2_CONFIG_PAGETYPE_BIOS,
        mpt3sas_config_bios_3,
    },
    {
        8, MPI2_CONFIG_PAGETYPE_IOC,
        mpt3sas_config_ioc_8,
    },
    {
        0, MPI2_CONFIG_PAGETYPE_IO_UNIT,
        mpt3sas_config_io_unit_0,
    },
    {
        1, MPI2_CONFIG_PAGETYPE_IO_UNIT,
        mpt3sas_config_io_unit_1,
    },
    {
        8, MPI2_CONFIG_PAGETYPE_IO_UNIT,
        mpt3sas_config_io_unit_8,
    },
    {
        0, MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT,
        mpt3sas_config_sas_io_unit_0,
    },
    {
        1, MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT,
        mpt3sas_config_sas_io_unit_1,
    },
    {
        0, MPI2_CONFIG_EXTPAGETYPE_SAS_DEVICE,
        mpt3sas_config_sas_device_0,
    },
    {
        0, MPI2_CONFIG_EXTPAGETYPE_SAS_PHY,
        mpt3sas_config_sas_phy_0,
    }
};

static const MPT3SASConfigPage *mpt3sas_find_config_page(int type, int number)
{
    const MPT3SASConfigPage *page;
    int i;

    for (i = 0; i < ARRAY_SIZE(mpt3sas_config_pages); i++) {
        page = &mpt3sas_config_pages[i];
        if (page->type == type && page->number == number)
            return page;
    }
    return NULL;
}

static void mpt3sas_update_interrupt(MPT3SASState *s)
{
    PCIDevice *pci =(PCIDevice *)s;
    
    uint32_t state = s->intr_status & ~(s->intr_mask | MPI2_HIS_SYS2IOC_DB_STATUS);

    trace_mpt3sas_irq_int(state);
    pci_set_irq(pci, !!state);
}

static void mpt3sas_clear_reply_descriptor_int(MPT3SASState *s)
{
    if (s->reply_post_host_index == s->reply_post_ioc_index) {
        trace_mpt3sas_clear_reply_descriptor_int(s->intr_status);
        // host reply post queue is empty
        if (s->intr_status & MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT) {
            s->intr_status &= ~MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
            smp_wmb();
            mpt3sas_update_interrupt(s);
        }
    }
}

static void mpt3sas_interrupt_status_write(MPT3SASState *s)
{
    switch (s->doorbell_state) {
        case DOORBELL_NONE:
        case DOORBELL_WRITE:
            s->intr_status &= ~MPI2_HIS_IOC2SYS_DB_STATUS;
            break;
        case DOORBELL_READ:
            assert(s->intr_status & MPI2_HIS_IOC2SYS_DB_STATUS);
            if (s->doorbell_reply_idx == s->doorbell_reply_size) {
                s->doorbell_state = DOORBELL_NONE;
            }
            break;
        default:
            abort();
    }

    mpt3sas_update_interrupt(s);
}

static void mpt3sas_set_fault(MPT3SASState *s, uint32_t code)
{
    trace_mpt3sas_set_fault(code);
    if ((s->state & MPI2_IOC_STATE_FAULT) == 0)
        s->state = MPI2_IOC_STATE_FAULT | code;
}

static void mpt3sas_post_reply(MPT3SASState *s, MPI2DefaultReply_t *reply, uint8_t reply_flags)
{
    uint32_t reply_address_lo = 0;
    PCIDevice *pci = (PCIDevice *)s;
    Mpi2ReplyDescriptorsUnion_t descriptor;
    uint16_t smid = reply->Reserved1;

    reply->Reserved1 = 0;

    if (s->reply_free_ioc_index == s->reply_free_host_index ||
        (s->reply_post_host_index ==
        (s->reply_post_ioc_index + 1) % s->reply_descriptor_post_queue_depth)) {
        mpt3sas_set_fault(s, MPI2_IOCSTATUS_INSUFFICIENT_RESOURCES);
        trace_mpt3sas_post_reply_error(s->reply_free_ioc_index, s->reply_free_host_index, s->reply_post_ioc_index, s->reply_post_host_index);
        return;
    }

    trace_mpt3sas_reply_free_queue(s->reply_free_ioc_index, s->reply_free_host_index);
    trace_mpt3sas_reply_post_queue(s->reply_post_ioc_index, s->reply_post_host_index);

    // Prepare Reply Descriptor and Generate Interrupt to notify host 
    // a Reply Descriptor was arrived.
    if (reply_flags == MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY) {
        // Get Reply free Queue and Write the data to dest through DMA.
        // Read reply address low 32-bit
        reply_address_lo = ldl_le_pci_dma(pci, s->reply_free_queue_address + s->reply_free_ioc_index * sizeof(uint32_t));

        trace_mpt3sas_reply_frame_address(((hwaddr)s->system_reply_address_hi << 32) | reply_address_lo);
        // write the data to dest address
        pci_dma_write(pci, ((hwaddr)s->system_reply_address_hi << 32) | reply_address_lo,
                reply, MIN(MPT3SAS_REPLY_FRAME_SIZE * 4, reply->MsgLength * 4));

        //Update reply_free_ioc_index
        s->reply_free_ioc_index = (s->reply_free_ioc_index == s->reply_free_queue_depth - 1) ? 0 : s->reply_free_ioc_index + 1;

        descriptor.AddressReply.ReplyFlags = reply_flags;
        descriptor.AddressReply.MSIxIndex = 0;
        descriptor.AddressReply.SMID = smid;
        descriptor.AddressReply.ReplyFrameAddress = reply_address_lo;
    } else if (reply_flags == MPI2_RPY_DESCRIPT_FLAGS_SCSI_IO_SUCCESS) {
        //TODO:
        descriptor.SCSIIOSuccess.ReplyFlags = reply_flags;
        descriptor.SCSIIOSuccess.MSIxIndex = 0;
        descriptor.SCSIIOSuccess.SMID = smid;
        descriptor.SCSIIOSuccess.TaskTag = 0x0; //TODO, this value is assigned by IOC
    }
    //Write reply descriptor to reply post queue.1
    pci_dma_write(pci, s->reply_descriptor_post_queue_address + s->reply_post_ioc_index * sizeof(uint64_t), &descriptor, sizeof(descriptor));
    s->reply_post_ioc_index = (s->reply_post_ioc_index == s->reply_descriptor_post_queue_depth - 1) ? 0 : s->reply_post_ioc_index + 1;

    trace_mpt3sas_post_reply_completed(smid);
    //Generate interrupt
    s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
    if (s->doorbell_state == DOORBELL_WRITE) {
        s->doorbell_state = DOORBELL_NONE;
        s->intr_status |= MPI2_HIS_IOC2SYS_DB_STATUS;
    }
    mpt3sas_update_interrupt(s);
}

static void mpt3sas_reply(MPT3SASState *s, MPI2DefaultReply_t *reply)
{
    if (s->doorbell_state == DOORBELL_WRITE) {
        s->doorbell_state = DOORBELL_READ;
        s->doorbell_reply_idx = 0;
        s->doorbell_reply_size = reply->MsgLength * 2;
        memcpy(s->doorbell_reply, reply, s->doorbell_reply_size * 2);
        s->intr_status |= MPI2_HIS_IOC2SYS_DB_STATUS;
        smp_wmb();
        mpt3sas_update_interrupt(s);
    }
}

static void mpt3sas_cancel_notify(Notifier *notifier, void *data)
{
    MPT3SASCancelNotifier *n = container_of(notifier, MPT3SASCancelNotifier, notifier);

    if (++n->reply->TerminationCount == n->reply->IOCLogInfo) {
        n->reply->IOCLogInfo = 0;
        mpt3sas_post_reply(n->s, (MPI2DefaultReply_t *)n->reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
        g_free(n->reply);
    }
    g_free(n);
}

static void mpt3sas_event_sas_topology_change_list(MPT3SASState *s)
{
    Mpi2EventDataSasTopologyChangeList_t *stcl = NULL;
    uint8_t i = 0;
    Mpi2EventNotificationReply_t *reply = NULL;
    uint32_t event_data_length = 0;

    event_data_length = offsetof(Mpi2EventDataSasTopologyChangeList_t, PHY) + sizeof(MPI2_EVENT_SAS_TOPO_PHY_ENTRY) * MPT3SAS_NUM_PORTS;

    stcl = g_malloc(event_data_length);
    memset(stcl, 0, event_data_length);
    stcl->EnclosureHandle = 0x0;
    stcl->ExpanderDevHandle = 0x0;
    stcl->NumPhys = 0x0;
    stcl->NumEntries = MPT3SAS_NUM_PORTS; //TODO
    stcl->StartPhyNum = 0x0;
    stcl->ExpStatus =  MPI2_EVENT_SAS_TOPO_ES_NO_EXPANDER; //Currently not support expander
    stcl->PhysicalPort = 0x0; //ignore
    for (i = 0; i < stcl->NumEntries; i++) {
        uint16_t dev_handle = 0;
        SCSIDevice *dev = mpt3sas_phy_get_device(s, i, &dev_handle);

        stcl->PHY[i].AttachedDevHandle = dev ? dev_handle : 0x0;
        stcl->PHY[i].LinkRate = dev ? MPI25_EVENT_SAS_TOPO_LR_RATE_12_0 << MPI2_EVENT_SAS_TOPO_LR_CURRENT_SHIFT : MPI2_EVENT_SAS_TOPO_LR_NEGOTIATION_FAILED;
        stcl->PHY[i].PhyStatus = dev ? MPI2_EVENT_SAS_TOPO_RC_TARG_ADDED : MPI2_EVENT_SAS_TOPO_PHYSTATUS_VACANT | MPI2_EVENT_SAS_TOPO_RC_NO_CHANGE;
    }

    reply = g_malloc(sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    memset(reply, 0, sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    reply->EventDataLength = event_data_length / 4;
    reply->AckRequired = 1;
    reply->MsgLength = (sizeof(reply) + event_data_length) / 4;
    reply->Function = MPI2_FUNCTION_EVENT_NOTIFICATION;
    reply->Event = MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST;
    reply->Reserved1 = 0;
    memcpy(reply->EventData, stcl, event_data_length);
    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
    free(reply);
    free(stcl);
}

static void mpt3sas_set_event(MPT3SASState *s, uint16_t event_type)
{
    s->event = event_type;
}

/*
 * Send event to host
 */
static void mpt3sas_send_event(void *opaque)
{
    MPT3SASState *s = (MPT3SASState *)opaque;

    trace_mpt3sas_send_event(s->event);
    switch (s->event) {
        case MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST:
            mpt3sas_event_sas_topology_change_list(s);
            break;
        default:
            trace_mpt3sas_unhandled_event(s->event);
            break;
    }
}

static void mpt3sas_handle_ioc_facts(MPT3SASState *s,Mpi2IOCFactsRequest_t *req)
{
    Mpi2IOCFactsReply_t reply;

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
    reply.IOCCapabilities = MPI2_IOCFACTS_CAPABILITY_TLR | MPI2_IOCFACTS_CAPABILITY_EEDP | MPI2_IOCFACTS_CAPABILITY_SNAPSHOT_BUFFER | MPI2_IOCFACTS_CAPABILITY_DIAG_TRACE_BUFFER | MPI2_IOCFACTS_CAPABILITY_TASK_SET_FULL_HANDLING;
    reply.FWVersion.Struct.Major = 0xd;
    reply.FWVersion.Struct.Minor = 0x0;
    reply.FWVersion.Struct.Unit = 0x0;
    reply.FWVersion.Struct.Dev = 0x0;
    reply.IOCRequestFrameSize = MPT3SAS_REQUEST_FRAME_SIZE;
    reply.IOCMaxChainSegmentSize = 0;
    reply.MaxInitiators = 1;
    reply.MaxTargets = s->max_devices;
    reply.MaxSasExpanders = 0x1;
    reply.MaxEnclosures = 0x1;
    reply.ProtocolFlags = MPI2_IOCFACTS_PROTOCOL_SCSI_INITIATOR | MPI2_IOCFACTS_PROTOCOL_SCSI_TARGET;
    reply.HighPriorityCredit = 124;
    reply.MaxReplyDescriptorPostQueueDepth = MPT3SAS_MAX_REPLY_DESCRIPTOR_QUEUE_DEPTH;
    reply.ReplyFrameSize = MPT3SAS_REPLY_FRAME_SIZE;
    reply.MaxVolumes = 0x0;
    reply.MaxDevHandle = s->max_devices;
    reply.MaxPersistentEntries = 0x0;
    reply.CurrentHostPageSize = 0x0;

    mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_port_facts(MPT3SASState *s, Mpi2PortFactsRequest_t *req)
{
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

    s->who_init = req->WhoInit;
    s->host_page_size = req->HostPageSize;
    s->host_msix_vectors = req->HostMSIxVectors;
    s->system_request_frame_size = req->SystemRequestFrameSize;
    s->reply_descriptor_post_queue_depth = req->ReplyDescriptorPostQueueDepth;
    s->reply_free_queue_depth = req->ReplyFreeQueueDepth;
    s->sense_buffer_address_hi = req->SenseBufferAddressHigh;
    s->system_reply_address_hi = req->SystemReplyAddressHigh;
    s->system_request_frame_base_address = (hwaddr)req->SystemRequestFrameBaseAddress;
    s->reply_descriptor_post_queue_address = (hwaddr)req->ReplyDescriptorPostQueueAddress;
    s->reply_free_queue_address = (hwaddr)req->ReplyFreeQueueAddress;
    
    trace_mpt3sas_ioc_init(req->SystemRequestFrameSize, req->ReplyDescriptorPostQueueDepth, req->ReplyFreeQueueDepth, req->SystemRequestFrameBaseAddress, req->ReplyDescriptorPostQueueAddress, req->ReplyFreeQueueAddress, req->SystemReplyAddressHigh);

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

static void mpt3sas_handle_event_notification(MPT3SASState *s, uint16_t smid, Mpi2EventNotificationRequest_t *req)
{
    Mpi2EventNotificationReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.EventDataLength = sizeof(reply.EventData) / 4;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.Event = MPI2_EVENT_EVENT_CHANGE;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;

    // use reply.Reserved1 to store smid
    reply.Reserved2 = smid;

    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
}

static void mpt3sas_handle_config(MPT3SASState *s, uint16_t smid, Mpi2ConfigRequest_t *req)
{
    PCIDevice *pci = PCI_DEVICE(s);
    Mpi2ConfigReply_t reply;
    const MPT3SASConfigPage *page;
    uint8_t type = 0;
    size_t length = 0;
    uint32_t dmalen = 0;
    uint64_t pa = 0;
    uint8_t *data = NULL;

    trace_mpt3sas_handle_config(req->Action, req->Header.PageType,
            req->Header.PageNumber, req->Header.PageLength, req->Header.PageVersion, 
            req->ExtPageType, req->ExtPageLength, req->PageAddress);

    memset(&reply, 0, sizeof(reply));
    reply.Action = req->Action;
    reply.SGLFlags = req->SGLFlags;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.Header.PageType = req->Header.PageType;
    reply.Header.PageNumber = req->Header.PageNumber;
    reply.Header.PageLength = req->Header.PageLength;
    reply.Header.PageVersion = req->Header.PageVersion;

    reply.Reserved1 = smid;

    type = req->Header.PageType;
    if (type == MPI2_CONFIG_PAGETYPE_EXTENDED) {
        type = req->ExtPageType;
        if (type <= MPI2_CONFIG_PAGETYPE_MASK) {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_TYPE;
            goto out;
        }
        reply.ExtPageType = req->ExtPageType;
    }

    page = mpt3sas_find_config_page(type, req->Header.PageNumber);

    switch(req->Action) {
        case MPI2_CONFIG_ACTION_PAGE_DEFAULT:
        case MPI2_CONFIG_ACTION_PAGE_HEADER:
        case MPI2_CONFIG_ACTION_PAGE_READ_NVRAM:
        case MPI2_CONFIG_ACTION_PAGE_READ_CURRENT:
        case MPI2_CONFIG_ACTION_PAGE_WRITE_CURRENT:
        case MPI2_CONFIG_ACTION_PAGE_WRITE_NVRAM:
            break;
        default:
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_ACTION;
            goto out;
    }

    if (!page) {
        // Just find out the reason why fails to find the page
        page = mpt3sas_find_config_page(type, 1);
        if (page) {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_PAGE;
        } else {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_TYPE;
        }
        goto out;
    }

    if (req->Action == MPI2_CONFIG_ACTION_PAGE_DEFAULT ||
        req->Action == MPI2_CONFIG_ACTION_PAGE_HEADER) {
        length = page->mpt_config_build(s, NULL, req->PageAddress);
        if ((ssize_t)length < 0) {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_PAGE;
            goto out;
        } else {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_CANT_COMMIT;
            goto done;
        }
    }

    if (req->Action == MPI2_CONFIG_ACTION_PAGE_WRITE_CURRENT ||
        req->Action == MPI2_CONFIG_ACTION_PAGE_WRITE_NVRAM) {
        length = page->mpt_config_build(s, NULL, req->PageAddress);
        if ((ssize_t)length < 0) {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_PAGE;
        } else {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_CANT_COMMIT;
        }
        goto out;
    }

    assert(req->ChainOffset == 0);

    //determin SGL type
    if ((req->SGLFlags & MPI2_SGLFLAGS_SGL_TYPE_MASK) == MPI2_SGLFLAGS_SGL_TYPE_MPI) {
        uint32_t flags_and_length = req->PageBufferSGE.MpiSimple.FlagsLength;
        dmalen = flags_and_length & MPI2_SGE_LENGTH_MASK;
        if (flags_and_length & MPI2_SGE_FLAGS_64_BIT_ADDRESSING) {
            pa = req->PageBufferSGE.MpiSimple.u.Address64;
        } else {
            pa = req->PageBufferSGE.MpiSimple.u.Address32;
        }
    } else if ((req->SGLFlags & MPI2_SGLFLAGS_SGL_TYPE_MASK) == MPI2_SGLFLAGS_SGL_TYPE_IEEE64) {
        dmalen = req->PageBufferSGE.IeeeSimple.Simple64.Length;
        pa = req->PageBufferSGE.IeeeSimple.Simple64.Address;
    }

    if (dmalen == 0) {
        length = page->mpt_config_build(s, NULL, req->PageAddress);
        if ((ssize_t)length < 0) {
            reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_PAGE;
            goto out;
        } else {
            goto done;
        }
    }

    length = page->mpt_config_build(s, &data, req->PageAddress);

    if (0){
        uint8_t i = 0;
        for (i = 0; i < length; i++) {
            if (i && (i % 8) == 0) {
                qemu_log_mask(LOG_TRACE, "\n");
            }

            qemu_log_mask(LOG_TRACE, "%02x ", data[i]);
        }
        qemu_log_mask(LOG_TRACE, "\n");
    }

    if ((ssize_t)length < 0) {
        reply.IOCStatus = MPI2_IOCSTATUS_CONFIG_INVALID_PAGE;
        goto out;
    } else {
        assert(data[2] == page->number);
        reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;
        pci_dma_write(pci, pa, data, MIN(length, dmalen));
        goto done;
    }

    abort();

done:
    if (type > MPI2_CONFIG_PAGETYPE_MASK) {
        reply.ExtPageLength = length / 4;
        reply.ExtPageType = req->ExtPageType;
    } else {
        reply.Header.PageLength = length / 4;
    }

out:
    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
    g_free(data);
    return ;
}

static void mpt3sas_handle_port_enable(MPT3SASState *s, uint16_t smid, Mpi2PortEnableRequest_t *req)
{
    Mpi2PortEnableReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.PortFlags = req->PortFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;

    reply.Reserved4 = smid;

    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
}

static int mpt3sas_build_ieee_sgl(MPT3SASState *s, MPT3SASRequest *req, hwaddr addr)
{
    PCIDevice *pci = (PCIDevice *)s;
    //uint32_t left = req->scsi_io.DataLength;
    uint8_t chain_offset = req->scsi_io.ChainOffset;
    hwaddr sgaddr = addr + req->scsi_io.SGLOffset0 * sizeof(uint32_t);
    hwaddr next_chain_addr = addr + chain_offset * sizeof(uint32_t) * 4;
    uint32_t sglen = 0;

    pci_dma_sglist_init(&req->qsg, pci, 4);
    trace_mpt3sas_ieee_sgl_build_table(req, req->scsi_io.DataLength, chain_offset);
    for (;;) {
        uint8_t flags;
        dma_addr_t addr, len = 0;

        flags = ldl_le_pci_dma(pci, sgaddr + 12) >> 24;

        if ((flags & MPI2_IEEE_SGE_FLAGS_ELEMENT_TYPE_MASK) == MPI2_IEEE_SGE_FLAGS_SIMPLE_ELEMENT) {
            // This is a Simple Element
            addr = ldl_le_pci_dma(pci, sgaddr) |
               (hwaddr)ldl_le_pci_dma(pci, sgaddr + 4) << 32;
            len = ldl_le_pci_dma(pci, sgaddr + 8);

            trace_mpt3sas_ieee_sgl_add_simple_element(addr, len, flags);
            qemu_sglist_add(&req->qsg, addr, len);
            //Update sgaddr
            sgaddr += 16;
        } else if ((flags & MPI2_IEEE_SGE_FLAGS_ELEMENT_TYPE_MASK) == MPI2_IEEE_SGE_FLAGS_CHAIN_ELEMENT) {
            //Read chain offset offset
            chain_offset = (ldl_le_pci_dma(pci, sgaddr + 12) >> 16) & 0xff;

            sgaddr = ldl_le_pci_dma(pci, next_chain_addr) | 
                (hwaddr)ldl_le_pci_dma(pci, next_chain_addr + 4) << 32;

            sglen = ldl_le_pci_dma(pci, next_chain_addr + 8);
            trace_mpt3sas_ieee_sgl_add_chain_element(sgaddr, sglen, flags);

            //next chain element addr
            next_chain_addr = sgaddr + chain_offset * sizeof(uint32_t) * 4;
        }

       if ((flags & 0xc0) ==  MPI25_IEEE_SGE_FLAGS_END_OF_LIST)
            break;
    }

    return 0;
}

static void __attribute__((unused)) dump_cdb(uint8_t *buf)
{
    int length = 8;

    switch (buf[0]) {
        case 0x00:
        case 0x1a:
        case 0x12:
            length = 6;
            break;
        case 0x5a:
        case 0x28:
        case 0x2a:
        case 0xa3:
            length = 10;
            break;
        case 0x9e:
        case 0x85:
            length = 16;
            break;
    }

    switch (length) {
        case 10:
            trace_mpt3sas_scsi_command_cdb10(buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                    buf[6], buf[7], buf[8], buf[9]);
            break;
        default:
            break;
    }
}

static int mpt3sas_handle_scsi_io_request(MPT3SASState *s, uint16_t smid, Mpi25SCSIIORequest_t *req, hwaddr addr)
{

    MPT3SASRequest *mpt3sas_req = NULL;
    int status;
    SCSIDevice *sdev;
    Mpi2SCSIIOReply_t reply;

    trace_mpt3sas_scsi_io_request(req->DevHandle, req->LUN[1], req->DataLength, req->Control);
    //mpt3sas_print_scsi_devices(&s->bus);
    status = mpt3sas_scsi_device_find(s, req->DevHandle, req->LUN, &sdev);
    if (status) {
        goto bad;
    }

    mpt3sas_req = g_new0(MPT3SASRequest, 1);
    QTAILQ_INSERT_TAIL(&s->pending, mpt3sas_req, next);
    mpt3sas_req->scsi_io = *req;
    mpt3sas_req->dev = s;
    mpt3sas_req->smid = smid;
    trace_mpt3sas_scsi_io_command_info(req, req->CDB.CDB32[0], smid);
    dump_cdb(req->CDB.CDB32);

    status = mpt3sas_build_ieee_sgl(s, mpt3sas_req, addr);
    if (status) {
        goto free_bad;
    }

    if (mpt3sas_req->qsg.size < req->DataLength) {
        status = MPI2_IOCSTATUS_INVALID_SGL;
        DPRINTF("%s:%d qsg size 0x%x request data length 0x%x\n",
                __func__, __LINE__, (uint32_t)mpt3sas_req->qsg.size, req->DataLength);
        goto free_bad;
    }

    mpt3sas_req->sreq = scsi_req_new(sdev, 0, req->LUN[1], req->CDB.CDB32, mpt3sas_req);

    if (mpt3sas_req->sreq->cmd.xfer > req->DataLength)
        goto overrun;

    switch (req->Control & MPI2_SCSIIO_CONTROL_DATADIRECTION_MASK) {
    case MPI2_SCSIIO_CONTROL_NODATATRANSFER:
        if (mpt3sas_req->sreq->cmd.mode != SCSI_XFER_NONE)
            goto overrun;
        break;
    case MPI2_SCSIIO_CONTROL_WRITE:
        if (mpt3sas_req->sreq->cmd.mode != SCSI_XFER_TO_DEV)
            goto overrun;
        break;
    case MPI2_SCSIIO_CONTROL_READ:
        if (mpt3sas_req->sreq->cmd.mode != SCSI_XFER_FROM_DEV)
            goto overrun;
        break;
    }

    if (scsi_req_enqueue(mpt3sas_req->sreq)) {
        scsi_req_continue(mpt3sas_req->sreq);
    }

    return 0;

overrun:
    status = MPI2_IOCSTATUS_SCSI_DATA_OVERRUN;
free_bad:
    mpt3sas_free_request(mpt3sas_req);
bad:
    memset(&reply, 0, sizeof(reply));
    reply.DevHandle = req->DevHandle;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.MsgFlags =  req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.SCSIState = MPI2_SCSI_STATE_NO_SCSI_STATUS;
    reply.IOCStatus = status;

    reply.Reserved3 = smid;
    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
    trace_mpt3sas_scsi_io_error(status);
    return status;
}

static void mpt3sas_handle_event_ack(MPT3SASState *s, uint16_t smid, Mpi2EventAckRequest_t *req)
{
    Mpi2EventAckReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.MsgLength = sizeof(reply) / 4;
    reply.VF_ID = req->VF_ID;
    reply.VP_ID = req->VP_ID;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;

    reply.Reserved4 = smid;

    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
}

static void mpt3sas_handle_scsi_task_management(MPT3SASState *s, uint16_t smid, Mpi2SCSITaskManagementRequest_t *req)
{
    Mpi2SCSITaskManagementReply_t reply;
    Mpi2SCSITaskManagementReply_t *reply_async = NULL;
    int status = 0;
    int count = 1;
    SCSIDevice *sdev = NULL;
    SCSIRequest *r, *next;
    BusChild *kid;

    trace_mpt3sas_scsi_task_management(req->TaskType, req->TaskMID);
    memset(&reply, 0, sizeof(reply));
    reply.DevHandle = req->DevHandle;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.TaskType = req->TaskType;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;

    reply.Reserved2 = smid;

    switch (req->TaskType) {
        case MPI2_SCSITASKMGMT_TASKTYPE_ABORT_TASK:
        case MPI2_SCSITASKMGMT_TASKTYPE_QUERY_TASK:
            status = mpt3sas_scsi_device_find(s, req->DevHandle, req->LUN, &sdev);
            if (status) {
                reply.IOCStatus = status;
                goto out;
            }

            QTAILQ_FOREACH_SAFE(r, &sdev->requests, next, next) {
                MPT3SASRequest *cmd_req = r->hba_private;
                if (cmd_req && cmd_req->smid == req->TaskMID) {
                    break;
                }
            }

            if (r) {
                if (req->TaskType == MPI2_SCSITASKMGMT_TASKTYPE_QUERY_TASK) {
                    reply.ResponseCode = MPI2_SCSITASKMGMT_RSP_TM_SUCCEEDED;
                } else {
                    MPT3SASCancelNotifier *notifier = NULL;
                    reply_async = g_memdup(&reply, sizeof(Mpi2SCSITaskManagementReply_t));
                    reply_async->IOCLogInfo = INT_MAX;

                    notifier = g_new(MPT3SASCancelNotifier, 1);
                    notifier->s = s;
                    notifier->reply = reply_async;
                    notifier->notifier.notify = mpt3sas_cancel_notify;
                    scsi_req_cancel_async(r, &notifier->notifier);
                    goto reply_maybe_async;
                }
            }
            break;
        case MPI2_SCSITASKMGMT_TASKTYPE_ABRT_TASK_SET:
        case MPI2_SCSITASKMGMT_TASKTYPE_CLEAR_TASK_SET:
            status = mpt3sas_scsi_device_find(s, req->DevHandle, req->LUN, &sdev);
            if (status) {
                reply.IOCStatus = status;
                goto out;
            }

            reply_async = g_memdup(&reply, sizeof(Mpi2SCSITaskManagementReply_t));
            reply_async->IOCLogInfo = INT_MAX;
            count = 0;
            QTAILQ_FOREACH_SAFE(r, &sdev->requests, next, next) {
                if (r->hba_private) {
                    MPT3SASCancelNotifier *notifier = NULL;

                    count++;
                    notifier = g_new(MPT3SASCancelNotifier, 1);
                    notifier->s = s;
                    notifier->reply = reply_async;
                    notifier->notifier.notify = mpt3sas_cancel_notify;
                    scsi_req_cancel_async(r, &notifier->notifier);
                }
            }
reply_maybe_async:
            if (reply_async->TerminationCount < count) {
                reply_async->IOCLogInfo = count;
                return;
            }
            g_free(reply_async);
            reply.TerminationCount = count;
            break;
        case MPI2_SCSITASKMGMT_TASKTYPE_LOGICAL_UNIT_RESET:
            status = mpt3sas_scsi_device_find(s, req->DevHandle, req->LUN, &sdev);
            if (status) {
                reply.IOCStatus = status;
                goto out;
            }
            qdev_reset_all(&sdev->qdev);
            break;
        case MPI2_SCSITASKMGMT_TASKTYPE_TARGET_RESET:
            QTAILQ_FOREACH(kid, &s->bus.qbus.children, sibling) {
                sdev = SCSI_DEVICE(kid->child);
                if (sdev->channel == 0 && sdev->id == device_handle_to_phy(req->DevHandle)) {
                    qdev_reset_all(kid->child);
                }
            }
            break;
        default:
            reply.ResponseCode = MPI2_SCSITASKMGMT_RSP_TM_NOT_SUPPORTED;
            break;
    }
out:
    mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);

}

static void mpt3sas_handle_message(MPT3SASState *s, MPI2RequestHeader_t *req)
{
    trace_mpt3sas_handle_message(req->Function);
    switch (req->Function) {
        case MPI2_FUNCTION_IOC_INIT:
            mpt3sas_handle_ioc_init(s, (Mpi2IOCInitRequest_t *)req);
            break;
        case MPI2_FUNCTION_IOC_FACTS:
            mpt3sas_handle_ioc_facts(s, (Mpi2IOCFactsRequest_t *)req);
            break;
        case MPI2_FUNCTION_PORT_FACTS:
            mpt3sas_handle_port_facts(s, (Mpi2PortFactsRequest_t *)req);
            break;
        default:
            DPRINTF("%s:%d unknown function 0x%x\n", __func__, __LINE__, req->Function);
            mpt3sas_set_fault(s, MPI2_IOCSTATUS_INVALID_FUNCTION);
            break;
    }
}

static void mpt3sas_soft_reset(MPT3SASState *s)
{
    uint32_t save_mask;

    trace_mpt3sas_soft_reset();
    save_mask = s->intr_mask;
    s->intr_mask = MPI2_HIM_RESET_IRQ_MASK | MPI2_HIM_DIM | MPI2_HIM_DIM;
    mpt3sas_update_interrupt(s);

    qbus_reset_all(&s->bus.qbus);
    s->intr_status = 0;
    s->intr_mask = save_mask;
    s->reply_free_ioc_index = 0;
    s->reply_free_host_index = 0;
    s->reply_post_ioc_index = 0;
    s->reply_post_host_index = 0;

    s->request_descriptor_post_head = 0;
    s->request_descriptor_post_tail = 0;
    memset(s->request_descriptor_post, 0, ARRAY_SIZE(s->request_descriptor_post));

    s->controller_dev_handle = 0x1;
    //s->sas_address = 0x51866da05a753a00;
    qemu_bh_cancel(s->request_bh);
    qemu_bh_cancel(s->event_handler);
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

static void mpt3sas_handle_request(MPT3SASState *s)
{
    PCIDevice *pci = (PCIDevice *)s;
    uint8_t req[MPT3SAS_REQUEST_FRAME_SIZE * 4];
    MPI2RequestHeader_t *hdr = (MPI2RequestHeader_t *)req;
    hwaddr addr;
    int size;
    uint64_t mpi_request_descriptor = 0;
    uint8_t request_flags = 0;
    //uint8_t msix_index = 0;
    uint16_t smid = 0;
    const char *request_desc = NULL;

    memset(req, 0, sizeof(req));
    mpi_request_descriptor = s->request_descriptor_post[s->request_descriptor_post_head++];
    s->request_descriptor_post_head %= ARRAY_SIZE(s->request_descriptor_post);
    request_flags = mpi_request_descriptor & 0xff;
    //msix_index = (mpi_request_descriptor >> 8) & 0xff;
    smid = (mpi_request_descriptor >> 16) & 0xffff;
    switch (request_flags) {
        case MPI2_REQ_DESCRIPT_FLAGS_SCSI_IO:
            request_desc = "SCSI IO request";
            break;
        case MPI2_REQ_DESCRIPT_FLAGS_SCSI_TARGET:
            request_desc = "SCSI TARGET request";
            break;
        case MPI2_REQ_DESCRIPT_FLAGS_HIGH_PRIORITY:
            request_desc = "HIGH PRIORITY request";
            break;
        case MPI2_REQ_DESCRIPT_FLAGS_DEFAULT_TYPE:
            request_desc = "DEFAULT request";
            break;
        case MPI2_REQ_DESCRIPT_FLAGS_RAID_ACCELERATOR:
            request_desc = "RAID ACCELERATOR request";
            break;
        case MPI25_REQ_DESCRIPT_FLAGS_FAST_PATH_SCSI_IO:
            request_desc = "FAST PATH SCSI IO request";
            break;
        default:
            request_desc = "UNSUPPORTED request";
            break;

    }

    trace_mpt3sas_handle_request(request_desc, smid);
    addr = s->system_request_frame_base_address + (s->system_request_frame_size * 4) * smid;

    // Read request header from system request message frames queue
    pci_dma_read(pci, addr, req, sizeof(MPI2RequestHeader_t));

    if (hdr->Function < ARRAY_SIZE(mpi2_request_sizes) && 
        mpi2_request_sizes[hdr->Function]) {
        size = mpi2_request_sizes[hdr->Function];
        assert(size <= MPT3SAS_REQUEST_FRAME_SIZE * 4);
        pci_dma_read(pci, addr + sizeof(hdr), &req[sizeof(hdr)],
                size - sizeof(hdr));
        if (0) {
            uint32_t i = 0;
            //DEBUG information
            DPRINTF("Request (0x%lx/0x%x):\n", addr, size);
            DPRINTF("-------------------\n");
            for (i = 0; i < size; i++) {
                if (i && (i % 8 == 0)) {
                    qemu_log_mask(LOG_TRACE, "\n");
                }
                qemu_log_mask(LOG_TRACE, "%02x ", req[i]);
            }
            qemu_log_mask(LOG_TRACE, "\n");
        }
    }

    switch (hdr->Function) {
        case MPI2_FUNCTION_EVENT_NOTIFICATION:
            mpt3sas_handle_event_notification(s, smid, (Mpi2EventNotificationRequest_t *)req);
            break;
        case MPI2_FUNCTION_SCSI_IO_REQUEST:
            mpt3sas_handle_scsi_io_request(s, smid, (Mpi25SCSIIORequest_t *)req, addr);
            break;
        case MPI2_FUNCTION_PORT_ENABLE:
            mpt3sas_handle_port_enable(s, smid, (Mpi2PortEnableRequest_t *)req);
            sleep(5);
            mpt3sas_set_event(s, MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST);
            qemu_bh_schedule(s->event_handler);
            break;
        case MPI2_FUNCTION_CONFIG:
            mpt3sas_handle_config(s, smid, (Mpi2ConfigRequest_t *)req);
            break;
        case MPI2_FUNCTION_EVENT_ACK:
            mpt3sas_handle_event_ack(s, smid, (Mpi2EventAckRequest_t *)req);
            break;
        case MPI2_FUNCTION_SCSI_TASK_MGMT:
            mpt3sas_handle_scsi_task_management(s, smid, (Mpi2SCSITaskManagementRequest_t *)req);
            break;
        default:
            mpt3sas_handle_message(s, (MPI2RequestHeader_t *)req);
    }
}

static void mpt3sas_handle_requests(void *opaque)
{
    MPT3SASState *s = opaque;

    if (s->state != MPI2_IOC_STATE_OPERATIONAL) {
        mpt3sas_set_fault(s, MPI2_IOCSTATUS_INVALID_STATE);
        return;
    }

    while (s->request_descriptor_post_head != s->request_descriptor_post_tail) {
        mpt3sas_handle_request(s);
    }
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

            retval |= MPI2_DOORBELL_USED;
            if (s->doorbell_reply_idx < s->doorbell_reply_size) {
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
            mpt3sas_update_interrupt(s);
            break;
        default:
            DPRINTF("%s:%d unknown doorbell function 0x%x\n", __func__, __LINE__, function);
            break;
    }
}

static uint64_t mpt3sas_mmio_read(void *opaque, hwaddr addr,
        unsigned size)
{
    uint32_t ret = 0;

    MPT3SASState *s = opaque;

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

    trace_mpt3sas_mmio_read(register_description[addr & ~ 3], size, ret);

    return ret;
}

static void mpt3sas_mmio_write(void *opaque, hwaddr addr,
        uint64_t val, unsigned size)
{
    MPT3SASState *s = opaque;

    trace_mpt3sas_mmio_write(register_description[addr], size, val);

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
            s->reply_post_host_index = val & MPI2_REPLY_POST_HOST_INDEX_MASK;
            //clear ReplyDescriptorInterrupt
            mpt3sas_clear_reply_descriptor_int(s);
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
        {
            if (s->request_descriptor_post_head ==
                (s->request_descriptor_post_tail + 1) % ARRAY_SIZE(s->request_descriptor_post)) {
                DPRINTF("%s:%d Request descriptor post queue is full.\n", __func__, __LINE__);
                mpt3sas_set_fault(s, MPI2_IOCSTATUS_INSUFFICIENT_RESOURCES);
            } else {
                s->request_descriptor_post[s->request_descriptor_post_tail++] = val;
                s->request_descriptor_post_tail %= ARRAY_SIZE(s->request_descriptor_post);
                qemu_bh_schedule(s->request_bh);
            }
            break;
        }
        case MPI2_REQUEST_DESCRIPTOR_POST_HIGH_OFFSET:
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

static QEMUSGList *mpt3sas_get_ieee_sg_list(SCSIRequest *sreq)
{
    MPT3SASRequest *req = sreq->hba_private;
    return &req->qsg;
}

static void mpt3sas_command_complete(SCSIRequest *sreq,
        uint32_t status, size_t resid)
{
    MPT3SASRequest *req = sreq->hba_private;
    MPT3SASState *s = req->dev;
    uint8_t sense_buf[SCSI_SENSE_BUF_SIZE];
    uint8_t sense_len;

    hwaddr sense_buffer_addr = (hwaddr)req->dev->sense_buffer_address_hi << 32 | req->scsi_io.SenseBufferLowAddress;

    sense_len = scsi_req_get_sense(sreq, sense_buf, SCSI_SENSE_BUF_SIZE);
    if (sense_len > 0) {
        trace_mpt3sas_scsi_io_command_sense_data(sense_buf[0] & 0x7f, sense_buf[1] & 0xf, sense_buf[2], sense_buf[3]);
        pci_dma_write(PCI_DEVICE(s), sense_buffer_addr, sense_buf,
                MIN(req->scsi_io.SenseBufferLength, sense_len));
    }

    if (sreq->status != GOOD || resid || req->dev->doorbell_state == DOORBELL_WRITE) {
        Mpi2SCSIIOReply_t reply;

        memset(&reply, 0, sizeof(reply));
        reply.DevHandle = req->scsi_io.DevHandle;
        reply.MsgLength = sizeof(reply) / 4;
        reply.Function = req->scsi_io.Function;
        reply.MsgFlags = req->scsi_io.MsgFlags;
        reply.VP_ID = req->scsi_io.VP_ID;
        reply.VF_ID = req->scsi_io.VF_ID;
        reply.SCSIStatus = sreq->status;
        reply.TaskTag = 0; //TODO

        reply.Reserved3 = req->smid;

        if (sreq->status == GOOD) {
            reply.TransferCount = req->scsi_io.DataLength - resid;
            if (resid) {
                reply.IOCStatus = MPI2_IOCSTATUS_SCSI_DATA_UNDERRUN;
            }
        } else {
            reply.SCSIState = MPI2_SCSI_STATE_AUTOSENSE_VALID;
            reply.SenseCount = sense_len;
            reply.IOCStatus = MPI2_IOCSTATUS_SCSI_DATA_UNDERRUN;
        }

        trace_mpt3sas_scsi_io_command_error(req, req->scsi_io.CDB.CDB32[0], req->smid, sreq->status);
        mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY);
    } else {
        Mpi2SCSIIOReply_t reply;
        uint32_t lba = 0x0;

        memset(&reply, 0, sizeof(reply));
        reply.DevHandle = req->scsi_io.DevHandle;
        reply.MsgLength = sizeof(reply) / 4;
        reply.Function = req->scsi_io.Function;
        reply.MsgFlags = req->scsi_io.MsgFlags;
        reply.VP_ID = req->scsi_io.VP_ID;
        reply.VF_ID = req->scsi_io.VF_ID;
        reply.SCSIStatus = sreq->status;
        reply.TaskTag = 0; //TODO
        reply.TransferCount = req->scsi_io.DataLength;

        reply.Reserved3 = req->smid;

        if (req->scsi_io.CDB.CDB32[0] == 0x28 || req->scsi_io.CDB.CDB32[0] == 0x2a) {
            lba = (uint32_t)req->scsi_io.CDB.CDB32[2] << 24 |
                  (uint32_t)req->scsi_io.CDB.CDB32[3] << 16 |
                  (uint32_t)req->scsi_io.CDB.CDB32[4] << 8 |
                  (uint32_t)req->scsi_io.CDB.CDB32[5];
            //dump_cdb(req->scsi_io.CDB.CDB32);
        }

        trace_mpt3sas_scsi_io_command_succeed(req, req->scsi_io.CDB.CDB32[0], lba);

        mpt3sas_post_reply(s, (MPI2DefaultReply_t *)&reply, MPI2_RPY_DESCRIPT_FLAGS_SCSI_IO_SUCCESS);
    }
    
    mpt3sas_free_request(req);
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

    .get_sg_list = mpt3sas_get_ieee_sg_list,
    .complete = mpt3sas_command_complete,
    .cancel = mpt3sas_request_cancelled,
//    .save_request = mpt3sas_save_request,
//    .load_request = mpt3sas_load_request,

};

static void mpt3sas_scsi_init(PCIDevice *dev, Error **errp)
{
    DeviceState *d = DEVICE(dev);
    MPT3SASState *s = MPT3SAS(dev);

    dev->config[PCI_LATENCY_TIMER] = 0;
    dev->config[PCI_INTERRUPT_PIN] = 0x01;
    memory_region_init_io(&s->mmio_io, OBJECT(s), &mpt3sas_mmio_ops, s,
            "mpt3sas-mmio", 0x10000);
    memory_region_init_io(&s->port_io, OBJECT(s), &mpt3sas_port_ops, s,
            "mpt3sas-io", 256);

    // nentries - 15 ??
    // table_bar_nr 0x1
    // table_offset 0x2000??
    // pba_bar_nr 0x1
    // pba_offset 0x3800 ??
    // cap_pos 0x68 ??

    //Disable msix now
#if 0
    if (s->msix_available  &&
        !msix_init(dev, 15, &s->mmio_io, 0x1, 0x2000,
            &s->mmio_io, 0x1, 0x3800, 0x68)) {
        DPRINTF("Initialize msix ok.\n");
        s->msix_in_use = true;
    }

    if (pci_is_express(dev)) {
        pcie_endpoint_cap_init(dev, 0xa0);
    }
#endif

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

    if (!s->sas_address) {
        s->sas_address = ((NAA_LOCALLY_ASSIGNED_ID << 24) |
                       IEEE_COMPANY_LOCALLY_ASSIGNED) << 36;
        s->sas_address |= (pci_bus_num(dev->bus) << 16);
        s->sas_address |= (PCI_SLOT(dev->devfn) << 8);
        s->sas_address |= PCI_FUNC(dev->devfn);
    }

    s->max_devices = MPT3SAS_NUM_PORTS;

    s->request_bh = qemu_bh_new(mpt3sas_handle_requests, s);
    s->event_handler = qemu_bh_new(mpt3sas_send_event, s);

    QTAILQ_INIT(&s->pending);

    scsi_bus_new(&s->bus, sizeof(s->bus), &dev->qdev, &mpt3sas_scsi_info, NULL);

    if (!d->hotplugged) {
        scsi_bus_legacy_handle_cmdline(&s->bus, errp);
    }

    // Do the hardreset
    mpt3sas_hard_reset(s);
}

static void mpt3sas_scsi_uninit(PCIDevice *dev)
{
    MPT3SASState *s = MPT3SAS(dev);

    qemu_bh_delete(s->request_bh);
    qemu_bh_delete(s->event_handler);
}

static void mpt3sas_reset(DeviceState *dev)
{
}

static Property mpt3sas_properties[] = {
    DEFINE_PROP_UINT64("sas_address", MPT3SASState, sas_address, 0),
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
