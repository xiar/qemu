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
#include "qapi/error.h"

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

#define MAX_SCSI_TARGETS    (255)
#define MAX_SCSI_LUNS        (1)

static uint32_t ioc_reset_sequence[] = {
    MPI2_WRSEQ_1ST_KEY_VALUE,
    MPI2_WRSEQ_2ND_KEY_VALUE,
    MPI2_WRSEQ_3RD_KEY_VALUE,
    MPI2_WRSEQ_4TH_KEY_VALUE,
    MPI2_WRSEQ_5TH_KEY_VALUE,
    MPI2_WRSEQ_6TH_KEY_VALUE};

static const int mpi2_request_sizes[] = {
    [MPI2_FUNCTION_SCSI_IO_REQUEST]     = sizeof(Mpi25SCSIIORequest_t),
    [MPI2_FUNCTION_SCSI_TASK_MGMT]      = sizeof(Mpi2SCSITaskManagementReply_t),
    [MPI2_FUNCTION_IOC_INIT]            = sizeof(Mpi2IOCInitRequest_t),
    [MPI2_FUNCTION_IOC_FACTS]           = sizeof(Mpi2IOCFactsRequest_t),
    [MPI2_FUNCTION_CONFIG]              = sizeof(Mpi2ConfigRequest_t),
    [MPI2_FUNCTION_PORT_FACTS]          = sizeof(Mpi2PortFactsRequest_t),
    [MPI2_FUNCTION_PORT_ENABLE]         = sizeof(Mpi2PortEnableRequest_t),
    [MPI2_FUNCTION_EVENT_NOTIFICATION]  = sizeof(Mpi2EventNotificationRequest_t),
    [MPI2_FUNCTION_EVENT_ACK]           = sizeof(Mpi2EventAckRequest_t),
    [MPI2_FUNCTION_FW_DOWNLOAD]         = sizeof(Mpi2FWDownloadRequest),
    [MPI2_FUNCTION_FW_UPLOAD]           = sizeof(Mpi2FWUploadRequest_t),
    [MPI2_FUNCTION_SMP_PASSTHROUGH]     = sizeof(Mpi2SmpPassthroughRequest_t)
    //[MPI2_FUNCTION_TARGET_ASSIST]       = sizeof(Mpi2TargetAssistRequest_t),
};

#ifdef DEBUG_MPT3SAS
#define DPRINTF(fmt, ...) \
    do { struct timeval _now; gettimeofday(&_now, NULL); qemu_log_mask(LOG_TRACE, "[%zd.%06zd] mpt3sas: " fmt, (size_t)_now.tv_sec, (size_t)_now.tv_usec, ##__VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

static void mpt3sas_event_enqueue(MPT3SASState *s, uint16_t event_type, void *event_data);

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

#define HANDLE_TO_SCSI_ID(handle)   ((handle) - MPT3SAS_ATTACHED_DEV_HANDLE_START)
#define SCSI_ID_TO_HANDLE(scsi_id)  (MPT3SAS_ATTACHED_DEV_HANDLE_START + (scsi_id))

#define PHY_INDEX(s, scsi_id) ((scsi_id) % (s)->expander.downstream_phys)
#define SCSI_ID_TO_EXP_PHY(s, scsi_id) (PHY_INDEX(s, scsi_id) == s->expander.downstream_phys - 1 ? (s)->expander.all_phys - 1 : (s)->expander.downstream_start_phy + PHY_INDEX((s), (scsi_id)))
#define EXP_PHY_TO_SCSI_ID(s, exp_id, phy_id) ((phy_id) == (s)->expander.all_phys - 1 ? (s)->expander.downstream_phys - 1 : (phy_id) + (exp_id) * (s)->expander.downstream_phys - (s)->expander.downstream_start_phy)

#if 0
static int mpt3sas_get_scsi_drive_num(MPT3SASState *s)
{
    int n = 0;
    while (scsi_device_find(&s->bus, 0, n++, 0) != NULL);
    return n;
}
#endif

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

static void mpt3sas_print_scsi_devices(SCSIBus *bus)
{
    BusChild *kid;
    QTAILQ_FOREACH_REVERSE(kid, &bus->qbus.children, ChildrenHead, sibling) {
        DeviceState *qdev = kid->child;
        SCSIDevice *dev = SCSI_DEVICE(qdev);

        trace_mpt3sas_scsi_device_list(dev, dev->channel, dev->id, dev->lun);
    }
}
static int mpt3sas_scsi_device_find(MPT3SASState *s, uint16_t dev_handle,
        uint8_t *lun, SCSIDevice **sdev)
{
    int target;


    if (!dev_handle || dev_handle == 0xffff) {
        return MPI2_IOCSTATUS_SCSI_INVALID_DEVHANDLE;
    }

    target = HANDLE_TO_SCSI_ID(dev_handle);

    if (target >= s->max_devices) {
        return MPI2_IOCSTATUS_SCSI_DEVICE_NOT_THERE;
    }

    *sdev = scsi_device_find(&s->bus, 0, target, lun[1]);
    if (!*sdev) {
        return MPI2_IOCSTATUS_SCSI_DEVICE_NOT_THERE;
    }

    return 0;
}

static uint32_t  mpt3sas_get_sas_end_device_info(MPT3SASState *s, uint32_t scsi_id, uint16_t *attached_dev_handle)
{
    if (scsi_device_find(&s->bus, 0, scsi_id, 0)) {
        *attached_dev_handle = MPT3SAS_ATTACHED_DEV_HANDLE_START + scsi_id;

        return MPI2_SAS_DEVICE_INFO_END_DEVICE |
            MPI2_SAS_DEVICE_INFO_LSI_DEVICE |
            MPI2_SAS_DEVICE_INFO_SSP_TARGET |
            MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH;

    }
    *attached_dev_handle = 0;
    return MPI2_SAS_DEVICE_INFO_NO_DEVICE;
}

static uint16_t mpt3sas_get_parent_dev_handle(MPT3SASState *s, uint32_t scsi_id, uint8_t *phy_num, uint8_t *physical_port)
{
    if (s->expander.count != 0) {
        uint8_t expander_idx = scsi_id / s->expander.downstream_phys;

        *phy_num = SCSI_ID_TO_EXP_PHY(s, scsi_id);
        *physical_port = scsi_id + s->expander.count;
        return MPT3SAS_EXPANDER_HANDLE_START + expander_idx;
    }

    *phy_num = scsi_id;
    *physical_port = scsi_id;
    return MPT3SAS_IOC_HANDLE_START + scsi_id;
}

static uint16_t mpt3sas_get_expander_parent_handle(MPT3SASState *s, uint8_t expander_idx)
{
    return MPT3SAS_IOC_HANDLE_START + expander_idx * s->expander.upstream_phys;
}

static uint32_t mpt3sas_get_sas_expander_device_info(MPT3SASState *s, uint8_t expander_idx, uint8_t *port, uint16_t *parent_handle, uint16_t *expander_handle)
{
    *port = expander_idx;
    *expander_handle = MPT3SAS_EXPANDER_HANDLE_START + expander_idx;
    *parent_handle = mpt3sas_get_expander_parent_handle(s, expander_idx);
    return MPI2_SAS_DEVICE_INFO_EDGE_EXPANDER |
           MPI2_SAS_DEVICE_INFO_LSI_DEVICE |
           MPI2_SAS_DEVICE_INFO_SMP_INITIATOR |
           MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH;
}

static void mpt3sas_get_attached_device_info(MPT3SASState *s, uint32_t phy_num, uint32_t *device_info, uint8_t *port, uint16_t *ioc_dev_handle, uint16_t *attached_dev_handle)
{
    if (s->expander.count) {
        uint8_t expander_idx = phy_num / s->expander.upstream_phys;

        if (expander_idx >= s->expander.count) {
            *port = phy_num - s->expander.count * (s->expander.upstream_phys - 1);
            *device_info = MPI2_SAS_DEVICE_INFO_NO_DEVICE;
            *ioc_dev_handle = MPT3SAS_IOC_HANDLE_START + phy_num;
            *attached_dev_handle = 0;
        } else {
            *device_info = mpt3sas_get_sas_expander_device_info(s, expander_idx, port,
                    ioc_dev_handle, attached_dev_handle);
        }
    } else {
        *port = phy_num;
        *device_info = mpt3sas_get_sas_end_device_info(s, phy_num, attached_dev_handle);
        *ioc_dev_handle = MPT3SAS_IOC_HANDLE_START + phy_num;
    }
}

static uint32_t mpt3sas_get_phy_device_info(MPT3SASState *s)
{
    uint32_t device_info = MPI2_SAS_DEVICE_INFO_SSP_INITIATOR | MPI2_SAS_DEVICE_INFO_END_DEVICE;

    if (s->expander.count) {
        device_info |= MPI2_SAS_DEVICE_INFO_SMP_INITIATOR;
    }
    return device_info;
}

//TODO: Most the config pages need to be configured again for making the host driver works.
// currently just for make host linux driver initialization working
static size_t mpt3sas_config_manufacturing_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ManufacturingPage0_t man_pg0;

    if (!data)
        return sizeof(man_pg0);

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

static size_t mpt3sas_config_manufacturing_3(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ManufacturingPage3_t man_pg3;

    if (!data)
        return sizeof(man_pg3);

    memset(&man_pg3, 0, sizeof(man_pg3));
    man_pg3.Header.PageVersion = MPI2_MANUFACTURING4_PAGEVERSION;
    man_pg3.Header.PageLength = sizeof(man_pg3) / 4;
    man_pg3.Header.PageNumber = 0x3;
    man_pg3.Header.PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;

    // TODO: get real data from hw
    man_pg3.ChipId.DeviceID = MPI25_MFGPAGE_DEVID_SAS3008;
    man_pg3.ChipId.PCIRevisionID = 0x01;

    if (data) {
        *data = g_malloc(sizeof(man_pg3));
        memcpy(*data, &man_pg3, sizeof(man_pg3));
    }
    return sizeof(man_pg3);
}

static size_t mpt3sas_config_manufacturing_4(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ManufacturingPage4_t man_pg4;

    if (!data)
        return sizeof(man_pg4);

    memset(&man_pg4, 0, sizeof(man_pg4));
    man_pg4.Header.PageVersion = MPI2_MANUFACTURING4_PAGEVERSION;
    man_pg4.Header.PageLength = sizeof(man_pg4) / 4;
    man_pg4.Header.PageNumber = 0x4;
    man_pg4.Header.PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;
    if (data) {
        *data = g_malloc(sizeof(man_pg4));
        memcpy(*data, &man_pg4, sizeof(man_pg4));
    }
    return sizeof(man_pg4);
}

static size_t mpt3sas_config_manufacturing_5(MPT3SASState *s, uint8_t **data, int address)
{
    size_t page_len;
    int i;
    Mpi2ManufacturingPage5_t *man_pg5 = NULL;

    page_len = offsetof(Mpi2ManufacturingPage5_t, Phy) + sizeof(Mpi2Manufacturing5Entry_t) * MPT3SAS_NUM_PHYS;

    if (!data)
        return page_len;

    man_pg5 = g_malloc(page_len);
    man_pg5->Header.PageVersion = MPI2_MANUFACTURING5_PAGEVERSION;
    man_pg5->Header.PageLength = sizeof(page_len) / 4;
    man_pg5->Header.PageNumber = 0x5;
    man_pg5->Header.PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;

    for (i = 0; i < MPT3SAS_NUM_PHYS; i++) {
        man_pg5->Phy[i].WWID = cpu_to_le64(s->sas_address); //FIXME
        man_pg5->Phy[i].DeviceName = cpu_to_le64(s->sas_address); //FIXME
    }
    if (data) {
        *data = (uint8_t *)man_pg5;
    }

    return page_len;
}

static size_t mpt3sas_config_manufacturing_10(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ManufacturingPage10_t *man_pg10 = NULL;
    uint32_t page_len = sizeof(Mpi2ManufacturingPage10_t);

    if (!data)
        return page_len;

    man_pg10 = g_malloc(page_len);
    memset(man_pg10, 0, page_len);

    man_pg10->Header.PageVersion = MPI2_MANUFACTURING11_PAGEVERSION;
    man_pg10->Header.PageNumber = 0xa;
    man_pg10->Header.PageType = MPI2_CONFIG_PAGETYPE_MANUFACTURING;
    man_pg10->Header.PageLength = page_len / 4;

    if (data) {
        *data = (uint8_t *)man_pg10;
    }

    return page_len;
}

static size_t mpt3sas_config_manufacturing_11(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ConfigPageHeader_t *man_pg11 = NULL;
    uint32_t page_len = sizeof(Mpi2ConfigPageHeader_t) + 0x40;

    if (!data)
        return page_len;

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

    if (!data)
        return sizeof(bios_pg2);

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

    if (!data)
        return sizeof(bios_pg3);

    memset(&bios_pg3, 0, sizeof(bios_pg3));
    bios_pg3.Header.PageVersion = MPI2_BIOSPAGE3_PAGEVERSION;
    bios_pg3.Header.PageNumber = 0x3;
    bios_pg3.Header.PageType = MPI2_CONFIG_PAGETYPE_BIOS;
    bios_pg3.Header.PageLength = sizeof(bios_pg3) / 4;
    bios_pg3.GlobalFlags = cpu_to_le32(MPI2_BIOSPAGE3_FLAGS_HOOK_INT_40_DISABLE | MPI2_BIOSPAGE3_FLAGS_VERBOSE_ENABLE);
    bios_pg3.BiosVersion = cpu_to_le32(0x2030101);
    if (data) {
       *data = g_malloc(sizeof(bios_pg3));
       memcpy(*data, &bios_pg3, sizeof(bios_pg3));
    }
    return sizeof(bios_pg3);
}

static size_t mpt3sas_config_ioc_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOCPage0_t ioc_pg0;

    if (!data)
        return sizeof(ioc_pg0);

    memset(&ioc_pg0, 0, sizeof(ioc_pg0));
    ioc_pg0.Header.PageVersion = MPI2_IOCPAGE0_PAGEVERSION;
    ioc_pg0.Header.PageNumber = 0x0;
    ioc_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_IOC;
    ioc_pg0.Header.PageLength = sizeof(ioc_pg0) / 4;
    ioc_pg0.VendorID = MPI2_MFGPAGE_VENDORID_LSI;
    ioc_pg0.DeviceID = MPI25_MFGPAGE_DEVID_SAS3008;
    ioc_pg0.RevisionID = 0x2;
    ioc_pg0.ClassCode = 0x107;
    ioc_pg0.SubsystemVendorID = 0x1f53;
    ioc_pg0.SubsystemID = 0x1028;

    if (data) {
       *data = g_malloc(sizeof(ioc_pg0));
       memcpy(*data, &ioc_pg0, sizeof(ioc_pg0));
    }

    return sizeof(ioc_pg0);
}

static size_t mpt3sas_config_ioc_8(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOCPage8_t ioc_pg8;

    if (!data)
        return sizeof(ioc_pg8);

    memset(&ioc_pg8, 0, sizeof(ioc_pg8));
    ioc_pg8.Header.PageVersion = MPI2_IOCPAGE8_PAGEVERSION;
    ioc_pg8.Header.PageNumber = 0x8;
    ioc_pg8.Header.PageType = MPI2_CONFIG_PAGETYPE_IOC;
    ioc_pg8.Header.PageLength = sizeof(ioc_pg8) / 4;
    ioc_pg8.NumDevsPerEnclosure = 0x0; //Default value, TODO
    ioc_pg8.MaxPersistentEntries = 0x0;
    ioc_pg8.MaxNumPhysicalMappedIDs = cpu_to_le32(0x8);
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

    if (!data)
        return sizeof(iounit_pg0);

    memset(&iounit_pg0, 0, sizeof(iounit_pg0));
    iounit_pg0.Header.PageVersion = MPI2_IOUNITPAGE0_PAGEVERSION;
    iounit_pg0.Header.PageNumber = 0x0;
    iounit_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;
    iounit_pg0.Header.PageLength = sizeof(iounit_pg0) / 4;
    iounit_pg0.UniqueValue = cpu_to_le64(0x53504D554D4553LL | (uint64_t)pci->devfn << 56);
    if (data) {
        *data = g_malloc(sizeof(iounit_pg0));
        memcpy(*data, &iounit_pg0, sizeof(iounit_pg0));
    }

    return sizeof(iounit_pg0);
}

static size_t mpt3sas_config_io_unit_1(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOUnitPage1_t iounit_pg1;

    if (!data)
        return sizeof(iounit_pg1);

    memset(&iounit_pg1, 0, sizeof(iounit_pg1));
    iounit_pg1.Header.PageVersion = MPI2_IOUNITPAGE1_PAGEVERSION;
    iounit_pg1.Header.PageNumber = 0x1;
    iounit_pg1.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;
    iounit_pg1.Header.PageLength = sizeof(iounit_pg1) / 4;
    iounit_pg1.Flags = cpu_to_le32(0x41);
    if (data) {
        *data = g_malloc(sizeof(iounit_pg1));
        memcpy(*data, &iounit_pg1, sizeof(iounit_pg1));
    }

    return sizeof(iounit_pg1);
}

static size_t mpt3sas_config_io_unit_7(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOUnitPage7_t iounit_pg7;

    if (!data)
        return sizeof(iounit_pg7);

    memset(&iounit_pg7, 0, sizeof(iounit_pg7));
    iounit_pg7.Header.PageVersion = MPI2_IOUNITPAGE8_PAGEVERSION;
    iounit_pg7.Header.PageNumber = 0x7;
    iounit_pg7.Header.PageType = MPI2_CONFIG_PAGETYPE_IO_UNIT;

    iounit_pg7.CurrentPowerMode = MPI25_IOUNITPAGE7_PM_INIT_HOST | MPI25_IOUNITPAGE7_PM_MODE_FULL_POWER;
    iounit_pg7.PreviousPowerMode = MPI25_IOUNITPAGE7_PM_INIT_HOST | MPI25_IOUNITPAGE7_PM_MODE_FULL_POWER;
    iounit_pg7.PCIeWidth = MPI2_IOUNITPAGE7_PCIE_WIDTH_X8;
    iounit_pg7.PCIeSpeed = MPI2_IOUNITPAGE7_PCIE_SPEED_8_0_GBPS;
    iounit_pg7.ProcessorState = MPI2_IOUNITPAGE7_PSTATE_ENABLED;
    iounit_pg7.PowerManagementCapabilities = MPI25_IOUNITPAGE7_PMCAP_HOST_FULL_PWR_MODE|MPI25_IOUNITPAGE7_PMCAP_HOST_REDUCED_PWR_MODE;
    iounit_pg7.IOCTemperature = MPI2_IOUNITPAGE7_IOC_TEMP_CELSIUS;
    iounit_pg7.IOCTemperatureUnits = 1;
    iounit_pg7.IOCSpeed = MPI2_IOUNITPAGE7_IOC_SPEED_FULL;
    iounit_pg7.BoardTemperature = MPI2_IOUNITPAGE7_BOARD_TEMP_CELSIUS;
    // reserved in spec
    //iounit_pg7.BoardTemperatureUnits = 1;
    //iounit_pg7.BoardPowerRequirement = 1;

    if (data) {
        *data = g_malloc(sizeof(iounit_pg7));
        memcpy(*data, &iounit_pg7, sizeof(iounit_pg7));
    }

    return sizeof(iounit_pg7);
}

static size_t mpt3sas_config_io_unit_8(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2IOUnitPage8_t iounit_pg8;

    if (!data)
        return sizeof(iounit_pg8);

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

    page_len = offsetof(Mpi2SasIOUnitPage0_t, PhyData) + sizeof(Mpi2SasIOUnit0PhyData_t) * MPT3SAS_NUM_PHYS;

    if (!data)
        return page_len;

    sas_iounit_pg0 = g_malloc(page_len);
    memset(sas_iounit_pg0, 0, page_len);
    sas_iounit_pg0->Header.PageVersion = MPI2_SASIOUNITPAGE0_PAGEVERSION;
    sas_iounit_pg0->Header.PageNumber = 0x0;
    sas_iounit_pg0->Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_iounit_pg0->Header.ExtPageLength = page_len / 4;
    sas_iounit_pg0->Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT;
    sas_iounit_pg0->NumPhys = MPT3SAS_NUM_PHYS;

    for (i = 0; i < MPT3SAS_NUM_PHYS; i++) {
        uint32_t device_info;
        sas_iounit_pg0->PhyData[i].Port = 0;
        sas_iounit_pg0->PhyData[i].PortFlags = MPI2_SASIOUNIT0_PORTFLAGS_AUTO_PORT_CONFIG;
        sas_iounit_pg0->PhyData[i].PhyFlags = 0x0;
        sas_iounit_pg0->PhyData[i].NegotiatedLinkRate =  (MPI25_SAS_NEG_LINK_RATE_12_0 << MPI2_SAS_NEG_LINK_RATE_SHIFT_LOGICAL) | MPI25_SAS_NEG_LINK_RATE_12_0;
        sas_iounit_pg0->PhyData[i].ControllerPhyDeviceInfo = mpt3sas_get_phy_device_info(s);

        mpt3sas_get_attached_device_info(s, i, &device_info, &sas_iounit_pg0->PhyData[i].Port,
                                        &sas_iounit_pg0->PhyData[i].ControllerDevHandle,
                                        &sas_iounit_pg0->PhyData[i].AttachedDevHandle);
        switch (device_info & MPI2_SAS_DEVICE_INFO_MASK_DEVICE_TYPE) {
            case MPI2_SAS_DEVICE_INFO_EDGE_EXPANDER:
                sas_iounit_pg0->PhyData[i].NegotiatedLinkRate =  (MPI25_SAS_NEG_LINK_RATE_12_0 << MPI2_SAS_NEG_LINK_RATE_SHIFT_LOGICAL) | MPI25_SAS_NEG_LINK_RATE_12_0;
                break;
            default:
                sas_iounit_pg0->PhyData[i].NegotiatedLinkRate = MPI2_SAS_NEG_LINK_RATE_UNKNOWN_LINK_RATE;
                break;
        }
        sas_iounit_pg0->PhyData[i].DiscoveryStatus = 0;
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

    page_len = offsetof(Mpi2SasIOUnitPage1_t, PhyData) + sizeof(Mpi2SasIOUnit1PhyData_t) * MPT3SAS_NUM_PHYS;

    if (!data)
        return page_len;

    sas_iounit_pg1 = g_malloc(page_len);
    memset(sas_iounit_pg1, 0, page_len);
    sas_iounit_pg1->Header.PageVersion = MPI2_SASIOUNITPAGE1_PAGEVERSION;
    sas_iounit_pg1->Header.PageNumber = 0x1;
    sas_iounit_pg1->Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_iounit_pg1->Header.ExtPageLength = page_len / 4;
    sas_iounit_pg1->Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_IO_UNIT;
    sas_iounit_pg1->NumPhys = MPT3SAS_NUM_PHYS;

    for (i = 0; i < MPT3SAS_NUM_PHYS; i++) {
        uint32_t attached_dev_info;
        uint16_t ioc_dev_handle;
        uint16_t attached_dev_handle;

        sas_iounit_pg1->PhyData[i].PortFlags = 0x1;
        sas_iounit_pg1->PhyData[i].PhyFlags = 0x0;
        sas_iounit_pg1->PhyData[i].MaxMinLinkRate = MPI25_SASIOUNIT1_MAX_RATE_12_0 | MPI2_SASIOUNIT1_MIN_RATE_3_0;
        sas_iounit_pg1->PhyData[i].ControllerPhyDeviceInfo = mpt3sas_get_phy_device_info(s);
        mpt3sas_get_attached_device_info(s, i, &attached_dev_info, &sas_iounit_pg1->PhyData[i].Port,
                                         &ioc_dev_handle, &attached_dev_handle);
    }

    sas_iounit_pg1->ControlFlags = 0x0200;
    sas_iounit_pg1->AdditionalControlFlags = 0x0;
    sas_iounit_pg1->SATAMaxQDepth = 0x0;
    sas_iounit_pg1->IODeviceMissingDelay = 0x0;
    sas_iounit_pg1->ReportDeviceMissingDelay = 0x0;

    if (data) {
        *data = (uint8_t *)sas_iounit_pg1;
    }

    return page_len;
}

static bool mpt3sas_phy_handle_present(MPT3SASState *s, uint16_t handle)
{
    if (s->expander.count) {
        uint32_t phy_id = handle - MPT3SAS_IOC_HANDLE_START;
        uint8_t expander_idx = phy_id / s->expander.upstream_phys;

        if (expander_idx < s->expander.count)
            return (phy_id % s->expander.upstream_phys) == 0;
    }

    return true;
}

static uint16_t mpt3sas_get_next_sas_device_handle(MPT3SASState *s, uint16_t handle)
{
    handle++;
    if ((int16_t)handle == 0) {
        handle = MPT3SAS_IOC_HANDLE_START;
    } else if (handle <= MPT3SAS_IOC_HANDLE_START) {
        return 0xFFFF;
    }

    while (handle < MPT3SAS_IOC_HANDLE_START + MPT3SAS_NUM_PHYS) {
        if (mpt3sas_phy_handle_present(s, handle)) {
            return handle;
        }
        handle++;
    }

    if (handle == MPT3SAS_IOC_HANDLE_START + MPT3SAS_NUM_PHYS) {
        handle = MPT3SAS_EXPANDER_HANDLE_START;
    } else if (handle <= MPT3SAS_EXPANDER_HANDLE_START) {
        return 0xFFFF;
    }

    if (handle < MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        return handle;
    }

    if (handle == MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        handle = MPT3SAS_ATTACHED_DEV_HANDLE_START;
    } else if (handle <= MPT3SAS_ATTACHED_DEV_HANDLE_START) {
        return 0xFFFF;
    }

    while (handle < MPT3SAS_ATTACHED_DEV_HANDLE_START + s->expander.all_phys * s->expander.count) {
        if (scsi_device_find(&s->bus, 0, handle - MPT3SAS_ATTACHED_DEV_HANDLE_START, 0)) {
            return handle;
        }
        handle++;
    }

    return 0xFFFF;
}

static uint16_t mpt3sas_get_sas_device_handle(MPT3SASState *s, uint32_t page_address)
{
    uint32_t form = page_address & MPI2_SAS_DEVICE_PGAD_FORM_MASK;
    uint16_t dev_handle = 0;

    switch (form) {
        case MPI2_SAS_DEVICE_PGAD_FORM_GET_NEXT_HANDLE:
            dev_handle = page_address & MPI2_SAS_DEVICE_PGAD_HANDLE_MASK;
            dev_handle = mpt3sas_get_next_sas_device_handle(s, dev_handle);
            break;
        case MPI2_SAS_DEVICE_PGAD_FORM_HANDLE:
            dev_handle = page_address & MPI2_SAS_DEVICE_PGAD_HANDLE_MASK;
            break;
        default:
            dev_handle = 0xFFFF;
            break;
    }

    return dev_handle;
}

static uint16_t mpt3sas_get_sas_enclosure_handle(MPT3SASState *s, uint32_t page_address)
{
    uint32_t form = page_address & MPI2_SAS_ENCLOS_PGAD_FORM_MASK;
    uint16_t dev_handle = 0;

    switch (form) {
        case MPI2_SAS_ENCLOS_PGAD_FORM_GET_NEXT_HANDLE:
            dev_handle = page_address & MPI2_SAS_ENCLOS_PGAD_HANDLE_MASK;
            dev_handle++;
            if (dev_handle == 0) {
                return MPT3SAS_ENCLOSURE_HANDLE_START;
            } else if (dev_handle < MPT3SAS_ENCLOSURE_HANDLE_START + s->expander.count + 1) { // 2 * expander enclosure + 1 * HBA enclosure
                return dev_handle;
            } else {
                return 0xFFFF;
            }
            break;
        case MPI2_SAS_ENCLOS_PGAD_FORM_HANDLE:
            dev_handle = page_address & MPI2_SAS_ENCLOS_PGAD_HANDLE_MASK;
            break;
        default:
            dev_handle = 0xFFFF;
            break;
    }

    return dev_handle;
}

static uint16_t mpt3sas_get_next_sas_expander_handle(MPT3SASState *s, uint16_t handle)
{
    handle++;

    if (handle == 0) {
        handle = MPT3SAS_EXPANDER_HANDLE_START;
    } else if (handle <= MPT3SAS_EXPANDER_HANDLE_START) {
        return 0xFFFF;
    }

    if (handle < MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        return handle;
    }

    return 0xFFFF;

}

static uint16_t mpt3sas_get_sas_expander_handle_phy(MPT3SASState *s, int page_address, uint32_t *phy_num)
{
    uint32_t form = page_address & MPI2_SAS_EXPAND_PGAD_FORM_MASK;
    uint16_t dev_handle = 0;

    switch (form) {
        case MPI2_SAS_EXPAND_PGAD_FORM_GET_NEXT_HNDL:
            dev_handle = page_address & MPI2_SAS_EXPAND_PGAD_HANDLE_MASK;
            dev_handle = mpt3sas_get_next_sas_expander_handle(s, dev_handle);
            break;
        case MPI2_SAS_EXPAND_PGAD_FORM_HNDL:
            dev_handle = page_address & MPI2_SAS_EXPAND_PGAD_HANDLE_MASK;
            break;
        case MPI2_SAS_EXPAND_PGAD_FORM_HNDL_PHY_NUM:
            dev_handle = page_address & MPI2_SAS_EXPAND_PGAD_HANDLE_MASK;
            if (phy_num)
                *phy_num = (page_address & MPI2_SAS_EXPAND_PGAD_PHYNUM_MASK) >> MPI2_SAS_EXPAND_PGAD_PHYNUM_SHIFT;
            break;
        default:
            dev_handle = 0xFFFF;
            break;
    }

    return dev_handle;
}

static size_t mpt3sas_config_sas_device_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasDevicePage0_t sas_device_pg0;
    uint16_t handle;
    uint8_t phy_id = 0;
    SCSIDevice  *d = NULL;
    uint32_t form = 0;

    if (!data)
        return sizeof(sas_device_pg0);

    handle = address & MPI2_SAS_DEVICE_PGAD_HANDLE_MASK;
    form = address & MPI2_SAS_DEVICE_PGAD_FORM_MASK;

    trace_mpt3sas_handle_config_sas_device_0(form, handle);

    if (handle) {
        handle = mpt3sas_get_sas_device_handle(s, address);

        if (handle == 0xFFFF)
            return -1;
    } else {
        return -1;
    }

    memset(&sas_device_pg0, 0, sizeof(sas_device_pg0));
    sas_device_pg0.Header.PageVersion = MPI2_SASDEVICE0_PAGEVERSION;
    sas_device_pg0.Header.PageNumber = 0x0;
    sas_device_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_device_pg0.Header.ExtPageLength = cpu_to_le16(sizeof(sas_device_pg0) / 4);
    sas_device_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_DEVICE;
    sas_device_pg0.AccessStatus = MPI2_SAS_DEVICE0_ASTATUS_NO_ERRORS;

    if (handle >= MPT3SAS_IOC_HANDLE_START &&
        handle < MPT3SAS_IOC_HANDLE_START + MPT3SAS_NUM_PHYS &&
        mpt3sas_phy_handle_present(s, handle)) { // ioc device

        phy_id = handle - MPT3SAS_IOC_HANDLE_START;

        sas_device_pg0.EnclosureHandle = cpu_to_le16(MPT3SAS_IOC_ENCLOSURE_HANDLE);
        sas_device_pg0.DevHandle = cpu_to_le16(handle);
        sas_device_pg0.PhyNum = 0x0;
        sas_device_pg0.DeviceInfo = mpt3sas_get_phy_device_info(s);
        sas_device_pg0.ParentDevHandle = 0x0;
        sas_device_pg0.PhysicalPort = phy_id;
        sas_device_pg0.SASAddress = cpu_to_le64(s->sas_address + phy_id);
        sas_device_pg0.Flags = cpu_to_le16(MPI2_SAS_DEVICE0_FLAGS_DEVICE_PRESENT | MPI2_SAS_DEVICE0_FLAGS_ENCL_LEVEL_VALID);
        sas_device_pg0.DmaGroup = handle;
    } else if (handle >= MPT3SAS_ATTACHED_DEV_HANDLE_START &&
            handle < MPT3SAS_ATTACHED_DEV_HANDLE_START + s->expander.all_phys * s->expander.count) { // attached sas device

        uint64_t sas_address = 0;
        uint32_t scsi_id = 0;
        uint8_t expander_idx = 0;

        scsi_id = HANDLE_TO_SCSI_ID(handle);

        d = scsi_device_find(&s->bus, 0, scsi_id, 0);
        if (d && d->wwn) {
            sas_address = d->wwn;
        } else if (d) {
            sas_address = MPT3SAS_DRIVE_DEFAULT_SAS_ADDR + scsi_id;
        } else {
            sas_address = 0;
        }

        trace_mpt3sas_query_scsi_target_info(handle, scsi_id, sas_address);

        expander_idx = scsi_id / s->expander.downstream_phys;

        sas_device_pg0.EnclosureHandle = MPT3SAS_EXPANDER_ENCLOSURE_HANDLE + expander_idx;
        if (d)
            sas_device_pg0.Slot = d->slot_number;
        else
            sas_device_pg0.Slot = scsi_id % s->expander.downstream_phys;
        sas_device_pg0.SASAddress = cpu_to_le64(sas_address);
        sas_device_pg0.DevHandle = cpu_to_le16(handle);
        sas_device_pg0.ParentDevHandle = mpt3sas_get_parent_dev_handle(s, scsi_id,
                                                                       &sas_device_pg0.PhyNum,
                                                                       &sas_device_pg0.PhysicalPort);
        sas_device_pg0.DeviceInfo = cpu_to_le32(MPI2_SAS_DEVICE_INFO_SSP_TARGET |
                                                MPI2_SAS_DEVICE_INFO_END_DEVICE |
                                                MPI2_SAS_DEVICE_INFO_DIRECT_ATTACH);
        sas_device_pg0.Flags = cpu_to_le16(MPI2_SAS_DEVICE0_FLAGS_DEVICE_PRESENT |
                                           MPI2_SAS_DEVICE0_FLAGS_ENCL_LEVEL_VALID);
        sas_device_pg0.DeviceName = cpu_to_le64(sas_address - 1);
        sas_device_pg0.DmaGroup = MPT3SAS_ATTACHED_DEV_HANDLE_START;
        sas_device_pg0.MaxPortConnections = 0x1;
    } else if (handle >= MPT3SAS_EXPANDER_HANDLE_START &&
            handle < MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        uint8_t expander_idx = handle - MPT3SAS_EXPANDER_HANDLE_START;

        sas_device_pg0.EnclosureHandle = MPT3SAS_EXPANDER_ENCLOSURE_HANDLE + expander_idx;
        sas_device_pg0.AttachedPhyIdentifier = MPT3SAS_EXPANDER_PORT_START_PHY; //TODO same with physical io controller
        sas_device_pg0.SASAddress = cpu_to_le64(MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR + expander_idx);
        sas_device_pg0.DeviceInfo = mpt3sas_get_sas_expander_device_info(s, expander_idx,
                                                                        &sas_device_pg0.PhysicalPort,
                                                                        &sas_device_pg0.ParentDevHandle,
                                                                        &sas_device_pg0.DevHandle);

        sas_device_pg0.PhyNum = sas_device_pg0.ParentDevHandle - MPT3SAS_IOC_HANDLE_START;
        sas_device_pg0.Flags = cpu_to_le16(MPI2_SAS_DEVICE0_FLAGS_DEVICE_PRESENT | MPI2_SAS_DEVICE0_FLAGS_ENCL_LEVEL_VALID);
        sas_device_pg0.DmaGroup = MPT3SAS_EXPANDER_HANDLE_START;
        sas_device_pg0.MaxPortConnections = s->expander.count ? s->expander.upstream_phys : MPT3SAS_NUM_PHYS;
        sas_device_pg0.DeviceName = cpu_to_le64(MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR - 1);
    } else {
        sas_device_pg0.DevHandle = 0xFFFF;
        sas_device_pg0.ParentDevHandle = 0xFFFF;
        //return sizeof(sas_device_pg0);
    }

    if (data) {
        *data = g_malloc(sizeof(sas_device_pg0));
        memset(*data, 0, sizeof(sas_device_pg0));
        memcpy(*data, &sas_device_pg0, sizeof(sas_device_pg0));
    }

    return sizeof(sas_device_pg0);
}

static size_t mpt3sas_config_sas_phy_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasPhyPage0_t sas_phy_pg0;
    uint32_t phy_number = 0;
    //SCSIDevice  *d = scsi_device_find(&s->bus, 0, phy_number, 0);
    uint32_t form = address & MPI2_SAS_PHY_PGAD_FORM_MASK;
    uint32_t device_info;
    uint8_t port;

    if (!data)
        return sizeof(sas_phy_pg0);

    if (form == MPI2_SAS_PHY_PGAD_FORM_PHY_NUMBER) {
        trace_mpt3sas_handle_config_sas_phy_0(form, address & MPI2_SAS_PHY_PGAD_PHY_NUMBER_MASK);
        phy_number = address & MPI2_SAS_PHY_PGAD_PHY_NUMBER_MASK;
    } else if (form == MPI2_SAS_PHY_PGAD_FORM_PHY_TBL_INDEX) {
        trace_mpt3sas_handle_config_sas_phy_0(form, address & MPI2_SAS_PHY_PGAD_PHY_TBL_INDEX_MASK);
        return -1;
    }

    memset(&sas_phy_pg0, 0, sizeof(sas_phy_pg0));
    sas_phy_pg0.Header.PageVersion = MPI2_SASPHY0_PAGEVERSION;
    sas_phy_pg0.Header.PageNumber = 0x0;
    sas_phy_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_phy_pg0.Header.ExtPageLength = sizeof(sas_phy_pg0) / 4;
    sas_phy_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_PHY;

    mpt3sas_get_attached_device_info(s, phy_number, &device_info, &port,
                                     &sas_phy_pg0.OwnerDevHandle,
                                     &sas_phy_pg0.AttachedDevHandle);
    //sas_phy_pg0.AttachedDeviceInfo = device_info;
    switch (device_info & MPI2_SAS_DEVICE_INFO_MASK_DEVICE_TYPE) {
    case MPI2_SAS_DEVICE_INFO_EDGE_EXPANDER:
        sas_phy_pg0.AttachedPhyIdentifier = phy_number % s->expander.upstream_phys + s->expander.downstream_phys;
        break;
    case MPI2_SAS_DEVICE_INFO_END_DEVICE:
        sas_phy_pg0.AttachedPhyIdentifier = 0;
        break;
    default:
        sas_phy_pg0.AttachedPhyIdentifier = 0;
        break;

    }
    sas_phy_pg0.AttachedPhyInfo = MPI2_SAS_APHYINFO_REASON_POWER_ON;
    //if it's an expander device, then we should set, or set 0.
   // sas_phy_pg0.AttachedPhyIdentifier = MPT3SAS_EXPANDER_PORT_START_PHY + phy_number;
    sas_phy_pg0.HwLinkRate = MPI25_SAS_HWRATE_MAX_RATE_12_0 | MPI2_SAS_HWRATE_MIN_RATE_3_0;
    sas_phy_pg0.NegotiatedLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0;
    sas_phy_pg0.ProgrammedLinkRate = MPI25_SAS_PRATE_MAX_RATE_12_0 | MPI2_SAS_PRATE_MIN_RATE_3_0;
    sas_phy_pg0.Flags = 0x0;
    sas_phy_pg0.PhyInfo = MPI2_SAS_PHYINFO_REASON_POWER_ON;

    if (data) {
        *data = g_malloc(sizeof(sas_phy_pg0));
        memcpy(*data, &sas_phy_pg0, sizeof(sas_phy_pg0));
    }
    return sizeof(sas_phy_pg0);
}

static size_t mpt3sas_config_sas_phy_1(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasPhyPage1_t sas_phy_pg1;
    //TODO: Get real data for specified PHY
    //uint32_t phy_number = address;
    //SCSIDevice  *d = scsi_device_find(&s->bus, 0, phy_number, 0);

    if (!data)
        return sizeof(sas_phy_pg1);

    memset(&sas_phy_pg1, 0, sizeof(sas_phy_pg1));
    sas_phy_pg1.Header.PageVersion = MPI2_SASPHY0_PAGEVERSION;
    sas_phy_pg1.Header.PageNumber = 0x1;
    sas_phy_pg1.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_phy_pg1.Header.ExtPageLength = sizeof(sas_phy_pg1) / 4;
    sas_phy_pg1.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_PHY;

    sas_phy_pg1.InvalidDwordCount = 0x0;
    sas_phy_pg1.RunningDisparityErrorCount= 0x0;
    sas_phy_pg1.LossDwordSynchCount= 0x0;
    sas_phy_pg1.PhyResetProblemCount = 0x0;

    if (data) {
        *data = g_malloc(sizeof(sas_phy_pg1));
        memcpy(*data, &sas_phy_pg1, sizeof(sas_phy_pg1));
    }
    return sizeof(sas_phy_pg1);
}

static size_t mpt3sas_config_sas_expander_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ExpanderPage0_t exp_pg0;
    uint16_t dev_handle = 0;

    if (!data)
        return sizeof(exp_pg0);

    dev_handle = mpt3sas_get_sas_expander_handle_phy(s, address, NULL);
    trace_mpt3sas_config_sas_exp_pg0_dev_handle_address(dev_handle, address);
    if (dev_handle == 0xffff)
        return -1;

    memset(&exp_pg0, 0, sizeof(exp_pg0));

    exp_pg0.Header.PageVersion = MPI2_SASEXPANDER0_PAGEVERSION;
    exp_pg0.Header.PageNumber = 0x0;
    exp_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    exp_pg0.Header.ExtPageLength = sizeof(exp_pg0) / 4;
    exp_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_EXPANDER;

    if (dev_handle >= MPT3SAS_EXPANDER_HANDLE_START &&
        dev_handle < MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        uint8_t expander_idx = dev_handle - MPT3SAS_EXPANDER_HANDLE_START;

        exp_pg0.PhysicalPort = expander_idx;
        exp_pg0.EnclosureHandle = MPT3SAS_EXPANDER_ENCLOSURE_HANDLE + expander_idx;

        exp_pg0.SASAddress = MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR + expander_idx;
        exp_pg0.DiscoveryStatus = 0x0;
        exp_pg0.DevHandle = MPT3SAS_EXPANDER_HANDLE_START + expander_idx;
        exp_pg0.ParentDevHandle = mpt3sas_get_expander_parent_handle(s, expander_idx);
        exp_pg0.ExpanderRouteIndexes = 0x0;
        exp_pg0.NumPhys = s->expander.all_phys;
    } else {
        return -1;
    }

    if (data) {
        *data = g_malloc(sizeof(exp_pg0));
        memcpy(*data, &exp_pg0, sizeof(exp_pg0));
    }

    return sizeof(exp_pg0);
}

static size_t mpt3sas_config_sas_expander_1(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2ExpanderPage1_t exp_pg1;
    uint32_t phy_id = 0;
    uint16_t dev_handle = 0;

    if (!data)
        return sizeof(exp_pg1);

    dev_handle = mpt3sas_get_sas_expander_handle_phy(s, address, &phy_id);

    if (dev_handle == 0xFFFF)
        return -1;

    memset(&exp_pg1, 0, sizeof(exp_pg1));
    exp_pg1.Header.PageVersion = MPI2_SASEXPANDER1_PAGEVERSION;
    exp_pg1.Header.PageNumber = 0x1;
    exp_pg1.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    exp_pg1.Header.ExtPageLength = sizeof(exp_pg1) / 4;
    exp_pg1.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_SAS_EXPANDER;

    if (dev_handle >= MPT3SAS_EXPANDER_HANDLE_START &&
        dev_handle < MPT3SAS_EXPANDER_HANDLE_START + s->expander.count) {
        uint8_t expander_idx = dev_handle - MPT3SAS_EXPANDER_HANDLE_START;

        if (phy_id > s->expander.all_phys)
            return -1;

        exp_pg1.PhysicalPort = expander_idx;
        exp_pg1.NumPhys = s->expander.all_phys;
        exp_pg1.Phy = phy_id;
        exp_pg1.NumTableEntriesProgrammed = 0x0;
        exp_pg1.ProgrammedLinkRate = MPI25_SAS_PRATE_MAX_RATE_12_0 | MPI2_SAS_PRATE_MIN_RATE_3_0;
        exp_pg1.HwLinkRate = MPI25_SAS_HWRATE_MAX_RATE_12_0 | MPI2_SAS_HWRATE_MIN_RATE_3_0;

        trace_mpt3sas_sas_expander_config_page_1(expander_idx, phy_id);

        if (phy_id >= s->expander.downstream_start_phy && phy_id < s->expander.downstream_phys) {
            uint32_t device_info;
            uint32_t scsi_id = EXP_PHY_TO_SCSI_ID(s, expander_idx, phy_id);

            device_info = mpt3sas_get_sas_end_device_info(s, scsi_id, &exp_pg1.AttachedDevHandle);
            if (exp_pg1.AttachedDevHandle) {
                exp_pg1.AttachedPhyInfo = MPI2_SAS_APHYINFO_REASON_POWER_ON;
                exp_pg1.ExpanderDevHandle = MPT3SAS_EXPANDER_HANDLE_START + expander_idx;
                exp_pg1.DiscoveryInfo = MPI2_SAS_EXPANDER1_DISCINFO_LINK_STATUS_CHANGE;
                exp_pg1.AttachedDeviceInfo = device_info;
            }
            switch (device_info & MPI2_SAS_DEVICE_INFO_MASK_DEVICE_TYPE) {
            case MPI2_SAS_DEVICE_INFO_EDGE_EXPANDER:
                abort();
            case MPI2_SAS_DEVICE_INFO_END_DEVICE:
                exp_pg1.NegotiatedLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0;
                break;
            default:
                exp_pg1.NegotiatedLinkRate = MPI2_SAS_NEG_LINK_RATE_UNKNOWN_LINK_RATE;
                break;
            }
        } else if (phy_id >= s->expander.upstream_start_phy && phy_id < s->expander.upstream_phys) {
            exp_pg1.AttachedDeviceInfo = mpt3sas_get_phy_device_info(s);
            exp_pg1.AttachedDevHandle = mpt3sas_get_expander_parent_handle(s, expander_idx);
            exp_pg1.NegotiatedLinkRate = MPI25_SAS_NEG_LINK_RATE_12_0;
            exp_pg1.AttachedPhyInfo = MPI2_SAS_APHYINFO_REASON_POWER_ON;
            exp_pg1.ExpanderDevHandle = MPT3SAS_EXPANDER_HANDLE_START + expander_idx;
            exp_pg1.DiscoveryInfo = MPI2_SAS_EXPANDER1_DISCINFO_LINK_STATUS_CHANGE;
        } else if (phy_id >= s->expander.upstream_start_phy + s->expander.upstream_phys && phy_id < s->expander.downstream_start_phy) {
            exp_pg1.AttachedDeviceInfo =0;
            exp_pg1.AttachedDevHandle = 0;
            exp_pg1.NegotiatedLinkRate = MPI2_SAS_NEG_LINK_RATE_UNKNOWN_LINK_RATE;
            exp_pg1.AttachedPhyInfo = 0;
            exp_pg1.ExpanderDevHandle = 0;
            exp_pg1.DiscoveryInfo = 0;
        } else {
            return -1;
        }
    }

    if (data) {
        *data = g_malloc(sizeof(exp_pg1));
        memcpy(*data, &exp_pg1, sizeof(exp_pg1));
    }

    return sizeof(exp_pg1);
}

static size_t mpt3sas_config_enclosure_0(MPT3SASState *s, uint8_t **data, int address)
{
    Mpi2SasEnclosurePage0_t sas_enclosure_pg0;
    uint16_t handle = mpt3sas_get_sas_enclosure_handle(s, address);

    if (!data)
        return sizeof(sas_enclosure_pg0);

    if (handle == 0xFFFF)
        return -1;

    memset(&sas_enclosure_pg0, 0, sizeof(sas_enclosure_pg0));
    sas_enclosure_pg0.Header.PageVersion = MPI2_SASENCLOSURE0_PAGEVERSION;
    sas_enclosure_pg0.Header.PageNumber = 0x0;
    sas_enclosure_pg0.Header.PageType = MPI2_CONFIG_PAGETYPE_EXTENDED;
    sas_enclosure_pg0.Header.ExtPageLength = sizeof(sas_enclosure_pg0) / 4;
    sas_enclosure_pg0.Header.ExtPageType = MPI2_CONFIG_EXTPAGETYPE_ENCLOSURE;

    if (handle == MPT3SAS_IOC_ENCLOSURE_HANDLE) {
        sas_enclosure_pg0.EnclosureLogicalID = s->sas_address;
        sas_enclosure_pg0.NumSlots = MPT3SAS_IOC_NUM_SLOTS;
    } else {
        uint8_t expander_idx = handle - MPT3SAS_ENCLOSURE_HANDLE_START;
        uint8_t i = 0;
        SCSIDevice *ses_device = NULL;

        sas_enclosure_pg0.EnclosureLogicalID = MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR + expander_idx;
        //sas_enclosure_pg0.NumSlots = MPT3SAS_EXPANDER_NUM_SLOTS; 
        sas_enclosure_pg0.NumSlots = s->expander.all_phys - s->expander.upstream_phys; 
        sas_enclosure_pg0.Flags = MPI2_SAS_ENCLS0_FLAGS_MNG_SES_ENCLOSURE;
        sas_enclosure_pg0.SEPDevHandle = 0x0;

        while (i < (expander_idx + 1) * s->expander.downstream_phys) {
            if ((ses_device = scsi_device_find(&s->bus, 0, i++, 0)) == NULL)
                continue;

            if (ses_device->type == TYPE_ENCLOSURE) {
                sas_enclosure_pg0.SEPDevHandle = cpu_to_le16(SCSI_ID_TO_HANDLE(ses_device->id));
                break;
            }
        }
        
    }

    sas_enclosure_pg0.EnclosureHandle = cpu_to_le16(handle);
    sas_enclosure_pg0.StartSlot = 0x0;
    sas_enclosure_pg0.EnclosureLevel = 0x0;

    if (data) {
        *data = g_malloc(sizeof(sas_enclosure_pg0));
        memcpy(*data, &sas_enclosure_pg0, sizeof(sas_enclosure_pg0));
    }

    return sizeof(sas_enclosure_pg0);
}

static const MPT3SASConfigPage mpt3sas_config_pages[] = {
    {
        0, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_0,
    },
    {
        3, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_3,
    },
    {
        4, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_4,
    },
    {
        5, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_5,
    },
    {
        10, MPI2_CONFIG_PAGETYPE_MANUFACTURING,
        mpt3sas_config_manufacturing_10,
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
        0, MPI2_CONFIG_PAGETYPE_IOC,
        mpt3sas_config_ioc_0,
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
        7, MPI2_CONFIG_PAGETYPE_IO_UNIT,
        mpt3sas_config_io_unit_7,
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
    },
    {
        1, MPI2_CONFIG_EXTPAGETYPE_SAS_PHY,
        mpt3sas_config_sas_phy_1,
    },
    {
        0, MPI2_CONFIG_EXTPAGETYPE_SAS_EXPANDER,
        mpt3sas_config_sas_expander_0,
    },
    {
        1, MPI2_CONFIG_EXTPAGETYPE_SAS_EXPANDER,
        mpt3sas_config_sas_expander_1
    },
    {
        0, MPI2_CONFIG_EXTPAGETYPE_ENCLOSURE,
        mpt3sas_config_enclosure_0
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

static void mpt3sas_msix_notify(MPT3SASState *s, uint8_t vector)
{
    PCIDevice *pdev = PCI_DEVICE(s);

    if (s->msix_in_use && msix_enabled(pdev)) {
        trace_mpt3sas_msix_notify(vector);
        //s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
        msix_notify(pdev, vector);
    }
}

static void mpt3sas_clear_reply_descriptor_int(MPT3SASState *s, uint8_t msix_index)
{
    if (s->msix_in_use) {
        return;
    }

    if (s->reply_post[msix_index].host_index == s->reply_post[msix_index].ioc_index) {
        trace_mpt3sas_clear_reply_descriptor_int(s->intr_status);
        // host reply post queue is empty
        s->intr_status &= ~MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
        smp_wmb();
        mpt3sas_update_interrupt(s);
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

static void mpt3sas_post_reply(MPT3SASState *s, uint16_t smid, uint8_t msix_index, MPI2DefaultReply_t *reply)
{
    uint32_t reply_address_lo = 0;
    PCIDevice *pci = (PCIDevice *)s;
    Mpi2ReplyDescriptorsUnion_t descriptor;
    //uint16_t smid = reply->Reserved1;

    reply->Reserved1 = 0;

    if (s->reply_free_ioc_index == s->reply_free_host_index ||
        (s->reply_post[msix_index].host_index ==
        (s->reply_post[msix_index].ioc_index + 1) % s->reply_descriptor_post_queue_depth)) {
        mpt3sas_set_fault(s, MPI2_IOCSTATUS_INSUFFICIENT_RESOURCES);
        trace_mpt3sas_post_reply_error(s->reply_free_ioc_index, s->reply_free_host_index, s->reply_post[msix_index].ioc_index, s->reply_post[msix_index].host_index);
        return;
    }

    trace_mpt3sas_reply_free_queue(s->reply_free_ioc_index, s->reply_free_host_index);
    trace_mpt3sas_reply_post_queue(msix_index, s->reply_post[msix_index].base, s->reply_post[msix_index].ioc_index, s->reply_post[msix_index].host_index);

    // Prepare Reply Descriptor and Generate Interrupt to notify host
    // a Reply Descriptor was arrived.
    //
    // Get Reply free Queue and Write the data to dest through DMA.
    // Read reply address low 32-bit
    reply_address_lo = ldl_le_pci_dma(pci, s->reply_free_queue_address + s->reply_free_ioc_index * sizeof(uint32_t));

    trace_mpt3sas_reply_frame_address(((hwaddr)s->system_reply_address_hi << 32) | reply_address_lo);

    // write the data to dest address
    pci_dma_write(pci, ((hwaddr)s->system_reply_address_hi << 32) | reply_address_lo,
            reply, MIN(MPT3SAS_REPLY_FRAME_SIZE * 4, reply->MsgLength * 4));

    //Update reply_free_ioc_index
    s->reply_free_ioc_index = (s->reply_free_ioc_index == s->reply_free_queue_depth - 1) ? 0 : s->reply_free_ioc_index + 1;

    descriptor.AddressReply.ReplyFlags = MPI2_RPY_DESCRIPT_FLAGS_ADDRESS_REPLY;
    descriptor.AddressReply.MSIxIndex = msix_index;
    descriptor.AddressReply.SMID = smid;
    descriptor.AddressReply.ReplyFrameAddress = cpu_to_le32(reply_address_lo);

    //Write reply descriptor to reply post queue.1
    stq_le_pci_dma(pci, s->reply_post[msix_index].base + s->reply_post[msix_index].ioc_index * sizeof(uint64_t), descriptor.Words);
    s->reply_post[msix_index].ioc_index = (s->reply_post[msix_index].ioc_index == s->reply_descriptor_post_queue_depth - 1) ? 0 : s->reply_post[msix_index].ioc_index + 1;

    stq_le_pci_dma(pci, s->reply_post[msix_index].base + s->reply_post[msix_index].ioc_index * sizeof(uint64_t), 0xFFFFFFFFFFFFFFFF);

    s->completed_commands++;
    trace_mpt3sas_post_reply_completed(smid);

    // if the interrupt bit is already set, leave it up
    //if (s->intr_status & MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT)
    //    return;

    if (s->doorbell_state == DOORBELL_WRITE)
        s->doorbell_state = DOORBELL_NONE;

    if (s->msix_in_use) {
        mpt3sas_msix_notify(s, msix_index);
    } else {
        s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
        s->intr_status |= MPI2_HIS_IOC2SYS_DB_STATUS;
        mpt3sas_update_interrupt(s);
    }
}

static void mpt3sas_scsi_io_reply(MPT3SASState *s, uint16_t smid, uint16_t msix_index)
{
    Mpi2ReplyDescriptorsUnion_t descriptor;

    if ( (s->reply_post[msix_index].host_index == (s->reply_post[msix_index].ioc_index + 1) % s->reply_descriptor_post_queue_depth)) {
        mpt3sas_set_fault(s, MPI2_IOCSTATUS_INSUFFICIENT_RESOURCES);
        trace_mpt3sas_post_reply_error(s->reply_free_ioc_index, s->reply_free_host_index, s->reply_post[msix_index].ioc_index, s->reply_post[msix_index].ioc_index);
        return;
    }

    descriptor.SCSIIOSuccess.ReplyFlags = MPI2_RPY_DESCRIPT_FLAGS_SCSI_IO_SUCCESS;
    descriptor.SCSIIOSuccess.MSIxIndex = msix_index;
    descriptor.SCSIIOSuccess.SMID = smid;
    descriptor.SCSIIOSuccess.TaskTag = 0x0; //TODO, this value is assigned by IOC

    stq_le_pci_dma(PCI_DEVICE(s), s->reply_post[msix_index].base + s->reply_post[msix_index].ioc_index * sizeof(uint64_t), descriptor.Words);
    s->reply_post[msix_index].ioc_index = (s->reply_post[msix_index].ioc_index == s->reply_descriptor_post_queue_depth - 1) ? 0 : s->reply_post[msix_index].ioc_index + 1;

    // Make sure the next queue slot is unused. I found that the head of reply descriptor post queue held by driver
    // is increased a bit fast than the tail of reply descriptor post queue held by IOC.
    stq_le_pci_dma(PCI_DEVICE(s), s->reply_post[msix_index].base + s->reply_post[msix_index].ioc_index * sizeof(uint64_t), 0xFFFFFFFFFFFFFFFF);

    //s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
    //mpt3sas_update_interrupt(s);
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
        mpt3sas_post_reply(n->s, n->smid, n->msix_index, (MPI2DefaultReply_t *)n->reply);
        g_free(n->reply);
    }
    g_free(n);
}

static void mpt3sas_event_sas_enclosure_device_status_change(MPT3SASState *s, void *data)
{
    pMpi2EventDataSasEnclDevStatusChange_t sedsc = NULL;
    uint32_t event_data_length = ((MPT3SASEventData *)data)->length;
    Mpi2EventNotificationReply_t *reply = NULL;

    sedsc = (Mpi2EventDataSasEnclDevStatusChange_t *)((MPT3SASEventData *)data)->data;

    reply = g_malloc(sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    memset(reply, 0, sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    reply->EventDataLength = cpu_to_le16(event_data_length / 4);
    reply->AckRequired = 1;
    reply->MsgLength = (sizeof(Mpi2EventNotificationReply_t) + event_data_length) / 4;
    reply->Function = MPI2_FUNCTION_EVENT_NOTIFICATION;
    reply->Event = cpu_to_le16(MPI2_EVENT_SAS_ENCL_DEVICE_STATUS_CHANGE);
    reply->Reserved1 = 0;
    memcpy(reply->EventData, sedsc, event_data_length);
    mpt3sas_post_reply(s, 0, 0, (MPI2DefaultReply_t *)reply);
    free(reply);
}

static void mpt3sas_event_sas_topology_change_list(MPT3SASState *s, void *data)
{
    Mpi2EventDataSasTopologyChangeList_t *stcl = NULL;
    Mpi2EventNotificationReply_t *reply = NULL;
    uint32_t event_data_length = 0;

    event_data_length = ((MPT3SASEventData *)data)->length;

    stcl = (Mpi2EventDataSasTopologyChangeList_t *)((MPT3SASEventData *)data)->data;

    mpt3sas_print_scsi_devices(&s->bus);

    reply = g_malloc(sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    memset(reply, 0, sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    reply->EventDataLength = cpu_to_le16(event_data_length / 4);
    reply->AckRequired = 1;
    reply->MsgLength = (sizeof(Mpi2EventNotificationReply_t) + event_data_length) / 4;
    reply->Function = MPI2_FUNCTION_EVENT_NOTIFICATION;
    reply->Event = cpu_to_le16(MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST);
    reply->Reserved1 = 0;
    memcpy(reply->EventData, stcl, event_data_length);
    mpt3sas_post_reply(s, 0, 0, (MPI2DefaultReply_t *)reply);
    free(reply);
}

static void mpt3sas_event_sas_discovery(MPT3SASState *s, void *data)
{
    Mpi2EventDataSasDiscovery_t *dsd = NULL;
    uint32_t event_data_length = 0;
    Mpi2EventNotificationReply_t *reply = NULL;

    dsd = (Mpi2EventDataSasDiscovery_t *)((MPT3SASEventData *)data)->data;
    event_data_length = ((MPT3SASEventData *)data)->length;

    reply = g_malloc(sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    memset(reply, 0, sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    reply->EventDataLength = cpu_to_le16(event_data_length / 4);
    reply->AckRequired = 1;
    reply->MsgLength = (sizeof(Mpi2EventNotificationReply_t) + event_data_length) / 4;
    reply->Function = MPI2_FUNCTION_EVENT_NOTIFICATION;
    reply->Event = cpu_to_le16(MPI2_EVENT_SAS_DISCOVERY);
    memcpy(reply->EventData, dsd, event_data_length);
    mpt3sas_post_reply(s, 0, 0, (MPI2DefaultReply_t *)reply);
    g_free(reply);
}

static void mpt3sas_event_sas_device_status_change(MPT3SASState *s, void *data)
{
    Mpi2EventDataSasDeviceStatusChange_t *dsd = NULL;
    uint32_t event_data_length = 0;
    Mpi2EventNotificationReply_t *reply = NULL;

    dsd = (Mpi2EventDataSasDeviceStatusChange_t *)((MPT3SASEventData *)data)->data;
    event_data_length = ((MPT3SASEventData *)data)->length;

    reply = g_malloc0(sizeof(Mpi2EventNotificationReply_t) + event_data_length);
    reply->EventDataLength = cpu_to_le16(event_data_length / 4);
    reply->MsgLength = (sizeof(Mpi2EventNotificationReply_t) + event_data_length) / 4;
    reply->Function = MPI2_FUNCTION_EVENT_NOTIFICATION;
    reply->AckRequired = 1;
    reply->Event = cpu_to_le16(MPI2_EVENT_SAS_DEVICE_STATUS_CHANGE);
    memcpy(reply->EventData, dsd, event_data_length);
    mpt3sas_post_reply(s, 0, 0, (MPI2DefaultReply_t *)reply);
    g_free(reply);
}

/*
 * Send event to host
 */
static int mpt3sas_trigger_event(MPT3SASState *s, uint8_t et, void *data)
{
    trace_mpt3sas_trigger_event(et);
    switch (et) {
        case MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST:
        {
            mpt3sas_event_sas_topology_change_list(s, data);
            return 0;
        }
        case MPI2_EVENT_SAS_DISCOVERY:
        {
            mpt3sas_event_sas_discovery(s, data);
            return 0;
        }
        case MPI2_EVENT_SAS_ENCL_DEVICE_STATUS_CHANGE:
        {
            mpt3sas_event_sas_enclosure_device_status_change(s, data);
            return 0;
        }
        case MPI2_EVENT_SAS_DEVICE_STATUS_CHANGE:
        {
            mpt3sas_event_sas_device_status_change(s, data);
            return 0;
        }
        default:
            trace_mpt3sas_unhandled_event(et);
            return -1;
    }
}

static void mpt3sas_lock_event_queue(MPT3SASEventQueue *queue)
{
    qemu_mutex_lock(&queue->mutex);
}

static void mpt3sas_unlock_event_queue(MPT3SASEventQueue *queue)
{
    qemu_mutex_unlock(&queue->mutex);
}

static MPT3SASEvent *mpt3sas_new_event(uint8_t event, void *data)
{
    MPT3SASEvent *e = g_new0(MPT3SASEvent, 1);

    e->event_type = event;
    e->data = data;
    return e;
}

static void mpt3sas_event_enqueue(MPT3SASState *s, uint16_t event_type, void *event_data)
{

    mpt3sas_lock_event_queue(s->event_queue);
    if (!s->event_queue->exit) {
        MPT3SASEvent *e = mpt3sas_new_event(event_type, event_data);
        QTAILQ_INSERT_TAIL(&s->event_queue->events, e, next);
        qemu_cond_broadcast(&s->event_queue->cond);
    }
    mpt3sas_unlock_event_queue(s->event_queue);
}

static int mpt3sas_event_queue_worker_thread_loop(MPT3SASState *s)
{
    MPT3SASEvent *e;
    MPT3SASEventQueue *queue = s->event_queue;

    mpt3sas_lock_event_queue(queue);
    while (QTAILQ_EMPTY(&queue->events) && !queue->exit) {
        qemu_cond_wait(&queue->cond, &queue->mutex);
    }

    e = QTAILQ_FIRST(&queue->events);
    mpt3sas_unlock_event_queue(queue);

    if (queue->exit) {
        return -1;
    }

    // handle event
    if (mpt3sas_trigger_event(s, e->event_type, e->data) == 1) {
        // continue to handle this event, since this event is waiting another event completed
        return 0;
    }

    mpt3sas_lock_event_queue(queue);
    QTAILQ_REMOVE(&queue->events, e, next);
    mpt3sas_unlock_event_queue(queue);
    qemu_cond_broadcast(&queue->cond);
    g_free(e->data);
    g_free(e);
    return 0;
}

static void mpt3sas_event_queue_init(MPT3SASState *s)
{
    s->event_queue = g_new0(MPT3SASEventQueue, 1);

    qemu_cond_init(&s->event_queue->cond);
    qemu_mutex_init(&s->event_queue->mutex);
    QTAILQ_INIT(&s->event_queue->events);
}

static void mpt3sas_event_queue_clear(MPT3SASState *s)
{
    qemu_cond_destroy(&s->event_queue->cond);
    qemu_mutex_destroy(&s->event_queue->mutex);
    g_free(s->event_queue);
    s->event_queue = NULL;
};

static void *mpt3sas_event_queue_worker_thread(void *arg)
{
    MPT3SASState *s = arg;

    qemu_thread_get_self(&s->event_queue->thread);

    while (!mpt3sas_event_queue_worker_thread_loop(s));
    mpt3sas_event_queue_clear(s);
    return NULL;
}

static void mpt3sas_handle_ioc_facts(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2IOCFactsRequest_t *req)
{
    Mpi2IOCFactsReply_t reply;

    trace_mpt3sas_handle_ioc_facts();
    memset(&reply, 0, sizeof(reply));
    reply.MsgVersion = cpu_to_le16(0x0205);
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.HeaderVersion = cpu_to_le16(0x2a00);
    reply.IOCNumber = 1;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.IOCExceptions = cpu_to_le16(0x1FF); //Report all errors.
    reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_SUCCESS);
    reply.IOCLogInfo = 0;
    reply.MaxChainDepth = MPT3SAS_MAX_CHAIN_DEPTH;
    reply.WhoInit = s->who_init;
    reply.NumberOfPorts = 0x1;
    reply.MaxMSIxVectors = MPT3SAS_MAX_MSIX_VECTORS;
    reply.RequestCredit = cpu_to_le16(MPT3SAS_MAX_OUTSTANDING_REQUESTS);
    reply.ProductID = cpu_to_le16(MPT3SAS_LSI3008_PRODUCT_ID);
    reply.IOCCapabilities = cpu_to_le32(MPI2_IOCFACTS_CAPABILITY_TLR | MPI2_IOCFACTS_CAPABILITY_EEDP | MPI2_IOCFACTS_CAPABILITY_SNAPSHOT_BUFFER | MPI2_IOCFACTS_CAPABILITY_DIAG_TRACE_BUFFER | MPI2_IOCFACTS_CAPABILITY_TASK_SET_FULL_HANDLING | MPI2_IOCFACTS_CAPABILITY_MSI_X_INDEX);// | MPI2_IOCFACTS_CAPABILITY_RDPQ_ARRAY_CAPABLE;
    reply.FWVersion.Struct.Major = 0xd;
    reply.FWVersion.Struct.Minor = 0x0;
    reply.FWVersion.Struct.Unit = 0x0;
    reply.FWVersion.Struct.Dev = 0x0;
    reply.IOCRequestFrameSize = cpu_to_le16(MPT3SAS_REQUEST_FRAME_SIZE);
    reply.IOCMaxChainSegmentSize = 0;
    reply.MaxInitiators = 0x0;
    reply.MaxTargets = cpu_to_le16(s->max_devices + 1);
    reply.MaxSasExpanders = cpu_to_le16(s->expander.count + 1);
    reply.MaxEnclosures = cpu_to_le16(s->expander.count + 1 + 1);
    reply.ProtocolFlags = cpu_to_le16(MPI2_IOCFACTS_PROTOCOL_SCSI_INITIATOR | MPI2_IOCFACTS_PROTOCOL_SCSI_TARGET);
    reply.HighPriorityCredit = cpu_to_le16(124);
    reply.MaxReplyDescriptorPostQueueDepth = cpu_to_le16(MPT3SAS_MAX_REPLY_DESCRIPTOR_QUEUE_DEPTH);
    reply.ReplyFrameSize = MPT3SAS_REPLY_FRAME_SIZE;
    reply.MaxVolumes = 0x0;
    reply.MaxDevHandle = cpu_to_le16(s->max_devices + MPT3SAS_ATTACHED_DEV_HANDLE_START);
    reply.MinDevHandle = cpu_to_le16(MPT3SAS_ATTACHED_DEV_HANDLE_START);
    reply.MaxPersistentEntries = 0x0;
    reply.CurrentHostPageSize = 128;

    if (s->doorbell_state == DOORBELL_WRITE) {
        mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
    } else {
        mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
    }
}

static void mpt3sas_handle_port_facts(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2PortFactsRequest_t *req)
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
    if (req->PortNumber < MPT3SAS_NUM_PHYS) {
        reply.PortType = MPI2_PORTFACTS_PORTTYPE_SAS_PHYSICAL;
        reply.MaxPostedCmdBuffers = cpu_to_le16(128); //TODO
    }

    if (s->doorbell_state == DOORBELL_WRITE) {
        mpt3sas_reply(s, (MPI2DefaultReply_t *)&reply);
    } else {
        mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
    }
}

static void mpt3sas_handle_ioc_init(MPT3SASState *s, Mpi2IOCInitRequest_t *req)
{
    Mpi2IOCInitReply_t reply;

    s->who_init = req->WhoInit;
    s->host_page_size = req->HostPageSize;
    s->host_msix_vectors = req->HostMSIxVectors;
    s->system_request_frame_size = le16_to_cpu(req->SystemRequestFrameSize);
    s->reply_descriptor_post_queue_depth = le16_to_cpu(req->ReplyDescriptorPostQueueDepth);
    s->reply_free_queue_depth = le16_to_cpu(req->ReplyFreeQueueDepth);
    s->sense_buffer_address_hi = le32_to_cpu(req->SenseBufferAddressHigh);
    s->system_reply_address_hi = le32_to_cpu(req->SystemReplyAddressHigh);
    s->system_request_frame_base_address = le64_to_cpu(req->SystemRequestFrameBaseAddress);
    s->reply_descriptor_post_queue_address = le64_to_cpu(req->ReplyDescriptorPostQueueAddress);
    s->reply_free_queue_address = le64_to_cpu(req->ReplyFreeQueueAddress);

    trace_mpt3sas_ioc_init(s->system_request_frame_size, s->reply_descriptor_post_queue_depth, s->reply_free_queue_depth,
            s->system_request_frame_base_address, s->reply_descriptor_post_queue_address, s->reply_free_queue_address, s->system_reply_address_hi);

    if (s->state == MPI2_IOC_STATE_READY) {
        s->state = MPI2_IOC_STATE_OPERATIONAL;
    }

    if (s->msix_in_use) {
        int i;
        for (i = 0; i < s->host_msix_vectors; i++) {
            s->reply_post[i].base = s->reply_descriptor_post_queue_address + i * sizeof(uint64_t) * s->reply_descriptor_post_queue_depth;
        }
    } else {
        s->reply_post[0].base = s->reply_descriptor_post_queue_address;
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

static void mpt3sas_handle_event_notification(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2EventNotificationRequest_t *req)
{
    Mpi2EventNotificationReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.EventDataLength = cpu_to_le16(sizeof(reply.EventData) / 4);
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.Event = cpu_to_le16(MPI2_EVENT_EVENT_CHANGE);
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_config(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2ConfigRequest_t *req)
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
            req->ExtPageType, req->ExtPageLength, le32_to_cpu(req->PageAddress));

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

    type = req->Header.PageType & MPI2_CONFIG_PAGETYPE_MASK;
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
        case MPI2_CONFIG_ACTION_PAGE_READ_DEFAULT:
        case MPI2_CONFIG_ACTION_PAGE_READ_CURRENT:
        case MPI2_CONFIG_ACTION_PAGE_WRITE_CURRENT:
        case MPI2_CONFIG_ACTION_PAGE_WRITE_NVRAM:
            break;
        default:
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_ACTION);
            goto out;
    }

    if (!page) {
        // Just find out the reason why fails to find the page
        page = mpt3sas_find_config_page(type, 1);
        if (page) {
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_PAGE);
        } else {
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_TYPE);
            trace_mpt3sas_handle_config_invalid_page(req->Action, req->Header.PageType,
                req->Header.PageNumber, req->Header.PageLength, req->Header.PageVersion,
                req->ExtPageType, req->ExtPageLength, le32_to_cpu(req->PageAddress));
        }
        goto out;
    }

    if (req->Action == MPI2_CONFIG_ACTION_PAGE_DEFAULT ||
        req->Action == MPI2_CONFIG_ACTION_PAGE_HEADER) {
        length = page->mpt_config_build(s, NULL, req->PageAddress);
        if ((ssize_t)length < 0) {
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_PAGE);
            goto out;
        } else {
            goto done;
        }
    }

    if (req->Action == MPI2_CONFIG_ACTION_PAGE_WRITE_CURRENT ||
        req->Action == MPI2_CONFIG_ACTION_PAGE_WRITE_NVRAM) {
        length = page->mpt_config_build(s, NULL, req->PageAddress);
        if ((ssize_t)length < 0) {
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_PAGE);
        } else {
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_CANT_COMMIT);
        }
        goto out;
    }

    assert(req->ChainOffset == 0);

    //determin SGL type
    if ((req->SGLFlags & MPI2_SGLFLAGS_SGL_TYPE_MASK) == MPI2_SGLFLAGS_SGL_TYPE_MPI) {
        uint32_t flags_and_length = req->PageBufferSGE.MpiSimple.FlagsLength;
        uint8_t flags = (flags_and_length) >> 24;
        dmalen = flags_and_length & MPI2_SGE_LENGTH_MASK;
        if (flags & MPI2_SGE_FLAGS_64_BIT_ADDRESSING) {
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
            reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_PAGE);
            goto out;
        } else {
            goto done;
        }
    }

    length = page->mpt_config_build(s, &data, le32_to_cpu(req->PageAddress));

    if (data) {
        uint8_t i = 0;
        char buf[1024] = {0};
        uint32_t buf_len = 0;

        buf_len += sprintf(buf + buf_len, "Page(%x/%x/%x) Ext Page(%x/%x) Page Address 0x%x: \n",
                req->Header.PageType, req->Header.PageNumber, req->Header.PageLength,
                req->ExtPageType, req->ExtPageLength, req->PageAddress);
        for (i = 0; i < length; i++) {
            if (i && (i % 8) == 0) {
                buf_len += sprintf(buf + buf_len, "\n");
            }

            buf_len += sprintf(buf + buf_len, "%02x ", data[i]);
        }
        buf_len += sprintf(buf + buf_len, "\n");
        trace_mpt3sas_dump_buffer(buf);
    }

    if ((ssize_t)length < 0) {
        reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_CONFIG_INVALID_PAGE);
        goto out;
    } else {
        assert(data[2] == page->number);
        reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_SUCCESS);
        trace_mpt3sas_config_page_write(req->Header.PageType, req->Header.PageNumber, pa, MIN(length, dmalen));
        smp_wmb();
        pci_dma_write(pci, pa, data, MIN(length, dmalen));
        goto done;
    }

    abort();

done:
    if (type > MPI2_CONFIG_PAGETYPE_EXTENDED) {
        reply.ExtPageLength = cpu_to_le16(length / 4);
        reply.ExtPageType = req->ExtPageType;
    } else {
        reply.Header.PageLength = length / 4;
    }

    reply.IOCStatus = cpu_to_le16(MPI2_IOCSTATUS_SUCCESS);
out:
    if (reply.IOCStatus != MPI2_IOCSTATUS_SUCCESS) {
        reply.Header.PageLength = 0;
        reply.ExtPageLength = 0;
        trace_mpt3sas_handle_config_error(req->Action, req->Header.PageType,
            req->Header.PageNumber, req->Header.PageLength, req->Header.PageVersion, 
            req->ExtPageType, req->ExtPageLength, le32_to_cpu(req->PageAddress), reply.IOCStatus);
    }

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
    if (data)
        g_free(data);
    return ;
}

static void mpt3sas_handle_port_enable(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2PortEnableRequest_t *req)
{
    Mpi2PortEnableReply_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.PortFlags = req->PortFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
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

        if (req->scsi_io.DataLength == 0)  { // This is a zero len page
           if ((flags & 0xc0) ==  MPI25_IEEE_SGE_FLAGS_END_OF_LIST) { // if this is a IEEE SG format
                addr = ldl_le_pci_dma(pci, sgaddr) | (hwaddr)ldl_le_pci_dma(pci, sgaddr + 4) << 32;
                trace_mpt3sas_ieee_sgl_zero_len("ieee", addr, flags);
                qemu_sglist_add(&req->qsg, addr, 0);
                break;
            } else { // if this is a MPI SG format, some guest os used MPI SG format to build zero len table
                uint32_t flags_length = ldl_le_pci_dma(pci, sgaddr);
                addr = ldl_le_pci_dma(pci, sgaddr + 4) | (hwaddr)ldl_le_pci_dma(pci, sgaddr + 8) << 32;

                trace_mpt3sas_ieee_sgl_zero_len("mpi", addr, flags_length >> 24);
                if ((flags_length >> MPI2_SGE_FLAGS_SHIFT) & MPI2_SGE_FLAGS_END_OF_LIST) {
                    qemu_sglist_add(&req->qsg, addr, 0);
                    break;
                }
            }
        }

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

static void dump_cdb(uint8_t *buf, int len)
{
    int cdb_len = 0;
    char cdb[256] = {0};
    int i = 0;

    for (i = 0; i < len; i++) {
        cdb_len += sprintf(cdb + cdb_len, "%02x ", buf[i]);
    }

    trace_mpt3sas_scsi_command_cdb(len, cdb);
}

static int mpt3sas_handle_scsi_io_request(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi25SCSIIORequest_t *req, hwaddr addr)
{

    MPT3SASRequest *mpt3sas_req = NULL;
    int status;
    SCSIDevice *sdev;
    Mpi2SCSIIOReply_t reply;

    trace_mpt3sas_scsi_io_request(req->DevHandle, req->LUN[1], req->SkipCount,
            req->DataLength,
            req->BidirectionalDataLength,
            req->IoFlags,
            req->EEDPFlags,
            req->EEDPBlockSize,
            req->Control, smid);
    mpt3sas_print_scsi_devices(&s->bus);
    status = mpt3sas_scsi_device_find(s, req->DevHandle, req->LUN, &sdev);
    if (status) {
        goto bad;
    }

    mpt3sas_req = g_new0(MPT3SASRequest, 1);
    QTAILQ_INSERT_TAIL(&s->pending, mpt3sas_req, next);
    mpt3sas_req->scsi_io = *req;
    mpt3sas_req->dev = s;
    mpt3sas_req->smid = smid;
    mpt3sas_req->msix_index = msix_index;
    trace_mpt3sas_scsi_io_command_info(req, req->CDB.CDB32[0], smid);

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

    dump_cdb(req->CDB.CDB32, req->IoFlags & 0xff);

    mpt3sas_req->sreq = scsi_req_new(sdev, 0, req->LUN[1], req->CDB.CDB32, mpt3sas_req);

#if 0
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
#endif

    if (scsi_req_enqueue(mpt3sas_req->sreq)) {
        scsi_req_continue(mpt3sas_req->sreq);
    }

    return 0;

//overrun:
    trace_mpt3sas_scsi_io_overrun(sdev->wwn, req->Control, req->DataLength, mpt3sas_req->sreq->cmd.xfer, mpt3sas_req->sreq->cmd.mode);
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

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
    trace_mpt3sas_scsi_io_error(status);
    return status;
}

static void mpt3sas_handle_event_ack(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2EventAckRequest_t *req)
{
    Mpi2EventAckReply_t reply;

    trace_mpt3sas_handle_event_ack(smid, le16_to_cpu(req->Event), le32_to_cpu(req->EventContext));
    memset(&reply, 0, sizeof(reply));
    reply.Function = req->Function;
    reply.MsgFlags = req->MsgFlags;
    reply.MsgLength = sizeof(reply) / 4;
    reply.VF_ID = req->VF_ID;
    reply.VP_ID = req->VP_ID;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_scsi_task_management(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2SCSITaskManagementRequest_t *req)
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
                    notifier->smid = smid;
                    notifier->msix_index = msix_index;
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
                    notifier->smid = smid;
                    notifier->msix_index = msix_index;
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
                if (sdev->channel == 0 && sdev->id == HANDLE_TO_SCSI_ID(req->DevHandle)) {
                    qdev_reset_all(kid->child);
                }
            }
            break;
        default:
            reply.ResponseCode = MPI2_SCSITASKMGMT_RSP_TM_NOT_SUPPORTED;
            break;
    }
out:
    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);

}

//FIXME: currently just respond the driver, no data were transfered to host.
static void mpt3sas_handle_fw_upload(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2FWUploadRequest_t *req)
{
    Mpi2FWUploadReply_t reply;

    trace_mpt3sas_handle_fw_upload(smid, msix_index, req->ImageType, req->ChainOffset);

    memset(&reply, 0, sizeof(reply));
    reply.Function = req->Function;
    reply.MsgLength = sizeof(reply) / 4;
    reply.ImageType = req->ImageType;
    reply.MsgFlags = req->MsgFlags;
    reply.VF_ID = req->VF_ID;
    reply.VP_ID = req->VP_ID;

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
}

struct rep_manu_request {
    uint8_t smp_frame_type;
    uint8_t function;
    uint8_t reserved;
    uint8_t request_length;
};

struct rep_manu_reply {
    uint8_t smp_frame_type;
    uint8_t function;
    uint8_t function_result;
    uint8_t response_length;
    uint16_t expander_change_count;
    uint8_t reserved0[2];
    uint8_t sas_format;
    uint8_t reserved2[3];
    #define SAS_EXPANDER_VENDOR_ID_LEN  8
    uint8_t vendor_id[SAS_EXPANDER_VENDOR_ID_LEN];
    #define SAS_EXPANDER_PRODUCT_ID_LEN 16
    uint8_t product_id[SAS_EXPANDER_PRODUCT_ID_LEN];
    #define SAS_EXPANDER_PRODUCT_REV_LEN    4
    uint8_t product_rev[SAS_EXPANDER_PRODUCT_REV_LEN];
    #define SAS_EXPANDER_COMPONENT_VENDOR_ID_LEN    8
    uint8_t component_vendor_id[SAS_EXPANDER_COMPONENT_VENDOR_ID_LEN];
    uint16_t component_id;
    uint8_t component_revision_id;
    uint8_t reserved3;
    uint8_t vendor_specific[8];
};

static void mpt3sas_handle_smp_passthrough(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2SmpPassthroughRequest_t *req)
{
    Mpi2SmpPassthroughReply_t reply;
    uint32_t datalen = 0;
    uint64_t pa = 0;
    uint8_t smp_frame_type = 0;
    uint8_t function = 0;

    trace_mpt3sas_handle_smp_passthrough(req->PassthroughFlags, req->PhysicalPort, req->RequestDataLength,
            req->SGLFlags, req->SASAddress);
    memset(&reply, 0, sizeof(reply));

    if ((req->SGLFlags & MPI2_SGLFLAGS_SGL_TYPE_MASK) == MPI2_SGLFLAGS_SGL_TYPE_MPI) {
        uint32_t flags_and_length = req->SGL.MpiSimple.FlagsLength;
        uint8_t flags = (flags_and_length) >> 24;
        datalen = flags_and_length & MPI2_SGE_LENGTH_MASK;
        if (flags & MPI2_SGE_FLAGS_64_BIT_ADDRESSING) {
            pa = req->SGL.MpiSimple.u.Address64;
        } else {
            pa = req->SGL.MpiSimple.u.Address32;
        }
    } else if ((req->SGLFlags & MPI2_SGLFLAGS_SGL_TYPE_MASK) == MPI2_SGLFLAGS_SGL_TYPE_IEEE64)  {
        datalen = req->SGL.IeeeSimple.Simple64.Length;
        pa = req->SGL.IeeeSimple.Simple64.Address;
    }

    // Read SMP request

    // read 2 bytes to determine what SMP frame is
    smp_frame_type = ldub_pci_dma(PCI_DEVICE(s), pa);
    function = ldub_pci_dma(PCI_DEVICE(s), pa + 1);

    trace_mpt3sas_handle_smp_frame(smp_frame_type, function, pa, datalen);
    if (smp_frame_type == 0x40) {
        switch (function) {
            case 0x1: // REPORT MANUFACTURE
            {
                struct rep_manu_request rmreq;
                struct rep_manu_reply rmrep;
                memset(&rmreq, 0, sizeof(rmreq));

                // read the whole smp frame
                pci_dma_read(PCI_DEVICE(s), pa, &rmreq, sizeof(rmreq));

                memset(&rmrep, 0, sizeof(rmrep));

                // prepare report manufacture response
                rmrep.smp_frame_type = 0x41;
                rmrep.function = 0x1;
                strcpy((void *)rmrep.vendor_id, "100");
                strcpy((void *)rmrep.product_id, "500888899991111");
                strcpy((void *)rmrep.product_rev, "001");

                // write the response to host through DMA
                pci_dma_write(PCI_DEVICE(s), pa + sizeof(rmreq), &rmrep, sizeof(rmrep));
                break;
            }
            default:
                trace_mpt3sas_unhandled_smp_frame(smp_frame_type, function);
                break;
        }
    } else {
        trace_mpt3sas_unhandled_smp_frame(smp_frame_type, function);
    }

    reply.Function = req->Function;
    reply.PassthroughFlags = req->PassthroughFlags;
    reply.PhysicalPort = req->PhysicalPort;
    reply.MsgLength = sizeof(reply) / 4;
    reply.MsgFlags = req->MsgFlags;
    reply.VF_ID = req->VF_ID;
    reply.VP_ID = req->VP_ID;
    reply.ResponseDataLength = sizeof(struct rep_manu_reply);

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
}

static void mpt3sas_handle_sas_io_unit_control(MPT3SASState *s, uint16_t smid, uint8_t msix_index, Mpi2SasIoUnitControlRequest_t *req)
{
    Mpi2SasIoUnitControlReply_t reply;

    trace_mpt3sas_handle_sas_io_unit_control(req->Operation, req->Function, req->DevHandle, req->PhyNum, req->LookupMethod, req->LookupAddress);

    if (req->Operation == MPI2_SAS_OP_LOOKUP_MAPPING) {
        //TODO: lookup dev handle and fill it to reply
        return;
    }

    memset(&reply, 0, sizeof(reply));
    reply.Operation = req->Operation;
    reply.MsgLength = sizeof(reply) / 4;
    reply.Function = req->Function;
    reply.IOCParameter = req->IOCParameter;
    reply.MsgFlags = req->MsgFlags;
    reply.VP_ID = req->VP_ID;
    reply.VF_ID = req->VF_ID;
    reply.IOCStatus = MPI2_IOCSTATUS_SUCCESS;

    mpt3sas_post_reply(s, smid, msix_index, (MPI2DefaultReply_t *)&reply);
}

/*
 * Handle doorbell message. Bascially IOCInit, IOCFacts, and PortFacts will be transfered
 * through doorbell register, but later, when all the queue are setup by host driver, the host
 * might use request queue to send commands for IOCTL.
 * When using doorbell register to transfer requests. smid and msix_index are useless.
 */
static void mpt3sas_handle_message(MPT3SASState *s, uint16_t smid, uint8_t msix_index, MPI2RequestHeader_t *req)
{
    trace_mpt3sas_handle_message(req->Function);
    switch (req->Function) {
        case MPI2_FUNCTION_IOC_INIT:
            mpt3sas_handle_ioc_init(s, (Mpi2IOCInitRequest_t *)req);
            break;
        case MPI2_FUNCTION_IOC_FACTS:
            mpt3sas_handle_ioc_facts(s, smid, msix_index, (Mpi2IOCFactsRequest_t *)req);
            break;
        case MPI2_FUNCTION_PORT_FACTS:
            mpt3sas_handle_port_facts(s, smid, msix_index, (Mpi2PortFactsRequest_t *)req);
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
    memset(s->reply_post, 0, ARRAY_SIZE(s->reply_post) * sizeof(MPT3SASReplyPost));
    s->reply_free_ioc_index = 0;
    s->reply_free_host_index = 0;

    s->request_descriptor_post_head = 0;
    s->request_descriptor_post_tail = 0;
    memset(s->request_descriptor_post, 0, ARRAY_SIZE(s->request_descriptor_post));

    memset(s->completed_queue, 0, ARRAY_SIZE(s->completed_queue));
    s->completed_queue_head = 0;
    s->completed_queue_tail = 0;

    //s->sas_address = 0x51866da05a753a00;
    qemu_bh_cancel(s->request_bh);
    qemu_bh_cancel(s->completed_request_bh);

    s->host_page_size = 0;
    s->host_msix_vectors = 0;
    s->system_request_frame_size = 0;
    s->reply_descriptor_post_queue_depth = 0;
    s->reply_free_queue_depth = 0;
    s->sense_buffer_address_hi = 0;
    s->system_reply_address_hi = 0;
    s->system_request_frame_base_address = 0;
    s->reply_descriptor_post_queue_address = 0;
    s->reply_free_queue_address = 0;

    s->ioc_reset = 0;

    s->state = MPI2_IOC_STATE_READY;
}

static void mpt3sas_hard_reset(MPT3SASState *s)
{
    s->state = MPI2_IOC_STATE_RESET;
    mpt3sas_soft_reset(s);
    s->intr_mask = MPI2_HIM_RESET_IRQ_MASK | MPI2_HIM_DIM | MPI2_HIM_DIM;
    s->max_devices = 0x220;
    s->max_buses = 1;
}

static void mpt3sas_send_discovery_event(MPT3SASState *s, uint32_t rc)
{
    MPT3SASEventData *event_data = NULL;

    event_data = g_malloc(sizeof(MPT3SASEventData) + sizeof(MPI2_EVENT_DATA_SAS_DISCOVERY));
    memset(event_data, 0, sizeof(MPT3SASEventData) + sizeof(MPI2_EVENT_DATA_SAS_DISCOVERY));
    event_data->length = sizeof(MPI2_EVENT_DATA_SAS_DISCOVERY);
    pMpi2EventDataSasDiscovery_t sas_discovery_data = (pMpi2EventDataSasDiscovery_t)event_data->data;
    sas_discovery_data->ReasonCode = rc;
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_DISCOVERY, event_data);
}

static void mpt3sas_enclosure_device_status_change_event_enqueue(MPT3SASState *s, uint16_t enclosure_handle,
                                                        uint32_t rc, uint8_t physical_port,
                                                        uint64_t enclosure_logical_id)
{
    MPT3SASEventData *event_data1 = NULL;
    pMpi2EventDataSasEnclDevStatusChange_t sas_encl_dev_status_change = NULL;

    event_data1 = g_malloc(sizeof(MPT3SASEventData) + sizeof(MPI2_EVENT_DATA_SAS_ENCL_DEV_STATUS_CHANGE));
    memset(event_data1, 0, sizeof(MPT3SASEventData) + sizeof(MPI2_EVENT_DATA_SAS_ENCL_DEV_STATUS_CHANGE));
    event_data1->length = sizeof(MPI2_EVENT_DATA_SAS_ENCL_DEV_STATUS_CHANGE);
    sas_encl_dev_status_change = (pMpi2EventDataSasEnclDevStatusChange_t)event_data1->data;
    sas_encl_dev_status_change->EnclosureHandle = cpu_to_le16(enclosure_handle);
    sas_encl_dev_status_change->ReasonCode = rc;
    sas_encl_dev_status_change->PhysicalPort = physical_port;
    sas_encl_dev_status_change->EnclosureLogicalID = cpu_to_le64(enclosure_logical_id);
    sas_encl_dev_status_change->NumSlots = s->expander.downstream_phys;
    sas_encl_dev_status_change->StartSlot = 0x0;
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_ENCL_DEVICE_STATUS_CHANGE, event_data1);

}

static void mpt3sas_phy_change_list_event_enqueue(MPT3SASState *s, uint16_t enclosure_handle,
                                            uint16_t expander_dev_handle, uint16_t attached_dev_handle,
                                            uint32_t num_phys, uint32_t num_entries,
                                            uint32_t start_phy_num, uint32_t exp_status,
                                            uint32_t physical_port, uint32_t phy_status)
{
    MPT3SASEventData *event_data2 = NULL;
    int i = 0;
    uint32_t event_data_length = offsetof(Mpi2EventDataSasTopologyChangeList_t, PHY) + sizeof(MPI2_EVENT_SAS_TOPO_PHY_ENTRY) * MPT3SAS_NUM_PHYS;

    event_data2 = g_malloc(sizeof(MPT3SASEventData) + event_data_length);
    memset(event_data2, 0, sizeof(MPT3SASEventData) + event_data_length);
    event_data2->length = event_data_length;
    pMpi2EventDataSasTopologyChangeList_t sas_topology_change_list = NULL;
    sas_topology_change_list = (pMpi2EventDataSasTopologyChangeList_t)event_data2->data;
    sas_topology_change_list->EnclosureHandle = enclosure_handle;
    sas_topology_change_list->ExpanderDevHandle = expander_dev_handle;
    sas_topology_change_list->NumPhys = num_phys;
    sas_topology_change_list->NumEntries = num_entries;
    sas_topology_change_list->StartPhyNum = start_phy_num;
    sas_topology_change_list->ExpStatus = exp_status;
    sas_topology_change_list->PhysicalPort = physical_port;

    for (i = 0; i < sas_topology_change_list->NumEntries; i++) {
        sas_topology_change_list->PHY[i].AttachedDevHandle = attached_dev_handle;
        sas_topology_change_list->PHY[i].LinkRate = MPI25_EVENT_SAS_TOPO_LR_RATE_12_0 << MPI2_EVENT_SAS_TOPO_LR_CURRENT_SHIFT;
        sas_topology_change_list->PHY[i].PhyStatus = phy_status;
    }

    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST, event_data2);
}

static int mpt3sas_disk_change_list_event_enqueue(MPT3SASState *s, uint16_t enclosure_handle,
                                            uint16_t expander_dev_handle, uint32_t num_phys,
                                            uint32_t start_dev_idx, uint32_t exp_status,
                                            uint32_t physical_port, uint32_t phy_status)
{
    MPT3SASEventData *event_data2 = NULL;
    uint32_t i = 0;
    SCSIDevice *sdev = NULL;
    uint8_t expander_idx = expander_dev_handle - MPT3SAS_EXPANDER_HANDLE_START;
    uint8_t entries = 0;

    uint32_t event_data_length = offsetof(Mpi2EventDataSasTopologyChangeList_t, PHY) + sizeof(MPI2_EVENT_SAS_TOPO_PHY_ENTRY) * MPT3SAS_NUM_PHYS;
    event_data2 = g_malloc(sizeof(MPT3SASEventData) + event_data_length);
    memset(event_data2, 0, sizeof(MPT3SASEventData) + event_data_length);
    event_data2->length = event_data_length;
    pMpi2EventDataSasTopologyChangeList_t sas_topology_change_list = NULL;
    sas_topology_change_list = (pMpi2EventDataSasTopologyChangeList_t)event_data2->data;
    sas_topology_change_list->EnclosureHandle = enclosure_handle;
    sas_topology_change_list->ExpanderDevHandle = expander_dev_handle;
    sas_topology_change_list->NumPhys = num_phys;
    sas_topology_change_list->NumEntries = 0;
    sas_topology_change_list->StartPhyNum = SCSI_ID_TO_EXP_PHY(s, start_dev_idx);
    sas_topology_change_list->ExpStatus = exp_status;
    sas_topology_change_list->PhysicalPort = physical_port; /* port associated with expander port */

    i = start_dev_idx;

    while (i < (expander_idx + 1) * s->expander.downstream_phys) {
        if ((sdev = scsi_device_find(&s->bus, 0, i++, 0)) == NULL) 
            break;

        uint16_t dev_handle = SCSI_ID_TO_HANDLE(sdev->id);

        trace_mpt3sas_event_add_device(sdev, dev_handle, sdev->id, SCSI_ID_TO_EXP_PHY(s, sdev->id));

        sas_topology_change_list->PHY[entries].AttachedDevHandle = cpu_to_le16(dev_handle);
        sas_topology_change_list->PHY[entries].LinkRate = MPI25_EVENT_SAS_TOPO_LR_RATE_12_0 << MPI2_EVENT_SAS_TOPO_LR_CURRENT_SHIFT;
        sas_topology_change_list->PHY[entries].PhyStatus = phy_status;
        entries++;
    }

    if (!entries) {
        g_free(event_data2);
        return i;
    }

    //update entries
    sas_topology_change_list->NumEntries = entries;
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST, event_data2);

    //return end phy
    return i;
}

static void mpt3sas_sas_device_status_change_event_enqueue(MPT3SASState *s, uint32_t rc, int scsi_id) {
    MPT3SASEventData *event_data = NULL;
    uint8_t dev_handle = SCSI_ID_TO_HANDLE(scsi_id);
    event_data = g_malloc0(sizeof(MPT3SASEventData) + sizeof(MPI2_EVENT_DATA_SAS_DEVICE_STATUS_CHANGE));
    event_data->length = sizeof(MPI2_EVENT_DATA_SAS_DEVICE_STATUS_CHANGE);
    pMpi2EventDataSasDeviceStatusChange_t sas_dev_stat_change_data = (pMpi2EventDataSasDeviceStatusChange_t)event_data->data;
    sas_dev_stat_change_data->TaskTag = 0xFFFF;
    sas_dev_stat_change_data->ReasonCode = rc;
    sas_dev_stat_change_data->PhysicalPort = 0x0;
    sas_dev_stat_change_data->DevHandle = cpu_to_le16(dev_handle);
    sas_dev_stat_change_data->SASAddress = s->sas_address;
    memset(sas_dev_stat_change_data->LUN, 0xFF, sizeof(U8) * 8);
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_DEVICE_STATUS_CHANGE, event_data);
}

// report all devices to guest driver
static void mpt3sas_add_events(MPT3SASState *s)
{
    uint16_t enclosure_handle = MPT3SAS_ENCLOSURE_HANDLE_START;
    uint32_t expander;

    /* Trigger the following events
     *
     * If supporting host-based discovery, then qemu SHOULD _NOT_ trigger SAS DISCOVERY and SAS TOPOLOGY
     * CHANGE LIST events.
     */

    /* 1. notify driver sas discovery start  */
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_STARTED);

    /* 2. notify driver sas enclosure device status change (hba enclosure) */
    mpt3sas_enclosure_device_status_change_event_enqueue(s, enclosure_handle,
                                                MPI2_EVENT_SAS_ENCL_RC_ADDED,
                                                0, /*Physical port*/
                                                s->sas_address);


    enclosure_handle++;

    for (expander = 0; expander < s->expander.count; expander++) {
        uint32_t scsi_id_idx = expander * s->expander.downstream_phys;
        bool exp_added = false;

        /* notify guest driver sas topology change (HBA PHY Change) */
        mpt3sas_phy_change_list_event_enqueue(s, enclosure_handle,
                                           0x0, /* expander device handle */
                                           MPT3SAS_EXPANDER_HANDLE_START + expander, /* attached device handle */
                                           MPT3SAS_NUM_PHYS, /* number phys */
                                           s->expander.upstream_phys, /* number entries */
                                           expander * s->expander.upstream_phys, /* start phy number */
                                           MPI2_EVENT_SAS_TOPO_ES_RESPONDING,
                                           expander, /* physical port */
                                           MPI2_EVENT_SAS_TOPO_RC_PHY_CHANGED);


        /* notify guest driver sas enclosure device status change (Expander enclosure) */
        mpt3sas_enclosure_device_status_change_event_enqueue(s, enclosure_handle + expander,
                                                    MPI2_EVENT_SAS_ENCL_RC_ADDED,
                                                    expander, /* Physical Port */
                                                    MPT3SAS_EXPANDER_DEFAULT_SAS_ADDR + expander);


        /* notify guest driver sas topology change(report expander PHY), add expander and drives */
        while (scsi_id_idx != (expander + 1) * s->expander.downstream_phys) {
            scsi_id_idx = mpt3sas_disk_change_list_event_enqueue(s, enclosure_handle + expander,
                                            MPT3SAS_EXPANDER_HANDLE_START + expander, /* expander handle */
                                            s->expander.all_phys, /* num phys */
                                            scsi_id_idx, /* start phy number */
                                            exp_added ? MPI2_EVENT_SAS_TOPO_ES_RESPONDING : MPI2_EVENT_SAS_TOPO_ES_ADDED,
                                            expander, /* physical port */
                                            MPI2_EVENT_SAS_TOPO_RC_TARG_ADDED); 
            exp_added = true;
        }

        /* send change list event for phys that forming the wide port */
        mpt3sas_phy_change_list_event_enqueue(s, enclosure_handle + expander,
                                           MPT3SAS_EXPANDER_HANDLE_START + expander, /* expander device handle */
                                           MPT3SAS_IOC_HANDLE_START + expander * s->expander.upstream_phys, /* attached device handle */
                                           s->expander.all_phys,  /* number phys */
                                           s->expander.upstream_phys, /* number entries */
                                           s->expander.upstream_start_phy, /* start phy number */
                                           MPI2_EVENT_SAS_TOPO_ES_RESPONDING,
                                           expander,
                                           MPI2_EVENT_SAS_TOPO_RC_PHY_CHANGED);
    }

    /* notify drive sas discovery completed */
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_COMPLETED);

}

static void mpt3sas_handle_request(MPT3SASState *s)
{
    PCIDevice *pci = (PCIDevice *)s;
    uint8_t req[MPT3SAS_REQUEST_FRAME_SIZE * 4] = {0};
    MPI2RequestHeader_t *hdr = (MPI2RequestHeader_t *)req;
    hwaddr addr;
    int size;
    uint64_t mpi_request_descriptor = 0;
    uint8_t request_flags = 0;
    uint8_t msix_index = 0;
    uint16_t smid = 0;
    const char *request_desc = NULL;

    memset(req, 0, sizeof(req));
    mpi_request_descriptor = s->request_descriptor_post[s->request_descriptor_post_head++];
    s->request_descriptor_post_head %= ARRAY_SIZE(s->request_descriptor_post);
    request_flags = mpi_request_descriptor & 0xff;
    msix_index = (mpi_request_descriptor >> 8) & 0xff;
    smid = le16_to_cpu((mpi_request_descriptor >> 16) & 0xffff);
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

    addr = s->system_request_frame_base_address + (s->system_request_frame_size * 4) * smid;

    // Read request header from system request message frames queue
    pci_dma_read(pci, addr, req, sizeof(MPI2RequestHeader_t));

    trace_mpt3sas_handle_request(request_desc, smid, msix_index, hdr->Function, mpi2_request_sizes[hdr->Function]);
    if (hdr->Function < ARRAY_SIZE(mpi2_request_sizes) &&
        mpi2_request_sizes[hdr->Function]) {

        size = mpi2_request_sizes[hdr->Function];

        // Workaround in case the Guest OS uses IEEE SGL for FWUPLoad
        // FIXME here
        if (hdr->Function == MPI2_FUNCTION_FW_UPLOAD) {
            size += 4;
        }

        assert(size <= MPT3SAS_REQUEST_FRAME_SIZE * 4);
        pci_dma_read(pci, addr + sizeof(hdr), &req[sizeof(hdr)],
                size - sizeof(hdr));
        if (size > 0) {
            uint32_t i = 0;
            int buf_len = 0;
            char buf[1024] = {0};

            buf_len += sprintf(buf + buf_len, "Request (0x%lx/0x%x): \n", addr, size);
            for (i = 0; i < size; i++) {
                if (i && (i % 8 == 0)) {
                    buf_len += sprintf(buf + buf_len, "\n");
                }
                buf_len += sprintf(buf + buf_len, "%02x ", req[i]);
            }
            buf_len += sprintf(buf + buf_len, "\n");
            trace_mpt3sas_dump_buffer(buf);
        }
    }

    switch (hdr->Function) {
        case MPI2_FUNCTION_EVENT_NOTIFICATION:
            mpt3sas_handle_event_notification(s, smid, msix_index, (Mpi2EventNotificationRequest_t *)req);
            break;
        case MPI2_FUNCTION_SCSI_IO_REQUEST:
            mpt3sas_handle_scsi_io_request(s, smid, msix_index, (Mpi25SCSIIORequest_t *)req, addr);
            break;
        case MPI2_FUNCTION_PORT_ENABLE:
            mpt3sas_handle_port_enable(s, smid, msix_index, (Mpi2PortEnableRequest_t *)req);
            mpt3sas_add_events(s);
            break;
        case MPI2_FUNCTION_CONFIG:
            mpt3sas_handle_config(s, smid, msix_index, (Mpi2ConfigRequest_t *)req);
            break;
        case MPI2_FUNCTION_EVENT_ACK:
            mpt3sas_handle_event_ack(s, smid, msix_index, (Mpi2EventAckRequest_t *)req);
            break;
        case MPI2_FUNCTION_SCSI_TASK_MGMT:
            mpt3sas_handle_scsi_task_management(s, smid, msix_index, (Mpi2SCSITaskManagementRequest_t *)req);
            break;
        case MPI2_FUNCTION_FW_UPLOAD:
            mpt3sas_handle_fw_upload(s, smid, msix_index, (Mpi2FWUploadRequest_t *)req);
            break;
        case MPI2_FUNCTION_SMP_PASSTHROUGH:
            mpt3sas_handle_smp_passthrough(s, smid, msix_index, (Mpi2SmpPassthroughRequest_t *)req);
            break;
        case MPI2_FUNCTION_SAS_IO_UNIT_CONTROL:
            mpt3sas_handle_sas_io_unit_control(s, smid, msix_index, (Mpi2SasIoUnitControlRequest_t *)req);
            break;
        default:
            mpt3sas_handle_message(s, smid, msix_index, (MPI2RequestHeader_t *)req);
            break;
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

// TO emulate interrupt coalescing
static void mpt3sas_handle_completed_request(void *opaque)
{
    MPT3SASState *s = opaque;
    int processed = 0;
    uint64_t delta = 0;
    struct timeval start, now;
    //uint32_t reply_post_queue_len = 0;

    //reply_post_queue_len = (s->reply_post_ioc_index - s->reply_post_host_index + s->reply_descriptor_post_queue_depth) % s->reply_descriptor_post_queue_depth;

    gettimeofday(&start, NULL);
    while (s->completed_queue_head != s->completed_queue_tail) {
        MPT3SASRequest *req = s->completed_queue[s->completed_queue_head];

        mpt3sas_scsi_io_reply(s, req->smid, req->msix_index);
        s->completed_commands++;
        trace_mpt3sas_scsi_io_command_completed(req, req->smid, req->scsi_io.CDB.CDB32[0], s->completed_commands);
        mpt3sas_free_request(req);
        s->completed_queue[s->completed_queue_head] = NULL;
        s->completed_queue_head++;
        s->completed_queue_head %= ARRAY_SIZE(s->completed_queue);
        processed++;

        gettimeofday(&now, NULL);

        // If no interrupt is raised within 500ms, then raise an interrupt for host driver to
        // call ISR, and avoid request timeout.
        delta = (now.tv_sec * 1000 * 1000 + now.tv_usec) - (start.tv_sec * 1000 * 1000 + now.tv_usec);
        if (delta > 500000 && processed > 0 && !(s->intr_status & MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT)) {
            s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
            mpt3sas_update_interrupt(s);

            processed = 0;

            //update start
            gettimeofday(&start, NULL);
         }
    }

    //DPRINTF("%s:%d processed %d intr_status 0x%08x reply post queue len 0x%x\n",
    //        __func__, __LINE__, processed, s->intr_status, reply_post_queue_len);
    if (!s->msix_in_use && processed > 0 && !(s->intr_status & MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT)) {
        trace_mpt3sas_interrupt_coalescing(processed);
        s->intr_status |= MPI2_HIS_REPLY_DESCRIPTOR_INTERRUPT;
        mpt3sas_update_interrupt(s);
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
                mpt3sas_handle_message(s, 0, 0, (MPI2RequestHeader_t *)s->doorbell_msg);
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
        {
            uint8_t msix_index = (val & MPI2_RPHI_MSIX_INDEX_MASK) >> 24;
            s->reply_post[msix_index].host_index = val & MPI2_REPLY_POST_HOST_INDEX_MASK;

            trace_mpt3sas_reply_post_queue(msix_index,
                    s->reply_post[msix_index].base,
                    s->reply_post[msix_index].host_index,
                    s->reply_post[msix_index].ioc_index);
            if (s->reply_post[msix_index].host_index ==
                (s->reply_post[msix_index].ioc_index + 1) % s->reply_descriptor_post_queue_depth) {
                mpt3sas_set_fault(s, MPI2_IOCSTATUS_INSUFFICIENT_RESOURCES);
                trace_mpt3sas_reply_post_queue_full();
                break;
            }
            //clear ReplyDescriptorInterrupt
            mpt3sas_clear_reply_descriptor_int(s, msix_index);
            break;
        }
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
                mpt3sas_set_fault(s, MPI2_IOCSTATUS_BUSY);
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
        trace_mpt3sas_scsi_io_command_sense_data(sense_buf[0] & 0x7f, sense_buf[2] & 0xf, sense_buf[12], sense_buf[13]);
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

        if (sreq->status == GOOD) {
            reply.TransferCount = req->scsi_io.DataLength - resid;
            if (resid) {
                reply.IOCStatus = MPI2_IOCSTATUS_SCSI_DATA_UNDERRUN;
            }
        } else {
            reply.SCSIState = MPI2_SCSI_STATE_AUTOSENSE_VALID;
            reply.SenseCount = sense_len;
            reply.IOCStatus = MPI2_IOCSTATUS_SCSI_DATA_UNDERRUN;
            dump_cdb(req->scsi_io.CDB.CDB32, req->scsi_io.IoFlags & 0xff);
            trace_mpt3sas_scsi_io_command_error(sreq->dev->wwn, req, req->scsi_io.CDB.CDB32[0], req->smid, sreq->status);
        }

        mpt3sas_post_reply(s, req->smid, req->msix_index, (MPI2DefaultReply_t *)&reply);
        mpt3sas_free_request(req);
    } else {
        uint32_t lba = 0x0;

        if (req->scsi_io.CDB.CDB32[0] == 0x28 || req->scsi_io.CDB.CDB32[0] == 0x2a) {
            lba = (uint32_t)req->scsi_io.CDB.CDB32[2] << 24 |
                  (uint32_t)req->scsi_io.CDB.CDB32[3] << 16 |
                  (uint32_t)req->scsi_io.CDB.CDB32[4] << 8 |
                  (uint32_t)req->scsi_io.CDB.CDB32[5];
            //dump_cdb(req->scsi_io.CDB.CDB32);
        }

        trace_mpt3sas_scsi_io_add_command(req, req->scsi_io.CDB.CDB32[0], req->smid, lba);

        if (s->msix_in_use)  {
            trace_mpt3sas_scsi_io_command_completed(req, req->smid, req->scsi_io.CDB.CDB32[0], s->completed_commands);
            mpt3sas_scsi_io_reply(s, req->smid, req->msix_index);
            mpt3sas_free_request(req);
            mpt3sas_msix_notify(s, req->msix_index);
        } else {
            //mpt3sas_scsi_io_reply(s, req->smid);
            s->completed_queue[s->completed_queue_tail++] = req;
            s->completed_queue_tail %= ARRAY_SIZE(s->completed_queue);
            qemu_bh_schedule(s->completed_request_bh);
        }
    }

}

static void mpt3sas_msix_unuse_vectors(MPT3SASState *s, uint32_t num_vectors)
{
    PCIDevice *dev = PCI_DEVICE(s);
    int i;

    for (i = 0; i < num_vectors; i++) {
        msix_vector_unuse(dev, i);
    }
}

static int mpt3sas_msix_use_vectors(MPT3SASState *s, uint32_t num_vectors)
{
    PCIDevice *dev = PCI_DEVICE(s);
    int err;
    int i;

    for (i = 0; i < num_vectors; i++) {
        err = msix_vector_use(dev, i);
        if (err) {
            goto rollback;
        }
    }

    return 0;
rollback:
    DPRINTF("%s:%d msix vector use failed.\n", __func__, __LINE__);
    mpt3sas_msix_unuse_vectors(s, num_vectors);
    return err;
}

static void mpt3sas_request_cancelled(SCSIRequest *sreq)
{
}

static void mpt3sas_init_expander(MPT3SASState *s)
{
    if (!s->expander.count)
        s->expander.count = MPT3SAS_EXPANDER_COUNT;

    if (!s->expander.all_phys)
        s->expander.all_phys = MPT3SAS_EXPANDER_NUM_PHYS;

    s->expander.all_phys += 1; // Leave one virtual phy for enclosure target
    
    //s->expander.downstream_start_phy = MPT3SAS_NUM_PHYS;

    if (!s->expander.upstream_phys || (s->expander.upstream_phys > MPT3SAS_NUM_PHYS / s->expander.count))
        s->expander.upstream_phys = MPT3SAS_NUM_PHYS / s->expander.count;

    // Expander PHY0-PHY3 <----> HBA PHY0-PHY3
    // Expander PHY4-PHY7 <----> [ X X X X ]
    if (!s->expander.downstream_phys)
        s->expander.downstream_phys = s->expander.all_phys - s->expander.downstream_start_phy;

    //s->expander.upstream_start_phy = 0;

    // Upstream phy can't be larger than s->expander.all_phys - 1 - MPT3SAS_NUM_PHYS
    if (s->expander.upstream_start_phy > s->expander.all_phys - MPT3SAS_NUM_PHYS - 1)
        s->expander.upstream_start_phy = s->expander.all_phys - MPT3SAS_NUM_PHYS - 1;

    if (s->expander.upstream_start_phy == s->expander.downstream_start_phy ||
        (s->expander.upstream_start_phy + s->expander.upstream_phys) != s->expander.downstream_start_phy ||
        (s->expander.downstream_start_phy + s->expander.downstream_phys) != s->expander.upstream_start_phy) {
        s->expander.upstream_start_phy = 0;
        s->expander.downstream_start_phy = MPT3SAS_NUM_PHYS;
    }
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

static int mpt3sas_parse_cdb(SCSIDevice *dev, SCSICommand *cmd, uint8_t *buf, void *hba_private)
{
    if (buf[0] == ATA_PASSTHROUGH_12 || buf[0] == ATA_PASSTHROUGH_16)
        return -1;

    return scsi_req_parse_cdb(dev, cmd, buf);
}
#endif

static const struct SCSIBusInfo mpt3sas_scsi_info = {
    .tcq = true,
    .max_target = MAX_SCSI_TARGETS,
    .max_lun = MAX_SCSI_LUNS,

    .get_sg_list = mpt3sas_get_ieee_sg_list,
    .complete = mpt3sas_command_complete,
    .cancel = mpt3sas_request_cancelled,
//    .parse_cdb = mpt3sas_parse_cdb,
//    .save_request = mpt3sas_save_request,
//    .load_request = mpt3sas_load_request,

};

static void mpt3sas_config_write(PCIDevice *pci_dev, uint32_t address,
        uint32_t val, int len)
{
    trace_mpt3sas_config_write(address, val, len);

    //Workround in case the OS doesn't set BME bit.
    if (address == PCI_COMMAND) {
        val |= PCI_COMMAND_MASTER;
    }

    pci_default_write_config(pci_dev, address, val, len);
}

static uint32_t mpt3sas_config_read(PCIDevice *pci_dev, uint32_t address, int len)
{
    uint32_t val = pci_default_read_config(pci_dev, address, len);
    trace_mpt3sas_config_read(address, val, len);
    return val;
}

static void mpt3sas_hotplug(HotplugHandler *sptr, DeviceState *dptr, Error **errp)
{

    MPT3SASState *s = MPT3SAS(sptr);
    SCSIDevice *d = SCSI_DEVICE(dptr);

    uint32_t event_data_length = 0;
    uint32_t scsi_id = d->id;
    trace_mpt3sas_hotplug(d->wwn, scsi_id);

    // 1. notify driver sas discovery start
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_STARTED);

    uint8_t expander_id = scsi_id / s->expander.downstream_phys;
    uint8_t phy_id = SCSI_ID_TO_EXP_PHY(s, scsi_id);
    uint8_t dev_handle = SCSI_ID_TO_HANDLE(scsi_id);

    // 2. notify driver sas topology change for all newly added device
    MPT3SASEventData *event_data2 = NULL;
    pMpi2EventDataSasTopologyChangeList_t sas_topology_change_list = NULL;
    event_data_length = offsetof(Mpi2EventDataSasTopologyChangeList_t, PHY) + sizeof(MPI2_EVENT_SAS_TOPO_PHY_ENTRY) * 1;
    event_data2 = g_malloc0(sizeof(MPT3SASEventData) + event_data_length);
    event_data2->length = event_data_length;
    sas_topology_change_list = (pMpi2EventDataSasTopologyChangeList_t)event_data2->data;
    sas_topology_change_list->EnclosureHandle = MPT3SAS_EXPANDER_ENCLOSURE_HANDLE + expander_id;
    sas_topology_change_list->ExpanderDevHandle = MPT3SAS_EXPANDER_HANDLE_START + expander_id;
    sas_topology_change_list->NumPhys = MPT3SAS_EXPANDER_NUM_PHYS;
    sas_topology_change_list->NumEntries = 1;
    sas_topology_change_list->StartPhyNum = phy_id;
    sas_topology_change_list->ExpStatus = MPI2_EVENT_SAS_TOPO_ES_ADDED;
    sas_topology_change_list->PhysicalPort = 0x0;
    sas_topology_change_list->PHY[0].AttachedDevHandle = cpu_to_le16(dev_handle);
    sas_topology_change_list->PHY[0].LinkRate = MPI25_EVENT_SAS_TOPO_LR_RATE_12_0 << MPI2_EVENT_SAS_TOPO_LR_CURRENT_SHIFT;
    sas_topology_change_list->PHY[0].PhyStatus = MPI2_EVENT_SAS_TOPO_RC_TARG_ADDED;
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST, event_data2);

    // 3. notify driver sas discovery completed
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_COMPLETED);
}

static void mpt3sas_hotunplug(HotplugHandler *sptr, DeviceState *dptr, Error **errp)
{

    MPT3SASState *s = MPT3SAS(sptr);
    SCSIDevice *d = SCSI_DEVICE(dptr);

    uint32_t event_data_length = 0;
    uint32_t scsi_id = d->id;
    trace_mpt3sas_hotunplug(d->wwn, scsi_id);

    qdev_simple_device_unplug_cb(sptr, dptr, errp);

    // 1. notify driver sas discovery start
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_STARTED);

    uint8_t expander_id = scsi_id / s->expander.downstream_phys;
    uint8_t phy_id = SCSI_ID_TO_EXP_PHY(s, scsi_id);
    uint8_t dev_handle = SCSI_ID_TO_HANDLE(scsi_id);

    // 2. notify driver device status change (internel device reset)
    mpt3sas_sas_device_status_change_event_enqueue(s, MPI2_EVENT_SAS_DEV_STAT_RC_INTERNAL_DEVICE_RESET, scsi_id);

    // 3. notify driver sas topology change for all newly added device
    MPT3SASEventData *event_data3 = NULL;
    pMpi2EventDataSasTopologyChangeList_t sas_topology_change_list = NULL;
    event_data_length = offsetof(Mpi2EventDataSasTopologyChangeList_t, PHY) + sizeof(MPI2_EVENT_SAS_TOPO_PHY_ENTRY) * 1;
    event_data3 = g_malloc0(sizeof(MPT3SASEventData) + event_data_length);
    event_data3->length = event_data_length;
    sas_topology_change_list = (pMpi2EventDataSasTopologyChangeList_t)event_data3->data;
    sas_topology_change_list->EnclosureHandle = MPT3SAS_EXPANDER_ENCLOSURE_HANDLE + expander_id;
    sas_topology_change_list->ExpanderDevHandle = MPT3SAS_EXPANDER_HANDLE_START + expander_id;
    sas_topology_change_list->NumPhys = MPT3SAS_EXPANDER_NUM_PHYS;
    sas_topology_change_list->NumEntries = 1;
    sas_topology_change_list->StartPhyNum = phy_id;
    sas_topology_change_list->ExpStatus = MPI2_EVENT_SAS_TOPO_ES_RESPONDING;
    sas_topology_change_list->PhysicalPort = expander_id;
    sas_topology_change_list->PHY[0].AttachedDevHandle = cpu_to_le16(dev_handle);
    sas_topology_change_list->PHY[0].LinkRate = MPI2_EVENT_SAS_TOPO_LR_UNKNOWN_LINK_RATE << MPI2_EVENT_SAS_TOPO_LR_CURRENT_SHIFT;
    sas_topology_change_list->PHY[0].PhyStatus = MPI2_EVENT_SAS_TOPO_RC_TARG_NOT_RESPONDING;
    mpt3sas_event_enqueue(s, MPI2_EVENT_SAS_TOPOLOGY_CHANGE_LIST, event_data3);

    // 4. notify driver device status change (internel device reset complete)
    mpt3sas_sas_device_status_change_event_enqueue(s, MPI2_EVENT_SAS_DEV_STAT_RC_CMP_INTERNAL_DEV_RESET, scsi_id);

    // 5. notify driver sas discovery completed
    mpt3sas_send_discovery_event(s, MPI2_EVENT_SAS_DISC_RC_COMPLETED);
}


static void mpt3sas_scsi_init(PCIDevice *dev, Error **errp)
{
    DeviceState *d = DEVICE(dev);
    MPT3SASState *s = MPT3SAS(dev);

    dev->config_write = mpt3sas_config_write;
    dev->config_read = mpt3sas_config_read;

    dev->config[PCI_LATENCY_TIMER] = 0;
    dev->config[PCI_INTERRUPT_PIN] = 0x01;

    memory_region_init_io(&s->mmio_io, OBJECT(s), &mpt3sas_mmio_ops, s,
            "mpt3sas-mmio", 0x10000);
    memory_region_init_io(&s->port_io, OBJECT(s), &mpt3sas_port_ops, s,
            "mpt3sas-io", 256);

    s->msix_in_use = false;

    // nentries -  96 (physical), but here we just set the entries to 16
    // table_bar_nr 0x1
    // table_offset 0xe000
    // pba_bar_nr 0x1
    // pba_offset 0xf000
    // cap_pos 0xc0
    if (s->msix_available &&
        !msix_init(dev, MPT3SAS_MAX_MSIX_VECTORS, &s->mmio_io, 0x1, 0xe000,
            &s->mmio_io, 0x1, 0xf000, 0xc0)) {
        DPRINTF("Initialize msix ok.\n");
        s->msix_in_use = true;
    }

    if (pci_is_express(dev)) {
        int pos = 0;

        pos = pcie_endpoint_cap_init(dev, 0x68);

        // Power Management
        pos = pci_add_capability(dev, PCI_CAP_ID_PM, 0x50, PCI_PM_SIZEOF);
        pci_set_word(dev->config + pos + PCI_PM_PMC, 0x0603);
        pci_set_word(dev->config + pos + PCI_PM_CTRL, 0x8);

        // AER init
        pcie_aer_init(dev, 0x100, PCI_ERR_SIZEOF);
    }

    // bar0 for IO space, size: 256 bytes
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->port_io);

    // bar1 for memory io space, size: 64K
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY |
                                 PCI_BASE_ADDRESS_MEM_TYPE_64, &s->mmio_io);

    if (s->msix_in_use) {
        mpt3sas_msix_use_vectors(s, MPT3SAS_MAX_MSIX_VECTORS);
    }

    if (!s->sas_address) {
        s->sas_address = ((NAA_LOCALLY_ASSIGNED_ID << 24) |
                       IEEE_COMPANY_LOCALLY_ASSIGNED) << 36;
        s->sas_address |= (pci_bus_num(dev->bus) << 16);
        s->sas_address |= (PCI_SLOT(dev->devfn) << 8);
        s->sas_address |= PCI_FUNC(dev->devfn);
    }

    //s->max_devices = MPT3SAS_NUM_PHYS;
    s->request_bh = qemu_bh_new(mpt3sas_handle_requests, s);
    s->completed_request_bh = qemu_bh_new(mpt3sas_handle_completed_request, s);

    // Launch event handler thread
    mpt3sas_event_queue_init(s);
    qemu_thread_create(&s->event_queue->thread, "MPT3SAS Event Worker", mpt3sas_event_queue_worker_thread, s, QEMU_THREAD_DETACHED);

    QTAILQ_INIT(&s->pending);

    scsi_bus_new(&s->bus, sizeof(s->bus), &dev->qdev, &mpt3sas_scsi_info, NULL);
    qbus_set_hotplug_handler(BUS(&s->bus), (DeviceState *)dev, &error_abort);

    // init expander
    mpt3sas_init_expander(s);

    if (!d->hotplugged) {
        scsi_bus_legacy_handle_cmdline(&s->bus, errp);
    }
}

static void mpt3sas_scsi_uninit(PCIDevice *dev)
{
    MPT3SASState *s = MPT3SAS(dev);

    qemu_bh_delete(s->request_bh);
    qemu_bh_delete(s->completed_request_bh);

    if (s->msix_in_use) {
        msix_uninit(dev, &s->mmio_io, &s->mmio_io);
    }

    s->event_queue->exit = 0;
}

static void mpt3sas_reset(DeviceState *dev)
{
    MPT3SASState *s = MPT3SAS(dev);

    DPRINTF("%s:%d PCI Device reset.\n", __func__, __LINE__);
    mpt3sas_hard_reset(s);
}

static Property mpt3sas_properties[] = {
    DEFINE_PROP_UINT64("sas_address", MPT3SASState, sas_address, 0),
    DEFINE_PROP_BIT("use_msix", MPT3SASState, msix_available, 0, true),
    DEFINE_PROP_UINT32("expander-count", MPT3SASState, expander.count, MPT3SAS_EXPANDER_COUNT),
    DEFINE_PROP_UINT32("expander-phys", MPT3SASState, expander.all_phys, MPT3SAS_EXPANDER_NUM_PHYS),
    DEFINE_PROP_UINT32("downstream-start-phy", MPT3SASState, expander.downstream_start_phy, MPT3SAS_NUM_PHYS),
    DEFINE_PROP_UINT32("upstream-start-phy", MPT3SASState, expander.upstream_start_phy, 0),
    DEFINE_PROP_UINT32("upstream-phys", MPT3SASState, expander.upstream_phys, 0),
    DEFINE_PROP_UINT32("downstream-phys", MPT3SASState, expander.downstream_phys, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void mpt3sas3008_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);
    HotplugHandlerClass *hc = HOTPLUG_HANDLER_CLASS(oc);

    pc->realize = mpt3sas_scsi_init;
    pc->exit = mpt3sas_scsi_uninit;
    pc->romfile = 0;
    pc->vendor_id = MPI2_MFGPAGE_VENDORID_LSI;
    pc->device_id = MPI25_MFGPAGE_DEVID_SAS3008;

    // DELL ID
    // http://pciids.sourceforge.net/v2.2/pci.ids
    pc->subsystem_vendor_id = 0x1028; //Dell
    pc->subsystem_id = 0x1f46;

    // DEFAULT ID
    //pc->subsystem_vendor_id = 0x1000;
    //pc->subsystem_id = 0x80;
    pc->revision = 0x2;
    //pc->class_id = PCI_CLASS_STORAGE_SCSI;
    pc->class_id = 0x107;
    pc->is_express = true;
    dc->props = mpt3sas_properties;
    dc->reset = mpt3sas_reset;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "LSI SAS 3008";

    hc->plug = mpt3sas_hotplug;
    hc->unplug = mpt3sas_hotunplug;
}

static const TypeInfo mpt3sas_info = {
    .name = TYPE_MPT3SAS3008,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MPT3SASState),
    .class_init = mpt3sas3008_class_init,
    .interfaces = (InterfaceInfo[]) {
        { TYPE_HOTPLUG_HANDLER },
        { }
    }
};

static void mpt3sas_register_types(void)
{
    type_register(&mpt3sas_info);
}

type_init(mpt3sas_register_types)
