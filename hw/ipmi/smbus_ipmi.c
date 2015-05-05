/*
 * QEMU IPMI SMBus (SSIF) emulation
 *
 * Copyright (c) 2015 Corey Minyard, MontaVista Software, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hw/hw.h"
#include "hw/i2c/smbus.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "ipmi.h"

#define TYPE_IPMI_INTERFACE_SMBUS TYPE_IPMI_INTERFACE_PREFIX "smbus"
#define IPMI_INTERFACE_SMBUS(obj) OBJECT_CHECK(IPMISMBusInterface, (obj), \
                                               TYPE_IPMI_INTERFACE_SMBUS)

#define SSIF_IPMI_REQUEST                       2
#define SSIF_IPMI_MULTI_PART_REQUEST_START      6
#define SSIF_IPMI_MULTI_PART_REQUEST_MIDDLE     7
#define SSIF_IPMI_RESPONSE                      3
#define SSIF_IPMI_MULTI_PART_RESPONSE_MIDDLE    9

typedef struct IPMISMBusInterface {
    IPMIInterface intf;

    /*
     * This is a response number that we send with the command to make
     * sure that the response matches the command.
     */
    uint8_t waiting_rsp;

    char *alertname;
    SMBusAlertEntry alert_entry;
} IPMISMBusInterface;

static void ipmi_smbus_handle_event(IPMIInterface *s)
{
    /* No interrupts, so nothing to do here. */
}

static void ipmi_smbus_handle_rsp(IPMIInterface *intf, uint8_t msg_id,
                                  unsigned char *rsp, unsigned int rsp_len)
{
    IPMISMBusInterface *smbus = IPMI_INTERFACE_SMBUS(intf);

    if (smbus->waiting_rsp == msg_id) {
        smbus->waiting_rsp++;

        memcpy(intf->outmsg, rsp, rsp_len);
        intf->outlen = rsp_len;
        intf->outpos = 0;
        smbus_do_alert(smbus->alertname, &smbus->alert_entry);
    }
}

static void ipmi_smbus_set_atn(IPMIInterface *s, int val, int irq)
{
    /* This is where PEC would go. */
}

static const VMStateDescription vmstate_ipmi_smbus = {
    .name = TYPE_IPMI_INTERFACE_SMBUS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT8(waiting_rsp, IPMISMBusInterface),
        VMSTATE_END_OF_LIST()
    }
};

static void ipmi_smbus_init(IPMIInterface *s, Error **errp)
{
    IPMISMBusInterface *smbus = IPMI_INTERFACE_SMBUS(s);

    vmstate_register(NULL, 0, &vmstate_ipmi_smbus, smbus);
}

static void ipmi_smbus_class_init(ObjectClass *class, void *data)
{
    IPMIInterfaceClass *k = IPMI_INTERFACE_CLASS(class);

    k->init = ipmi_smbus_init;
    k->smbios_type = IPMI_SMBIOS_SSIF;
    k->set_atn = ipmi_smbus_set_atn;
    k->handle_rsp = ipmi_smbus_handle_rsp;
    k->handle_if_event = ipmi_smbus_handle_event;
}

static const TypeInfo ipmi_smbus_interface_type = {
    .name          = TYPE_IPMI_INTERFACE_SMBUS,
    .parent        = TYPE_IPMI_INTERFACE,
    .instance_size = sizeof(IPMISMBusInterface),
    .class_init    = ipmi_smbus_class_init,
};


#define TYPE_SMBUS_IPMI "smbus-ipmi"
#define SMBUS_IPMI(obj) OBJECT_CHECK(IPMISMBUS, (obj), TYPE_SMBUS_IPMI)

typedef struct SMBusIPMIDevice {
    SMBusDevice smbusdev;

    uint8_t slave_addr;
    bool threaded_bmc;
    CharDriverState *chr;
    IPMIInterface *intf;
    IPMIFwInfo fwinfo;
    char *alertname;
} SMBusIPMIDevice;

static void ipmi_quick_cmd(SMBusDevice *dev, uint8_t read)
{
}

static void ipmi_send_byte(SMBusDevice *dev, uint8_t val)
{
}

static uint8_t ipmi_receive_byte(SMBusDevice *dev)
{
    SMBusIPMIDevice *ipmi = (SMBusIPMIDevice *) dev;
    IPMIInterface *intf = ipmi->intf;

    if (intf->outpos >= intf->outlen)
        return 0;

    return intf->outmsg[intf->outpos++];
}

static void ipmi_write_data(SMBusDevice *dev, uint8_t cmd, uint8_t *buf, int len)
{
    SMBusIPMIDevice *ipmi = (SMBusIPMIDevice *) dev;
    IPMIInterface *intf = ipmi->intf;
    IPMISMBusInterface *smbus = IPMI_INTERFACE_SMBUS(intf);
    IPMIBmcClass *bk = IPMI_BMC_GET_CLASS(intf->bmc);

    if (cmd != SSIF_IPMI_REQUEST)
        return;

    if (len < 3 || len > MAX_IPMI_MSG_SIZE || buf[0] != len - 1)
        return;

    memcpy(intf->inmsg, buf + 1, len - 1);
    intf->inlen = len;

    intf->outlen = 0;
    intf->write_end = 0;
    intf->outpos = 0;
    bk->handle_command(intf->bmc, intf->inmsg, intf->inlen, sizeof(intf->inmsg),
                       smbus->waiting_rsp);
}

static uint8_t ipmi_read_data(SMBusDevice *dev, uint8_t cmd, int n)
{
    SMBusIPMIDevice *ipmi = (SMBusIPMIDevice *) dev;
    IPMIInterface *intf = ipmi->intf;

    if (cmd != SSIF_IPMI_RESPONSE)
        return 0;

    if (n == 0)
        return intf->outlen;

    return ipmi_receive_byte(dev);
}

static const VMStateDescription vmstate_smbus_ipmi = {
    .name = TYPE_SMBUS_IPMI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_STRUCT_POINTER(intf, SMBusIPMIDevice, vmstate_IPMIInterface,
                               IPMIInterface),
        VMSTATE_END_OF_LIST()
    }
};

static int smbus_ipmi_initfn(SMBusDevice *dev)
{
    SMBusIPMIDevice *ipmi = (SMBusIPMIDevice *) dev;
    Object *intfobj;
    IPMIInterface *intf;
    IPMISMBusInterface *smbus;
    Object *bmcobj;
    IPMIBmc *bmc;
    Error *err = NULL;

    if (ipmi->chr) {
        bmcobj = object_new(TYPE_IPMI_BMC_EXTERN);
    } else {
        bmcobj = object_new(TYPE_IPMI_BMC_SIMULATOR);
    }
    bmc = IPMI_BMC(bmcobj);
    bmc->chr = ipmi->chr;
    intfobj = object_new(TYPE_IPMI_INTERFACE_SMBUS);
    intf = IPMI_INTERFACE(intfobj);
    smbus = IPMI_INTERFACE_SMBUS(intf);
    bmc->intf = intf;
    intf->bmc = bmc;
    intf->threaded_bmc = ipmi->threaded_bmc;
    smbus->alertname = ipmi->alertname;
    smbus->alert_entry.val = ipmi->smbusdev.i2c.address << 1;
    ipmi_interface_init(intf, &err);
    if (err) {
        goto out_err;
    }

    ipmi_bmc_init(bmc, &err);
    if (err) {
        goto out_err;
    }

    ipmi->intf = intf;
    object_property_add_child(OBJECT(dev), "intf", OBJECT(intf), &err);
    if (err) {
        goto out_err;
    }
    object_property_add_child(OBJECT(dev), "bmc", OBJECT(bmc), &err);
    if (err) {
        goto out_err;
    }

    ipmi->fwinfo.interface_name = "smbus";
    ipmi->fwinfo.interface_type = IPMI_SMBIOS_SSIF;
    ipmi->fwinfo.ipmi_spec_major_revision = 2;
    ipmi->fwinfo.ipmi_spec_minor_revision = 0;
    ipmi->fwinfo.i2c_slave_address = ipmi->slave_addr;
    ipmi->fwinfo.base_address = ipmi->smbusdev.i2c.address;
    ipmi->fwinfo.memspace = IPMI_MEMSPACE_SMBUS;
    ipmi->fwinfo.register_spacing = 1;
    ipmi_add_fwinfo(&ipmi->fwinfo);

    return 0;

 out_err:
    error_report("%s", error_get_pretty(err));
    error_free(err);
    return -1;
}

static Property smbus_ipmi_properties[] = {
    DEFINE_PROP_UINT8("slave_addr", SMBusIPMIDevice, slave_addr,  0),
    DEFINE_PROP_CHR("chardev",  SMBusIPMIDevice, chr),
    DEFINE_PROP_BOOL("threadbmc",  SMBusIPMIDevice, threaded_bmc, false),
    DEFINE_PROP_STRING("alertname", SMBusIPMIDevice, alertname),
    DEFINE_PROP_END_OF_LIST(),
};

static void smbus_ipmi_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);

    sc->init = smbus_ipmi_initfn;
    sc->quick_cmd = ipmi_quick_cmd;
    sc->send_byte = ipmi_send_byte;
    sc->receive_byte = ipmi_receive_byte;
    sc->write_data = ipmi_write_data;
    sc->read_data = ipmi_read_data;
    dc->props = smbus_ipmi_properties;
    dc->vmsd = &vmstate_smbus_ipmi;
}

static const TypeInfo smbus_ipmi_info = {
    .name          = TYPE_SMBUS_IPMI,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(SMBusIPMIDevice),
    .class_init    = smbus_ipmi_class_initfn,
};

static void smbus_ipmi_register_types(void)
{
    type_register_static(&smbus_ipmi_info);
    type_register_static(&ipmi_smbus_interface_type);
}

type_init(smbus_ipmi_register_types)
