/*
 * QEMU IPMI SMBus alert
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
#include "hw/irq.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

typedef struct SMBusAlertDevice {
    SMBusDevice smbusdev;
    char *name;
    char *irqname;
    qemu_irq irq;

    QSIMPLEQ_HEAD(, SMBusAlertEntry) alert_queue;

    QSLIST_ENTRY(SMBusAlertDevice) link;
} SMBusAlertDevice;

#define TYPE_SMBUS_ALERT_DEVICE "smbus-alert"
#define SMBUS_ALERT_DEVICE(obj) OBJECT_CHECK(SMBusAlertDevice, (obj),    \
                                             TYPE_SMBUS_ALERT_DEVICE)

static QSLIST_HEAD(, SMBusAlertDevice) alert_devices = 
    QSLIST_HEAD_INITIALIZER(&alert_devices);


void smbus_do_alert(char *name, SMBusAlertEntry *alert)
{
    SMBusAlertDevice *e;

    if (alert->enqueued)
	return;

    QSLIST_FOREACH(e, &alert_devices, link) {
	if (strcmp(e->name, name) == 0) {
	    alert->enqueued = true;
	    QSIMPLEQ_INSERT_TAIL(&e->alert_queue, alert, link);
	    qemu_irq_raise(e->irq);
	    break;
	}
    }
}

static uint8_t alert_receive_byte(SMBusDevice *dev)
{
    SMBusAlertDevice *alertdev = (SMBusAlertDevice *) dev;
    SMBusAlertEntry *alert = QSIMPLEQ_FIRST(&alertdev->alert_queue);

    if (!alert)
	return -1;

    alert->enqueued = false;
    QSIMPLEQ_REMOVE_HEAD(&alertdev->alert_queue, link);

    if (QSIMPLEQ_EMPTY(&alertdev->alert_queue)) {
	    qemu_irq_lower(alertdev->irq);
    }

    return alert->val;
}

static int smbus_alert_event(I2CSlave *s, enum i2c_event event)
{
    SMBusAlertDevice *alertdev = (SMBusAlertDevice *) s;
    I2CSlaveClass *sc = I2C_SLAVE_GET_CLASS(s);

    if (event == I2C_START_RECV && QSIMPLEQ_EMPTY(&alertdev->alert_queue))
        return -1;

    sc->event(s, event);
    return 0;
}

static int smbus_alert_initfn(SMBusDevice *dev)
{
    SMBusAlertDevice *alertdev = (SMBusAlertDevice *) dev;
    SMBusAlertDevice *e;

    if (!alertdev->name) {
	error_report("SMBus alert device created without name");
	return -1;
    }

    if (!alertdev->irqname) {
	error_report("SMBus alert device %s created without irqname",
		     alertdev->name);
	return -1;
    }

    QSIMPLEQ_INIT(&alertdev->alert_queue);
    QSLIST_FOREACH(e, &alert_devices, link) {
	if (strcmp(e->name, alertdev->name) == 0) {
	    error_report("Duplicate smbus-alert device name: %s\n", e->name);
	    return -1;
	}
    }
    QSLIST_INSERT_HEAD(&alert_devices, alertdev, link);

    alertdev->irq = qemu_find_named_irq(alertdev->irqname);
    if (!alertdev->irq) {
	error_report("SMBus alert device was unable to find irq named %s\n",
		     alertdev->irqname);
	return -1;
    }

    return 0;
}

static Property smbus_alert_properties[] = {
    DEFINE_PROP_STRING("name", SMBusAlertDevice, name),
    DEFINE_PROP_STRING("irqname",  SMBusAlertDevice, irqname),
    DEFINE_PROP_END_OF_LIST(),
};

static void smbus_alert_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);

    sc->init = smbus_alert_initfn;
    sc->receive_byte = alert_receive_byte;
    ic->event_check = smbus_alert_event;
    dc->props = smbus_alert_properties;
}

static const TypeInfo smbus_alert_info = {
    .name          = TYPE_SMBUS_ALERT_DEVICE,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(SMBusAlertDevice),
    .class_init    = smbus_alert_class_initfn,
};

static void smbus_alert_register_types(void)
{
    type_register_static(&smbus_alert_info);
}

type_init(smbus_alert_register_types)
