#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/usb/quirks.h>
#include <linux/usb.h>

#include "usb.h"
#include "hcd.h"

static void cancel_async_set_config(struct usb_device *udev);

/**
 * usb_find_alt_setting() - Given a configuration, find the alternate setting
 * for the given interface.
 * @config: the configuration to search (not necessarily the current config).
 * @iface_num: interface number to search in
 * @alt_num: alternate interface setting number to search for.
 *
 * Search the configuration's interface cache for the given alt setting.
 */
static struct usb_host_interface *usb_find_alt_setting(
		struct usb_host_config *config,
		unsigned int iface_num,
		unsigned int alt_num)
{
	struct usb_interface_cache *intf_cache = NULL;
	int i;

	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		if (config->intf_cache[i]->altsetting[0].desc.bInterfaceNumber
				== iface_num) {
			intf_cache = config->intf_cache[i];
			break;
		}
	}
	if (!intf_cache)
		return NULL;
	for (i = 0; i < intf_cache->num_altsetting; i++)
		if (intf_cache->altsetting[i].desc.bAlternateSetting == alt_num)
			return &intf_cache->altsetting[i];

	printk(KERN_DEBUG "Did not find alt setting %u for intf %u, "
			"config %u\n", alt_num, iface_num,
			config->desc.bConfigurationValue);
	return NULL;
}

/**
 * usb_hcd_alloc_bandwidth - check whether a new bandwidth setting exceeds
 *				the bus bandwidth
 * @udev: target &usb_device
 * @new_config: new configuration to install
 * @cur_alt: the current alternate interface setting
 * @new_alt: alternate interface setting that is being installed
 *
 * To change configurations, pass in the new configuration in new_config,
 * and pass NULL for cur_alt and new_alt.
 *
 * To reset a device's configuration (put the device in the ADDRESSED state),
 * pass in NULL for new_config, cur_alt, and new_alt.
 *
 * To change alternate interface settings, pass in NULL for new_config,
 * pass in the current alternate interface setting in cur_alt,
 * and pass in the new alternate interface setting in new_alt.
 *
 * Returns an error if the requested bandwidth change exceeds the
 * bus bandwidth or host controller internal resources.
 */
int usb_hcd_alloc_bandwidth(struct usb_device *udev,
		struct usb_host_config *new_config,
		struct usb_host_interface *cur_alt,
		struct usb_host_interface *new_alt)
{
	int num_intfs, i, j;
	struct usb_host_interface *alt = NULL;
	int ret = 0;
	struct usb_hcd *hcd;
	struct usb_host_endpoint *ep;

	hcd = bus_to_hcd(udev->bus);
	if (!hcd->driver->check_bandwidth)
		return 0;

	/* Configuration is being removed - set configuration 0 */
	if (!new_config && !cur_alt) {
		for (i = 1; i < 16; ++i) {
			ep = udev->ep_out[i];
			if (ep)
				hcd->driver->drop_endpoint(hcd, udev, ep);
			ep = udev->ep_in[i];
			if (ep)
				hcd->driver->drop_endpoint(hcd, udev, ep);
		}
		hcd->driver->check_bandwidth(hcd, udev);
		return 0;
	}
	/* Check if the HCD says there's enough bandwidth.  Enable all endpoints
	 * each interface's alt setting 0 and ask the HCD to check the bandwidth
	 * of the bus.  There will always be bandwidth for endpoint 0, so it's
	 * ok to exclude it.
	 */
	if (new_config) {
		num_intfs = new_config->desc.bNumInterfaces;
		/* Remove endpoints (except endpoint 0, which is always on the
		 * schedule) from the old config from the schedule
		 */
		for (i = 1; i < 16; ++i) {
			ep = udev->ep_out[i];
			if (ep) {
				ret = hcd->driver->drop_endpoint(hcd, udev, ep);
				if (ret < 0)
					goto reset;
			}
			ep = udev->ep_in[i];
			if (ep) {
				ret = hcd->driver->drop_endpoint(hcd, udev, ep);
				if (ret < 0)
					goto reset;
			}
		}
		for (i = 0; i < num_intfs; ++i) {
			struct usb_host_interface *first_alt;
			int iface_num;

			first_alt = &new_config->intf_cache[i]->altsetting[0];
			iface_num = first_alt->desc.bInterfaceNumber;
			/* Set up endpoints for alternate interface setting 0 */
			alt = usb_find_alt_setting(new_config, iface_num, 0);
			if (!alt)
				/* No alt setting 0? Pick the first setting. */
				alt = first_alt;

			for (j = 0; j < alt->desc.bNumEndpoints; j++) {
				ret = hcd->driver->add_endpoint(hcd, udev, &alt->endpoint[j]);
				if (ret < 0)
					goto reset;
			}
		}
	}
	if (cur_alt && new_alt) {
		struct usb_interface *iface = usb_ifnum_to_if(udev,
				cur_alt->desc.bInterfaceNumber);

		if (iface->resetting_device) {
			/*
			 * The USB core just reset the device, so the xHCI host
			 * and the device will think alt setting 0 is installed.
			 * However, the USB core will pass in the alternate
			 * setting installed before the reset as cur_alt.  Dig
			 * out the alternate setting 0 structure, or the first
			 * alternate setting if a broken device doesn't have alt
			 * setting 0.
			 */
			cur_alt = usb_altnum_to_altsetting(iface, 0);
			if (!cur_alt)
				cur_alt = &iface->altsetting[0];
		}

		/* Drop all the endpoints in the current alt setting */
		for (i = 0; i < cur_alt->desc.bNumEndpoints; i++) {
			ret = hcd->driver->drop_endpoint(hcd, udev,
					&cur_alt->endpoint[i]);
			if (ret < 0)
				goto reset;
		}
		/* Add all the endpoints in the new alt setting */
		for (i = 0; i < new_alt->desc.bNumEndpoints; i++) {
			ret = hcd->driver->add_endpoint(hcd, udev,
					&new_alt->endpoint[i]);
			if (ret < 0)
				goto reset;
		}
	}
	ret = hcd->driver->check_bandwidth(hcd, udev);
reset:
	if (ret < 0)
		hcd->driver->reset_bandwidth(hcd, udev);
	return ret;
}

/**
 * usb_release_dev - free a usb device structure when all users of it are finished.
 * @dev: device that's been disconnected
 *
 * Will be called only by the device core when all users of this usb device are
 * done.
 */
void etapi_usb_release_dev(struct device *dev)
{
	struct usb_device *udev;
	struct usb_hcd *hcd;

	udev = to_usb_device(dev);
	hcd = bus_to_hcd(udev->bus);

	usb_destroy_configuration(udev);
	usb_put_hcd(hcd);
	kfree(udev->product);
	kfree(udev->manufacturer);
	kfree(udev->serial);
	kfree(udev);
}

static int create_intf_ep_devs(struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	if (intf->ep_devs_created || intf->unregistering)
		return 0;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		(void) usb_create_ep_devs(&intf->dev, &alt->endpoint[i], udev);
	intf->ep_devs_created = 1;
	return 0;
}

static void remove_intf_ep_devs(struct usb_interface *intf)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	int i;

	if (!intf->ep_devs_created)
		return;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i)
		usb_remove_ep_devs(&alt->endpoint[i]);
	intf->ep_devs_created = 0;
}

/**
 * usb_set_interface - Makes a particular alternate setting be current
 * @dev: the device whose interface is being updated
 * @interface: the interface being updated
 * @alternate: the setting being chosen.
 * Context: !in_interrupt ()
 *
 * This is used to enable data transfers on interfaces that may not
 * be enabled by default.  Not all devices support such configurability.
 * Only the driver bound to an interface may change its setting.
 *
 * Within any given configuration, each interface may have several
 * alternative settings.  These are often used to control levels of
 * bandwidth consumption.  For example, the default setting for a high
 * speed interrupt endpoint may not send more than 64 bytes per microframe,
 * while interrupt transfers of up to 3KBytes per microframe are legal.
 * Also, isochronous endpoints may never be part of an
 * interface's default setting.  To access such bandwidth, alternate
 * interface settings must be made current.
 *
 * Note that in the Linux USB subsystem, bandwidth associated with
 * an endpoint in a given alternate setting is not reserved until an URB
 * is submitted that needs that bandwidth.  Some other operating systems
 * allocate bandwidth early, when a configuration is chosen.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 * Also, drivers must not change altsettings while urbs are scheduled for
 * endpoints in that interface; all such urbs must first be completed
 * (perhaps forced by unlinking).
 *
 * Returns zero on success, or else the status code returned by the
 * underlying usb_control_msg() call.
 */
int etapi_usb_set_interface(struct usb_device *dev, int interface, int alternate)
{
	struct usb_interface *iface;
	struct usb_host_interface *alt;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);
	int ret;
	int manual = 0;
	unsigned int epaddr;
	unsigned int pipe;

	if (dev->state == USB_STATE_SUSPENDED)
		return -EHOSTUNREACH;

	iface = usb_ifnum_to_if(dev, interface);
	if (!iface) {
		dev_dbg(&dev->dev, "selecting invalid interface %d\n",
			interface);
		return -EINVAL;
	}

	alt = usb_altnum_to_altsetting(iface, alternate);
	if (!alt) {
		dev_warn(&dev->dev, "selecting invalid altsetting %d",
			 alternate);
		return -EINVAL;
	}

	/* Make sure we have enough bandwidth for this alternate interface.
	 * Remove the current alt setting and add the new alt setting.
	 */
	mutex_lock(&hcd->bandwidth_mutex);
	ret = usb_hcd_alloc_bandwidth(dev, NULL, iface->cur_altsetting, alt);
	if (ret < 0) {
		dev_info(&dev->dev, "Not enough bandwidth for altsetting %d\n",
				alternate);
		mutex_unlock(&hcd->bandwidth_mutex);
		return ret;
	}

	if (dev->quirks & USB_QUIRK_NO_SET_INTF)
		ret = -EPIPE;
	else
		ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				   USB_REQ_SET_INTERFACE, USB_RECIP_INTERFACE,
				   alternate, interface, NULL, 0, 5000);

	/* 9.4.10 says devices don't need this and are free to STALL the
	 * request if the interface only has one alternate setting.
	 */
	if (ret == -EPIPE && iface->num_altsetting == 1) {
		dev_dbg(&dev->dev,
			"manual set_interface for iface %d, alt %d\n",
			interface, alternate);
		manual = 1;
	} else if (ret < 0) {
		/* Re-instate the old alt setting */
		usb_hcd_alloc_bandwidth(dev, NULL, alt, iface->cur_altsetting);
		mutex_unlock(&hcd->bandwidth_mutex);
		return ret;
	}
	mutex_unlock(&hcd->bandwidth_mutex);

	/* FIXME drivers shouldn't need to replicate/bugfix the logic here
	 * when they implement async or easily-killable versions of this or
	 * other "should-be-internal" functions (like clear_halt).
	 * should hcd+usbcore postprocess control requests?
	 */

	/* prevent submissions using previous endpoint settings */
	if (iface->cur_altsetting != alt) {
		remove_intf_ep_devs(iface);
		usb_remove_sysfs_intf_files(iface);
	}
	usb_disable_interface(dev, iface, true);

	iface->cur_altsetting = alt;

	/* If the interface only has one altsetting and the device didn't
	 * accept the request, we attempt to carry out the equivalent action
	 * by manually clearing the HALT feature for each endpoint in the
	 * new altsetting.
	 */
	if (manual) {
		int i;

		for (i = 0; i < alt->desc.bNumEndpoints; i++) {
			epaddr = alt->endpoint[i].desc.bEndpointAddress;
			pipe = __create_pipe(dev,
					USB_ENDPOINT_NUMBER_MASK & epaddr) |
					(usb_endpoint_out(epaddr) ?
					USB_DIR_OUT : USB_DIR_IN);

			usb_clear_halt(dev, pipe);
		}
	}

	/* 9.1.1.5: reset toggles for all endpoints in the new altsetting
	 *
	 * Note:
	 * Despite EP0 is always present in all interfaces/AS, the list of
	 * endpoints from the descriptor does not contain EP0. Due to its
	 * omnipresence one might expect EP0 being considered "affected" by
	 * any SetInterface request and hence assume toggles need to be reset.
	 * However, EP0 toggles are re-synced for every individual transfer
	 * during the SETUP stage - hence EP0 toggles are "don't care" here.
	 * (Likewise, EP0 never "halts" on well designed devices.)
	 */
	usb_enable_interface(dev, iface, true);
	if (device_is_registered(&iface->dev)) {
		usb_create_sysfs_intf_files(iface);
		create_intf_ep_devs(iface);
	}
	return 0;
}

/**
 * usb_reset_configuration - lightweight device reset
 * @dev: the device whose configuration is being reset
 *
 * This issues a standard SET_CONFIGURATION request to the device using
 * the current configuration.  The effect is to reset most USB-related
 * state in the device, including interface altsettings (reset to zero),
 * endpoint halts (cleared), and endpoint state (only for bulk and interrupt
 * endpoints).  Other usbcore state is unchanged, including bindings of
 * usb device drivers to interfaces.
 *
 * Because this affects multiple interfaces, avoid using this with composite
 * (multi-interface) devices.  Instead, the driver for each interface may
 * use usb_set_interface() on the interfaces it claims.  Be careful though;
 * some devices don't support the SET_INTERFACE request, and others won't
 * reset all the interface state (notably endpoint state).  Resetting the whole
 * configuration would affect other drivers' interfaces.
 *
 * The caller must own the device lock.
 *
 * Returns zero on success, else a negative error code.
 */
int etapi_usb_reset_configuration(struct usb_device *dev)
{
	int			i, retval;
	struct usb_host_config	*config;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);

	if (dev->state == USB_STATE_SUSPENDED)
		return -EHOSTUNREACH;

	/* caller must have locked the device and must own
	 * the usb bus readlock (so driver bindings are stable);
	 * calls during probe() are fine
	 */

	for (i = 1; i < 16; ++i) {
		usb_disable_endpoint(dev, i, true);
		usb_disable_endpoint(dev, i + USB_DIR_IN, true);
	}

	config = dev->actconfig;
	retval = 0;
	mutex_lock(&hcd->bandwidth_mutex);
	/* Make sure we have enough bandwidth for each alternate setting 0 */
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_host_interface *alt;

		alt = usb_altnum_to_altsetting(intf, 0);
		if (!alt)
			alt = &intf->altsetting[0];
		if (alt != intf->cur_altsetting)
			retval = usb_hcd_alloc_bandwidth(dev, NULL,
					intf->cur_altsetting, alt);
		if (retval < 0)
			break;
	}
	/* If not, reinstate the old alternate settings */
	if (retval < 0) {
reset_old_alts:
		for (i--; i >= 0; i--) {
			struct usb_interface *intf = config->interface[i];
			struct usb_host_interface *alt;

			alt = usb_altnum_to_altsetting(intf, 0);
			if (!alt)
				alt = &intf->altsetting[0];
			if (alt != intf->cur_altsetting)
				usb_hcd_alloc_bandwidth(dev, NULL,
						alt, intf->cur_altsetting);
		}
		mutex_unlock(&hcd->bandwidth_mutex);
		return retval;
	}
	retval = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			USB_REQ_SET_CONFIGURATION, 0,
			config->desc.bConfigurationValue, 0,
			NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (retval < 0)
		goto reset_old_alts;
	mutex_unlock(&hcd->bandwidth_mutex);

	/* re-init hc/hcd interface/endpoint state */
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_host_interface *alt;

		alt = usb_altnum_to_altsetting(intf, 0);

		/* No altsetting 0?  We'll assume the first altsetting.
		 * We could use a GetInterface call, but if a device is
		 * so non-compliant that it doesn't have altsetting 0
		 * then I wouldn't trust its reply anyway.
		 */
		if (!alt)
			alt = &intf->altsetting[0];

		if (alt != intf->cur_altsetting) {
			remove_intf_ep_devs(intf);
			usb_remove_sysfs_intf_files(intf);
		}
		intf->cur_altsetting = alt;
		usb_enable_interface(dev, intf, true);
		if (device_is_registered(&intf->dev)) {
			usb_create_sysfs_intf_files(intf);
			create_intf_ep_devs(intf);
		}
	}
	return 0;
}

static struct usb_interface_assoc_descriptor *find_iad(struct usb_device *dev,
						struct usb_host_config *config,
						u8 inum)
{
	struct usb_interface_assoc_descriptor *retval = NULL;
	struct usb_interface_assoc_descriptor *intf_assoc;
	int first_intf;
	int last_intf;
	int i;

	for (i = 0; (i < USB_MAXIADS && config->intf_assoc[i]); i++) {
		intf_assoc = config->intf_assoc[i];
		if (intf_assoc->bInterfaceCount == 0)
			continue;

		first_intf = intf_assoc->bFirstInterface;
		last_intf = first_intf + (intf_assoc->bInterfaceCount - 1);
		if (inum >= first_intf && inum <= last_intf) {
			if (!retval)
				retval = intf_assoc;
			else
				dev_err(&dev->dev, "Interface #%d referenced"
					" by multiple IADs\n", inum);
		}
	}

	return retval;
}

/*
 * Internal function to queue a device reset
 *
 * This is initialized into the workstruct in 'struct
 * usb_device->reset_ws' that is launched by
 * message.c:usb_set_configuration() when initializing each 'struct
 * usb_interface'.
 *
 * It is safe to get the USB device without reference counts because
 * the life cycle of @iface is bound to the life cycle of @udev. Then,
 * this function will be ran only if @iface is alive (and before
 * freeing it any scheduled instances of it will have been cancelled).
 *
 * We need to set a flag (usb_dev->reset_running) because when we call
 * the reset, the interfaces might be unbound. The current interface
 * cannot try to remove the queued work as it would cause a deadlock
 * (you cannot remove your work from within your executing
 * workqueue). This flag lets it know, so that
 * usb_cancel_queued_reset() doesn't try to do it.
 *
 * See usb_queue_reset_device() for more details
 */
static void __usb_queue_reset_device(struct work_struct *ws)
{
	int rc;
	struct usb_interface *iface =
		container_of(ws, struct usb_interface, reset_ws);
	struct usb_device *udev = interface_to_usbdev(iface);

	rc = usb_lock_device_for_reset(udev, iface);
	if (rc >= 0) {
		iface->reset_running = 1;
		usb_reset_device(udev);
		iface->reset_running = 0;
		usb_unlock_device(udev);
	}
}

/*
 * usb_set_configuration - Makes a particular device setting be current
 * @dev: the device whose configuration is being updated
 * @configuration: the configuration being chosen.
 * Context: !in_interrupt(), caller owns the device lock
 *
 * This is used to enable non-default device modes.  Not all devices
 * use this kind of configurability; many devices only have one
 * configuration.
 *
 * @configuration is the value of the configuration to be installed.
 * According to the USB spec (e.g. section 9.1.1.5), configuration values
 * must be non-zero; a value of zero indicates that the device in
 * unconfigured.  However some devices erroneously use 0 as one of their
 * configuration values.  To help manage such devices, this routine will
 * accept @configuration = -1 as indicating the device should be put in
 * an unconfigured state.
 *
 * USB device configurations may affect Linux interoperability,
 * power consumption and the functionality available.  For example,
 * the default configuration is limited to using 100mA of bus power,
 * so that when certain device functionality requires more power,
 * and the device is bus powered, that functionality should be in some
 * non-default device configuration.  Other device modes may also be
 * reflected as configuration options, such as whether two ISDN
 * channels are available independently; and choosing between open
 * standard device protocols (like CDC) or proprietary ones.
 *
 * Note that a non-authorized device (dev->authorized == 0) will only
 * be put in unconfigured mode.
 *
 * Note that USB has an additional level of device configurability,
 * associated with interfaces.  That configurability is accessed using
 * usb_set_interface().
 *
 * This call is synchronous. The calling context must be able to sleep,
 * must own the device lock, and must not hold the driver model's USB
 * bus mutex; usb interface driver probe() methods cannot use this routine.
 *
 * Returns zero on success, or else the status code returned by the
 * underlying call that failed.  On successful completion, each interface
 * in the original device configuration has been destroyed, and each one
 * in the new configuration has been probed by all relevant usb device
 * drivers currently known to the kernel.
 */
int etapi_usb_set_configuration(struct usb_device *dev, int configuration)
{
	int i, ret;
	struct usb_host_config *cp = NULL;
	struct usb_interface **new_interfaces = NULL;
	struct usb_hcd *hcd = bus_to_hcd(dev->bus);
	int n, nintf;

	if (dev->authorized == 0 || configuration == -1)
		configuration = 0;
	else {
		for (i = 0; i < dev->descriptor.bNumConfigurations; i++) {
			if (dev->config[i].desc.bConfigurationValue ==
					configuration) {
				cp = &dev->config[i];
				break;
			}
		}
	}
	if ((!cp && configuration != 0))
		return -EINVAL;

	/* The USB spec says configuration 0 means unconfigured.
	 * But if a device includes a configuration numbered 0,
	 * we will accept it as a correctly configured state.
	 * Use -1 if you really want to unconfigure the device.
	 */
	if (cp && configuration == 0)
		dev_warn(&dev->dev, "config 0 descriptor??\n");

	/* Allocate memory for new interfaces before doing anything else,
	 * so that if we run out then nothing will have changed. */
	n = nintf = 0;
	if (cp) {
		nintf = cp->desc.bNumInterfaces;
		new_interfaces = kmalloc(nintf * sizeof(*new_interfaces),
				GFP_KERNEL);
		if (!new_interfaces) {
			dev_err(&dev->dev, "Out of memory\n");
			return -ENOMEM;
		}

		for (; n < nintf; ++n) {
			new_interfaces[n] = kzalloc(
					sizeof(struct usb_interface),
					GFP_KERNEL);
			if (!new_interfaces[n]) {
				dev_err(&dev->dev, "Out of memory\n");
				ret = -ENOMEM;
free_interfaces:
				while (--n >= 0)
					kfree(new_interfaces[n]);
				kfree(new_interfaces);
				return ret;
			}
		}

		i = dev->bus_mA - cp->desc.bMaxPower * 2;
		if (i < 0)
			dev_warn(&dev->dev, "new config #%d exceeds power "
					"limit by %dmA\n",
					configuration, -i);
	}

	/* Wake up the device so we can send it the Set-Config request */
	ret = usb_autoresume_device(dev);
	if (ret)
		goto free_interfaces;

	/* if it's already configured, clear out old state first.
	 * getting rid of old interfaces means unbinding their drivers.
	 */
	if (dev->state != USB_STATE_ADDRESS)
		usb_disable_device(dev, 1);	/* Skip ep0 */

	/* Get rid of pending async Set-Config requests for this device */
	cancel_async_set_config(dev);

	/* Make sure we have bandwidth (and available HCD resources) for this
	 * configuration.  Remove endpoints from the schedule if we're dropping
	 * this configuration to set configuration 0.  After this point, the
	 * host controller will not allow submissions to dropped endpoints.  If
	 * this call fails, the device state is unchanged.
	 */
	mutex_lock(&hcd->bandwidth_mutex);
	ret = usb_hcd_alloc_bandwidth(dev, cp, NULL, NULL);
	if (ret < 0) {
		mutex_unlock(&hcd->bandwidth_mutex);
		usb_autosuspend_device(dev);
		goto free_interfaces;
	}

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
			      USB_REQ_SET_CONFIGURATION, 0, configuration, 0,
			      NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (ret < 0) {
		/* All the old state is gone, so what else can we do?
		 * The device is probably useless now anyway.
		 */
		cp = NULL;
	}

	dev->actconfig = cp;
	if (!cp) {
		usb_set_device_state(dev, USB_STATE_ADDRESS);
		usb_hcd_alloc_bandwidth(dev, NULL, NULL, NULL);
		mutex_unlock(&hcd->bandwidth_mutex);
		usb_autosuspend_device(dev);
		goto free_interfaces;
	}
	mutex_unlock(&hcd->bandwidth_mutex);
	usb_set_device_state(dev, USB_STATE_CONFIGURED);

	/* Initialize the new interface structures and the
	 * hc/hcd/usbcore interface/endpoint state.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface_cache *intfc;
		struct usb_interface *intf;
		struct usb_host_interface *alt;

		cp->interface[i] = intf = new_interfaces[i];
		intfc = cp->intf_cache[i];
		intf->altsetting = intfc->altsetting;
		intf->num_altsetting = intfc->num_altsetting;
		intf->intf_assoc = find_iad(dev, cp, i);
		kref_get(&intfc->ref);

		alt = usb_altnum_to_altsetting(intf, 0);

		/* No altsetting 0?  We'll assume the first altsetting.
		 * We could use a GetInterface call, but if a device is
		 * so non-compliant that it doesn't have altsetting 0
		 * then I wouldn't trust its reply anyway.
		 */
		if (!alt)
			alt = &intf->altsetting[0];

		intf->cur_altsetting = alt;
		usb_enable_interface(dev, intf, true);
		intf->dev.parent = &dev->dev;
		intf->dev.driver = NULL;
		intf->dev.bus = &usb_bus_type;
		intf->dev.type = &usb_if_device_type;
		intf->dev.groups = usb_interface_groups;
		intf->dev.dma_mask = dev->dev.dma_mask;
		INIT_WORK(&intf->reset_ws, __usb_queue_reset_device);
		device_initialize(&intf->dev);
		mark_quiesced(intf);
		dev_set_name(&intf->dev, "%d-%s:%d.%d",
			dev->bus->busnum, dev->devpath,
			configuration, alt->desc.bInterfaceNumber);
	}
	kfree(new_interfaces);

	if (cp->string == NULL &&
			!(dev->quirks & USB_QUIRK_CONFIG_INTF_STRINGS))
		cp->string = usb_cache_string(dev, cp->desc.iConfiguration);

	/* Now that all the interfaces are set up, register them
	 * to trigger binding of drivers to interfaces.  probe()
	 * routines may install different altsettings and may
	 * claim() any interfaces not yet bound.  Many class drivers
	 * need that: CDC, audio, video, etc.
	 */
	for (i = 0; i < nintf; ++i) {
		struct usb_interface *intf = cp->interface[i];

		dev_dbg(&dev->dev,
			"adding %s (config #%d, interface %d)\n",
			dev_name(&intf->dev), configuration,
			intf->cur_altsetting->desc.bInterfaceNumber);
		ret = device_add(&intf->dev);
		if (ret != 0) {
			dev_err(&dev->dev, "device_add(%s) --> %d\n",
				dev_name(&intf->dev), ret);
			continue;
		}
		create_intf_ep_devs(intf);
	}

	usb_autosuspend_device(dev);
	return 0;
}

static LIST_HEAD(set_config_list);
static DEFINE_SPINLOCK(set_config_lock);

struct set_config_request {
	struct usb_device	*udev;
	int			config;
	struct work_struct	work;
	struct list_head	node;
};

/* Worker routine for usb_driver_set_configuration() */
static void driver_set_config_work(struct work_struct *work)
{
	struct set_config_request *req =
		container_of(work, struct set_config_request, work);
	struct usb_device *udev = req->udev;

	usb_lock_device(udev);
	spin_lock(&set_config_lock);
	list_del(&req->node);
	spin_unlock(&set_config_lock);

	if (req->config >= -1)		/* Is req still valid? */
		usb_set_configuration(udev, req->config);
	usb_unlock_device(udev);
	usb_put_dev(udev);
	kfree(req);
}

/* Cancel pending Set-Config requests for a device whose configuration
 * was just changed
 */
static void cancel_async_set_config(struct usb_device *udev)
{
	struct set_config_request *req;

	spin_lock(&set_config_lock);
	list_for_each_entry(req, &set_config_list, node) {
		if (req->udev == udev)
			req->config = -999;	/* Mark as cancelled */
	}
	spin_unlock(&set_config_lock);
}

/**
 * usb_driver_set_configuration - Provide a way for drivers to change device configurations
 * @udev: the device whose configuration is being updated
 * @config: the configuration being chosen.
 * Context: In process context, must be able to sleep
 *
 * Device interface drivers are not allowed to change device configurations.
 * This is because changing configurations will destroy the interface the
 * driver is bound to and create new ones; it would be like a floppy-disk
 * driver telling the computer to replace the floppy-disk drive with a
 * tape drive!
 *
 * Still, in certain specialized circumstances the need may arise.  This
 * routine gets around the normal restrictions by using a work thread to
 * submit the change-config request.
 *
 * Returns 0 if the request was succesfully queued, error code otherwise.
 * The caller has no way to know whether the queued request will eventually
 * succeed.
 */
int etapi_usb_driver_set_configuration(struct usb_device *udev, int config)
{
	struct set_config_request *req;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	req->udev = udev;
	req->config = config;
	INIT_WORK(&req->work, driver_set_config_work);

	spin_lock(&set_config_lock);
	list_add(&req->node, &set_config_list);
	spin_unlock(&set_config_lock);

	usb_get_dev(udev);
	schedule_work(&req->work);
	return 0;
}
