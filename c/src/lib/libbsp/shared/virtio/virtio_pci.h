/*-
 * Copyright IBM Corp. 2007
 *
 * Authors:
 *  Anthony Liguori  <aliguori@us.ibm.com>
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY `
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: release/10.0.0/sys/dev/virtio/pci/virtio_pci.h 238360 2012-07-11 02:57:19Z grehan $
 */

#ifndef _VIRTIO_PCI_H
#define _VIRTIO_PCI_H

#ifdef __rtems__
struct vtpci_interrupt {
	uint32_t		isr_number;
	void			*vti_handler;
};

struct vtpci_virtqueue {
	struct virtqueue	*vtv_vq;
	int			 vtv_no_intr;
};

struct vtpci_softc {
	device_t			 vtpci_dev;
#ifdef NOTUSED
	struct resource			*vtpci_res;
	struct resource			*vtpci_msix_res;
#endif
#ifdef __rtems__
	int unit_number;
	char *unit_name;

	int pci_bus;
	int pci_dev;
	int pci_fun;
	uint32_t pci_io_base;
	rtems_id daemonTid;
#endif
	uint64_t			 vtpci_features;
	uint32_t			 vtpci_flags;

	/* This "bus" will only ever have one child. */
	device_t			 vtpci_child_dev;
	struct virtio_feature_desc	*vtpci_child_feat_desc;

	int				 vtpci_nvqs;
	struct vtpci_virtqueue		*vtpci_vqs;

	/*
	 * Ideally, each virtqueue that the driver provides a callback for will
	 * receive its own MSIX vector. If there are not sufficient vectors
	 * available, then attempt to have all the VQs share one vector. For
	 * MSIX, the configuration changed notifications must be on their own
	 * vector.
	 *
	 * If MSIX is not available, we will attempt to have the whole device
	 * share one MSI vector, and then, finally, one legacy interrupt.
	 */
	struct vtpci_interrupt		 vtpci_device_interrupt;
	struct vtpci_interrupt		*vtpci_msix_vq_interrupts;
	int				 vtpci_nmsix_resources;
};

int rtems_vtpci_attach(struct rtems_bsdnet_ifconfig *config, struct vtpci_softc **xsc);
uint64_t vtpci_negotiate_features(device_t, uint64_t);
int	vtpci_with_feature(device_t, uint64_t);
int	vtpci_alloc_virtqueues(device_t, int, int,
		    struct vq_alloc_info *);
int	vtpci_setup_intr(device_t, enum intr_type);
void vtpci_stop(device_t);
int	vtpci_reinit(device_t, uint64_t);
void vtpci_reinit_complete(device_t);
void vtpci_notify_virtqueue(device_t, uint16_t);
void vtpci_read_dev_config(device_t, bus_size_t, void *, int);
void vtpci_write_dev_config(device_t, bus_size_t, void *, int);
#endif /* __rtems__ */

/* VirtIO PCI vendor/device ID. */
#define VIRTIO_PCI_VENDORID	0x1AF4
#define VIRTIO_PCI_DEVICEID_MIN	0x1000
#define VIRTIO_PCI_DEVICEID_MAX	0x103F

/* VirtIO ABI version, this must match exactly. */
#define VIRTIO_PCI_ABI_VERSION	0

/*
 * VirtIO Header, located in BAR 0.
 */
#define VIRTIO_PCI_HOST_FEATURES  0  /* host's supported features (32bit, RO)*/
#define VIRTIO_PCI_GUEST_FEATURES 4  /* guest's supported features (32, RW) */
#define VIRTIO_PCI_QUEUE_PFN      8  /* physical address of VQ (32, RW) */
#define VIRTIO_PCI_QUEUE_NUM      12 /* number of ring entries (16, RO) */
#define VIRTIO_PCI_QUEUE_SEL      14 /* current VQ selection (16, RW) */
#define VIRTIO_PCI_QUEUE_NOTIFY	  16 /* notify host regarding VQ (16, RW) */
#define VIRTIO_PCI_STATUS         18 /* device status register (8, RW) */
#define VIRTIO_PCI_ISR            19 /* interrupt status register, reading
				      * also clears the register (8, RO) */
/* Only if MSIX is enabled: */
#define VIRTIO_MSI_CONFIG_VECTOR  20 /* configuration change vector (16, RW) */
#define VIRTIO_MSI_QUEUE_VECTOR   22 /* vector for selected VQ notifications
					(16, RW) */

/* The bit of the ISR which indicates a device has an interrupt. */
#define VIRTIO_PCI_ISR_INTR	0x1
/* The bit of the ISR which indicates a device configuration change. */
#define VIRTIO_PCI_ISR_CONFIG	0x2
/* Vector value used to disable MSI for queue. */
#define VIRTIO_MSI_NO_VECTOR	0xFFFF

/*
 * The remaining space is defined by each driver as the per-driver
 * configuration space.
 */
#define VIRTIO_PCI_CONFIG(sc) \
    (((sc)->vtpci_flags & VTPCI_FLAG_MSIX) ? 24 : 20)

/*
 * How many bits to shift physical queue address written to QUEUE_PFN.
 * 12 is historical, and due to x86 page size.
 */
#define VIRTIO_PCI_QUEUE_ADDR_SHIFT	12

/* The alignment to use between consumer and producer parts of vring. */
#define VIRTIO_PCI_VRING_ALIGN	4096

#endif /* _VIRTIO_PCI_H */
