// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 */
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/msm_ion.h>

#include "adsprpc_compat.h"
#include "adsprpc_shared.h"

#define COMPAT_FASTRPC_IOCTL_INVOKE \
		_IOWR('R', 1, struct compat_fastrpc_ioctl_invoke)
#define COMPAT_FASTRPC_IOCTL_MMAP \
		_IOWR('R', 2, struct compat_fastrpc_ioctl_mmap)
#define COMPAT_FASTRPC_IOCTL_MUNMAP \
		_IOWR('R', 3, struct compat_fastrpc_ioctl_munmap)
#define COMPAT_FASTRPC_IOCTL_INVOKE_FD \
		_IOWR('R', 4, struct compat_fastrpc_ioctl_invoke_fd)
#define COMPAT_FASTRPC_IOCTL_INIT \
		_IOWR('R', 6, struct compat_fastrpc_ioctl_init)
#define COMPAT_FASTRPC_IOCTL_INVOKE_ATTRS \
		_IOWR('R', 7, struct compat_fastrpc_ioctl_invoke_attrs)
#define COMPAT_FASTRPC_IOCTL_INIT_ATTRS \
		_IOWR('R', 10, struct compat_fastrpc_ioctl_init_attrs)
#define COMPAT_FASTRPC_IOCTL_INVOKE_CRC \
		_IOWR('R', 11, struct compat_fastrpc_ioctl_invoke_crc)
#define COMPAT_FASTRPC_IOCTL_CONTROL \
		_IOWR('R', 12, struct compat_fastrpc_ioctl_control)
#define COMPAT_FASTRPC_IOCTL_MMAP_64 \
		_IOWR('R', 14, struct compat_fastrpc_ioctl_mmap_64)
#define COMPAT_FASTRPC_IOCTL_MUNMAP_64 \
		_IOWR('R', 15, struct compat_fastrpc_ioctl_munmap_64)
#define COMPAT_FASTRPC_IOCTL_GET_DSP_INFO \
		_IOWR('R', 17, \
			struct compat_fastrpc_ioctl_capability)
#define COMPAT_FASTRPC_IOCTL_INVOKE2 \
			 _IOWR('R', 18, struct compat_fastrpc_ioctl_invoke2)
#define COMPAT_FASTRPC_IOCTL_MEM_MAP \
		_IOWR('R', 19, struct compat_fastrpc_ioctl_mem_map)
#define COMPAT_FASTRPC_IOCTL_MEM_UNMAP \
		_IOWR('R', 20, struct compat_fastrpc_ioctl_mem_unmap)
#define COMPAT_FASTRPC_IOCTL_INVOKE_PERF \
		_IOWR('R', 21, struct compat_fastrpc_ioctl_invoke_perf)

struct compat_remote_buf {
	compat_uptr_t pv;	/* buffer pointer */
	compat_size_t len;	/* length of buffer */
};

union compat_remote_arg {
	struct compat_remote_buf buf;
	compat_uint_t h;
};

struct compat_fastrpc_ioctl_invoke {
	compat_uint_t handle;	/* remote handle */
	compat_uint_t sc;	/* scalars describing the data */
	compat_uptr_t pra;	/* remote arguments list */
};

struct compat_fastrpc_ioctl_invoke_fd {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;	/* fd list */
};

struct compat_fastrpc_ioctl_invoke_attrs {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;	/* fd list */
	compat_uptr_t attrs;	/* attribute list */
};

struct compat_fastrpc_ioctl_invoke_crc {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;	/* fd list */
	compat_uptr_t attrs;	/* attribute list */
	compat_uptr_t crc;	/* crc list */
};

struct compat_fastrpc_ioctl_invoke_perf {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;
	compat_uptr_t attrs;
	compat_uptr_t crc;
	compat_uptr_t perf_kernel;
	compat_uptr_t perf_dsp;
};

struct compat_fastrpc_ioctl_invoke_async {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;			/* fd list */
	compat_uptr_t attrs;		/* attribute list */
	compat_uptr_t crc;			/* crc list */
	compat_uptr_t perf_kernel;	/* Kernel perf data pointer */
	compat_uptr_t perf_dsp;		/* DSP perf data pointer */
	compat_uptr_t job;			/* Async job */
};
struct compat_fastrpc_ioctl_invoke_async_no_perf {
	struct compat_fastrpc_ioctl_invoke inv;
	compat_uptr_t fds;			/* fd list */
	compat_uptr_t attrs;		/* attribute list */
	compat_uptr_t crc;			/* crc list */
	compat_uptr_t job;			/* Async job */
};

struct compat_fastrpc_ioctl_invoke2 {
	compat_uint_t req;		/* type of invocation request */
	compat_uptr_t invparam;	/* invocation request param */
	compat_uint_t size;		/* size of invocation param */
	compat_int_t  err;		/* reserved */
};
struct compat_fastrpc_ioctl_async_response {
	compat_u64 jobid;			 /* job id generated by user */
	compat_int_t result;		 /* result from DSP */
	compat_uptr_t perf_kernel;	 /* Kernel perf data pointer */
	compat_uptr_t perf_dsp;		 /* DSP perf data pointer */
	compat_uint_t handle;
	compat_uint_t sc;
};

struct compat_fastrpc_mem_map {
	compat_int_t fd;	/* ion fd */
	compat_int_t offset;	/* buffer offset */
	compat_uint_t flags;	/* flags to control memory map */
	compat_uint_t attrs;	/* buffer attributes used for SMMU mapping */
	compat_uptr_t vaddrin;	/* virtual address */
	compat_size_t length;	/* buffer length */
	compat_u64 vaddrout;	/* dsp virtual address */
};

struct compat_fastrpc_ioctl_mem_map {
	compat_int_t version;
	union {
		struct compat_fastrpc_mem_map m;
		compat_int_t reserved[MAP_RESERVED_NUM];
	};
};

struct compat_fastrpc_mem_unmap {
	compat_int_t fd;		/* ion fd */
	compat_u64 vaddr;		/* dsp virtual address */
	compat_size_t length;		/* buffer length */
};

struct compat_fastrpc_ioctl_mem_unmap {
	compat_int_t version;
	union {
		struct compat_fastrpc_mem_unmap um;
		compat_int_t reserved[UNMAP_RESERVED_NUM];
	};
};

struct compat_fastrpc_ioctl_mmap {
	compat_int_t fd;	/* ion fd */
	compat_uint_t flags;	/* flags for dsp to map with */
	compat_uptr_t vaddrin;	/* optional virtual address */
	compat_size_t size;	/* size */
	compat_uptr_t vaddrout;	/* dsps virtual address */
};

struct compat_fastrpc_ioctl_mmap_64 {
	compat_int_t fd;	/* ion fd */
	compat_uint_t flags;	/* flags for dsp to map with */
	compat_u64 vaddrin;	/* optional virtual address */
	compat_size_t size;	/* size */
	compat_u64 vaddrout;	/* dsps virtual address */
};

struct compat_fastrpc_ioctl_munmap {
	compat_uptr_t vaddrout;	/* address to unmap */
	compat_size_t size;	/* size */
};

struct compat_fastrpc_ioctl_munmap_64 {
	compat_u64 vaddrout;	/* address to unmap */
	compat_size_t size;	/* size */
};

struct compat_fastrpc_ioctl_init {
	compat_uint_t flags;	/* one of FASTRPC_INIT_* macros */
	compat_uptr_t file;	/* pointer to elf file */
	compat_int_t filelen;	/* elf file length */
	compat_int_t filefd;	/* ION fd for the file */
	compat_uptr_t mem;	/* mem for the PD */
	compat_int_t memlen;	/* mem length */
	compat_int_t memfd;	/* ION fd for the mem */
};

struct compat_fastrpc_ioctl_init_attrs {
	struct compat_fastrpc_ioctl_init init;
	compat_int_t attrs;	/* attributes to init process */
	compat_int_t siglen;	/* test signature file length */
};

#define FASTRPC_CONTROL_LATENCY		(1)
struct compat_fastrpc_ctrl_latency {
	compat_uint_t enable;	/* latency control enable */
	compat_uint_t latency;	/* target latency in us */
};

#define FASTRPC_CONTROL_KALLOC		(3)
struct compat_fastrpc_ctrl_kalloc {
	compat_uint_t kalloc_support; /* Remote memory allocation from kernel */
};

struct compat_fastrpc_ctrl_wakelock {
	compat_uint_t enable;	/* wakelock control enable */
};

struct compat_fastrpc_ctrl_pm {
	compat_uint_t timeout;	/* timeout(in ms) for PM to keep system awake */
};

struct compat_fastrpc_ioctl_control {
	compat_uint_t req;
	union {
		struct compat_fastrpc_ctrl_latency lp;
		struct compat_fastrpc_ctrl_kalloc kalloc;
		struct compat_fastrpc_ctrl_wakelock wp;
		struct compat_fastrpc_ctrl_pm pm;
	};
};

struct compat_fastrpc_ioctl_capability {
	/*
	 * @param[in]: DSP domain ADSP_DOMAIN_ID,
	 * SDSP_DOMAIN_ID, or CDSP_DOMAIN_ID
	 */
	compat_uint_t domain;
	/*
	 * @param[in]: One of the DSP attributes
	 * from enum remote_dsp_attributes
	 */
	compat_uint_t attribute_ID;
	/*
	 * @param[out]: Result of the DSP
	 * capability query based on attribute_ID
	 */
	compat_uint_t capability;
};

static int compat_get_fastrpc_ioctl_invoke(
			struct compat_fastrpc_ioctl_invoke_async __user *inv32,
			struct fastrpc_ioctl_invoke_async __user *inv,
			unsigned int cmd, unsigned int sc)
{
	compat_uint_t u = 0;
	compat_size_t s;
	compat_uptr_t p, k;
	union compat_remote_arg *pra32;
	union remote_arg *pra;
	int err = 0, len = 0, j = 0;

	len = REMOTE_SCALARS_LENGTH(sc);

	pra = (union remote_arg *)(inv + 1);
	err = put_user(pra, &inv->inv.pra);
	err |= put_user(sc, &inv->inv.sc);
	err |= get_user(u, &inv32->inv.handle);
	err |= put_user(u, &inv->inv.handle);
	err |= get_user(p, &inv32->inv.pra);
	if (err)
		return err;
	pra32 = compat_ptr(p);
	pra = (union remote_arg *)(inv + 1);
	for (j = 0; j < len; j++) {
		err |= get_user(p, &pra32[j].buf.pv);
		err |= put_user(p, (uintptr_t *)&pra[j].buf.pv);
		err |= get_user(s, &pra32[j].buf.len);
		err |= put_user(s, &pra[j].buf.len);
	}

	err |= put_user(NULL, &inv->fds);
	if (cmd != COMPAT_FASTRPC_IOCTL_INVOKE) {
		err |= get_user(p, &inv32->fds);
		err |= put_user(p, (compat_uptr_t *)&inv->fds);
	}
	err |= put_user(NULL, &inv->attrs);
	if ((cmd == COMPAT_FASTRPC_IOCTL_INVOKE_ATTRS) ||
		(cmd == COMPAT_FASTRPC_IOCTL_INVOKE_CRC) ||
		(cmd == COMPAT_FASTRPC_IOCTL_INVOKE_PERF) ||
		(cmd == FASTRPC_INVOKE2_ASYNC)) {
		err |= get_user(p, &inv32->attrs);
		err |= put_user(p, (compat_uptr_t *)&inv->attrs);
	}
	err |= put_user(NULL, (compat_uptr_t __user **)&inv->crc);
	if ((cmd == COMPAT_FASTRPC_IOCTL_INVOKE_CRC) ||
		(cmd == COMPAT_FASTRPC_IOCTL_INVOKE_PERF)) {
		err |= get_user(p, &inv32->crc);
		err |= put_user(p, (compat_uptr_t __user *)&inv->crc);
	}
	err |= put_user(NULL, &inv->job);
	if (cmd == FASTRPC_INVOKE2_ASYNC) {
		err |= get_user(p, &inv32->job);
		err |= put_user(p, (compat_uptr_t __user *)&inv->job);
	}
	err |= put_user((compat_uptr_t)0, (compat_uptr_t __user *)&inv->perf_kernel);
	err |= put_user((compat_uptr_t)0, (compat_uptr_t __user *)&inv->perf_dsp);
	if ((cmd == COMPAT_FASTRPC_IOCTL_INVOKE_PERF) || (cmd == FASTRPC_INVOKE2_ASYNC)) {
		err |= get_user(k, &inv32->perf_kernel);
		err |= get_user(p, &inv32->perf_dsp);
		err |= put_user(k, (compat_uptr_t __user *)&inv->perf_kernel);
		err |= put_user(p, (compat_uptr_t __user *)&inv->perf_dsp);
	}
	return err;
}

static int compat_fastrpc_ioctl_invoke(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct compat_fastrpc_ioctl_invoke_async __user *inv32;
	struct fastrpc_ioctl_invoke_async __user *inv;
	compat_uint_t sc = 0;
	int err = 0, len = 0;

	inv32 = compat_ptr(arg);
	err = get_user(sc, &inv32->inv.sc);
	if (err)
		return err;
	len = REMOTE_SCALARS_LENGTH(sc);
	VERIFY(err, NULL != (inv = compat_alloc_user_space(
		sizeof(*inv) + len * sizeof(union remote_arg))));
	if (err)
		return -EFAULT;
	VERIFY(err, 0 == compat_get_fastrpc_ioctl_invoke(inv32,
						inv, cmd, sc));
	if (err)
		return err;
	return filp->f_op->unlocked_ioctl(filp,
			FASTRPC_IOCTL_INVOKE_PERF, (unsigned long)inv);
}

static int compat_get_fastrpc_ioctl_invoke2(
			struct compat_fastrpc_ioctl_invoke2 __user *inv32,
			struct fastrpc_ioctl_invoke2 __user **inva,
			unsigned int cmd)
{
	int err = 0;
	compat_uptr_t pparam, p;
	compat_uint_t req, size, ref_size = 0;
	struct fastrpc_ioctl_invoke2 __user *inv2_user = NULL;
	struct fastrpc_ioctl_invoke_async __user *asyncinv_user;

	err = get_user(req, &inv32->req);
	err |= get_user(pparam, &inv32->invparam);
	err |= get_user(size, &inv32->size);
	if (err)
		goto bail;

	switch (req) {
	case FASTRPC_INVOKE2_ASYNC:
	{
		struct compat_fastrpc_ioctl_invoke_async __user *lasync32;
		struct compat_fastrpc_ioctl_invoke_async_no_perf __user *lasync32_old;
		compat_uint_t sc = 0;
		int len = 0;

		VERIFY(err, size <= sizeof(*lasync32));
		if (err) {
			err = -EBADE;
			goto bail;
		}
		lasync32 = compat_ptr(pparam);
		err = get_user(sc, &lasync32->inv.sc);
		if (err)
			goto bail;
		len = REMOTE_SCALARS_LENGTH(sc);
		VERIFY(err, NULL != (inv2_user = compat_alloc_user_space(
				sizeof(*inv2_user) + sizeof(*asyncinv_user) +
					len * sizeof(union remote_arg))));
		if (err) {
			err = -EFAULT;
			goto bail;
		}
		asyncinv_user =
		(struct fastrpc_ioctl_invoke_async __user *)(inv2_user + 1);
		if (size < sizeof(struct compat_fastrpc_ioctl_invoke_async)) {
			lasync32_old = compat_ptr(pparam);
			VERIFY(err, 0 == compat_get_fastrpc_ioctl_invoke(lasync32,
					asyncinv_user, COMPAT_FASTRPC_IOCTL_INVOKE_CRC, sc));
			if (err)
				goto bail;
			err |= put_user(NULL, &asyncinv_user->job);
			err |= get_user(p, &lasync32_old->job);
			err |= put_user(p, (compat_uptr_t __user *)&asyncinv_user->job);
			err |= put_user((compat_uptr_t)0,
				(compat_uptr_t __user *)&asyncinv_user->perf_kernel);
			err |= put_user((compat_uptr_t)0,
				(compat_uptr_t __user *)&asyncinv_user->perf_dsp);
		} else {
			VERIFY(err, 0 == compat_get_fastrpc_ioctl_invoke(lasync32,
							asyncinv_user, req, sc));
		}
		if (err)
			goto bail;
		err |= put_user(req, &inv2_user->req);
		err |= put_user((uintptr_t __user)asyncinv_user,
							&inv2_user->invparam);
		err |= put_user(sizeof(*asyncinv_user), &inv2_user->size);
		if (err)
			goto bail;
		break;
	}
	case FASTRPC_INVOKE2_ASYNC_RESPONSE:
		ref_size = sizeof(struct compat_fastrpc_ioctl_async_response);
		VERIFY(err, size <= ref_size);
		if (err) {
			err = -EBADE;
			goto bail;
		}
		/* intentional fall through */
	case FASTRPC_INVOKE2_KERNEL_OPTIMIZATIONS:
	{
		if (!ref_size) {
			ref_size = sizeof(uint32_t);
			VERIFY(err, size == ref_size);
			if (err) {
				err = -EBADE;
				goto bail;
			}
		}
		VERIFY(err, NULL != (inv2_user = compat_alloc_user_space(
							sizeof(*inv2_user))));
		if (err) {
			err = -EFAULT;
			goto bail;
		}
		err |= put_user(req, &inv2_user->req);
		err |=
		put_user(pparam, &inv2_user->invparam);
		err |= put_user(size, &inv2_user->size);
		if (err)
			goto bail;
		break;
	}
	default:
		err = -ENOTTY;
		break;
	}
	*inva = inv2_user;
bail:
	return err;
}

static int compat_fastrpc_ioctl_invoke2(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct compat_fastrpc_ioctl_invoke2 __user *inv32;
	struct fastrpc_ioctl_invoke2 __user *inv;
	int err = 0;

	inv32 = compat_ptr(arg);
	VERIFY(err, 0 == compat_get_fastrpc_ioctl_invoke2(inv32,
							&inv, cmd));
	if (err)
		return err;
	return filp->f_op->unlocked_ioctl(filp,
				FASTRPC_IOCTL_INVOKE2, (unsigned long)inv);
}

static int compat_get_fastrpc_ioctl_mem_map(
			struct compat_fastrpc_ioctl_mem_map __user *map32,
			struct fastrpc_ioctl_mem_map __user *map)
{
	compat_uint_t u;
	compat_int_t i;
	compat_size_t s;
	compat_uptr_t p;
	int err;

	err = get_user(i, &map32->version);
	if (err || i != 0)
		return -EINVAL;

	err = put_user(i, &map->version);
	err |= get_user(i, &map32->m.fd);
	err |= put_user(i, &map->m.fd);
	err |= get_user(i, &map32->m.offset);
	err |= put_user(i, &map->m.offset);
	err |= get_user(u, &map32->m.flags);
	err |= put_user(u, &map->m.flags);
	err |= get_user(p, &map32->m.vaddrin);
	err |= put_user(p, &map->m.vaddrin);
	err |= get_user(s, &map32->m.length);
	err |= put_user(s, &map->m.length);
	err |= get_user(u, &map32->m.attrs);
	err |= put_user(u, &map->m.attrs);

	return err;
}

static int compat_put_fastrpc_ioctl_mem_map(
			struct compat_fastrpc_ioctl_mem_map __user *map32,
			struct fastrpc_ioctl_mem_map __user *map)
{
	compat_u64 p;
	int err;

	err = get_user(p, &map->m.vaddrout);
	err |= put_user(p, &map32->m.vaddrout);

	return err;
}

static int compat_get_fastrpc_ioctl_mem_unmap(
			struct compat_fastrpc_ioctl_mem_unmap __user *unmap32,
			struct fastrpc_ioctl_mem_unmap __user *unmap)
{
	compat_int_t i;
	compat_size_t s;
	compat_u64 p;
	int err;

	err = get_user(i, &unmap32->version);
	if (err || i != 0)
		return -EINVAL;

	err = put_user(i, &unmap->version);
	err |= get_user(i, &unmap32->um.fd);
	err |= put_user(i, &unmap->um.fd);
	err |= get_user(p, &unmap32->um.vaddr);
	err |= put_user(p, &unmap->um.vaddr);
	err |= get_user(s, &unmap32->um.length);
	err |= put_user(s, &unmap->um.length);

	return err;
}

static int compat_get_fastrpc_ioctl_mmap(
			struct compat_fastrpc_ioctl_mmap __user *map32,
			struct fastrpc_ioctl_mmap __user *map)
{
	compat_uint_t u;
	compat_int_t i;
	compat_size_t s;
	compat_uptr_t p;
	int err;

	err = get_user(i, &map32->fd);
	err |= put_user(i, &map->fd);
	err |= get_user(u, &map32->flags);
	err |= put_user(u, &map->flags);
	err |= get_user(p, &map32->vaddrin);
	err |= put_user(p, (uintptr_t *)&map->vaddrin);
	err |= get_user(s, &map32->size);
	err |= put_user(s, &map->size);

	return err;
}

static int compat_get_fastrpc_ioctl_mmap_64(
			struct compat_fastrpc_ioctl_mmap_64 __user *map32,
			struct fastrpc_ioctl_mmap __user *map)
{
	compat_uint_t u;
	compat_int_t i;
	compat_size_t s;
	compat_u64 p;
	int err;

	err = get_user(i, &map32->fd);
	err |= put_user(i, &map->fd);
	err |= get_user(u, &map32->flags);
	err |= put_user(u, &map->flags);
	err |= get_user(p, &map32->vaddrin);
	err |= put_user(p, &map->vaddrin);
	err |= get_user(s, &map32->size);
	err |= put_user(s, &map->size);

	return err;
}

static int compat_put_fastrpc_ioctl_mmap(
			struct compat_fastrpc_ioctl_mmap __user *map32,
			struct fastrpc_ioctl_mmap __user *map)
{
	compat_uptr_t p;
	int err;

	err = get_user(p, &map->vaddrout);
	err |= put_user(p, &map32->vaddrout);

	return err;
}

static int compat_put_fastrpc_ioctl_mmap_64(
			struct compat_fastrpc_ioctl_mmap_64 __user *map32,
			struct fastrpc_ioctl_mmap __user *map)
{
	compat_u64 p;
	int err;

	err = get_user(p, &map->vaddrout);
	err |= put_user(p, &map32->vaddrout);

	return err;
}

static int compat_get_fastrpc_ioctl_munmap(
			struct compat_fastrpc_ioctl_munmap __user *unmap32,
			struct fastrpc_ioctl_munmap __user *unmap)
{
	compat_uptr_t p;
	compat_size_t s;
	int err;

	err = get_user(p, &unmap32->vaddrout);
	err |= put_user(p, &unmap->vaddrout);
	err |= get_user(s, &unmap32->size);
	err |= put_user(s, &unmap->size);

	return err;
}

static int compat_get_fastrpc_ioctl_munmap_64(
			struct compat_fastrpc_ioctl_munmap_64 __user *unmap32,
			struct fastrpc_ioctl_munmap __user *unmap)
{
	compat_u64 p;
	compat_size_t s;
	int err;

	err = get_user(p, &unmap32->vaddrout);
	err |= put_user(p, &unmap->vaddrout);
	err |= get_user(s, &unmap32->size);
	err |= put_user(s, &unmap->size);

	return err;
}

static int compat_get_fastrpc_ioctl_control(
			struct compat_fastrpc_ioctl_control __user *ctrl32,
			struct fastrpc_ioctl_control __user *ctrl)
{
	compat_uptr_t p;
	int err;

	err = get_user(p, &ctrl32->req);
	err |= put_user(p, &ctrl->req);
	if (p == FASTRPC_CONTROL_LATENCY) {
		err |= get_user(p, &ctrl32->lp.enable);
		err |= put_user(p, &ctrl->lp.enable);
		err |= get_user(p, &ctrl32->lp.latency);
		err |= put_user(p, &ctrl->lp.latency);
	} else if (p == FASTRPC_CONTROL_WAKELOCK) {
		err |= get_user(p, &ctrl32->wp.enable);
		err |= put_user(p, &ctrl->wp.enable);
	} else if (p == FASTRPC_CONTROL_PM) {
		err |= get_user(p, &ctrl32->pm.timeout);
		err |= put_user(p, &ctrl->pm.timeout);
	}

	return err;
}

static int compat_get_fastrpc_ioctl_init(
			struct compat_fastrpc_ioctl_init_attrs __user *init32,
			struct fastrpc_ioctl_init_attrs __user *init,
			unsigned int cmd)
{
	compat_uint_t u;
	compat_uptr_t p;
	compat_int_t i;
	int err;

	err = get_user(u, &init32->init.flags);
	err |= put_user(u, &init->init.flags);
	err |= get_user(p, &init32->init.file);
	err |= put_user(p, &init->init.file);
	err |= get_user(i, &init32->init.filelen);
	err |= put_user(i, &init->init.filelen);
	err |= get_user(i, &init32->init.filefd);
	err |= put_user(i, &init->init.filefd);
	err |= get_user(p, &init32->init.mem);
	err |= put_user(p, &init->init.mem);
	err |= get_user(i, &init32->init.memlen);
	err |= put_user(i, &init->init.memlen);
	err |= get_user(i, &init32->init.memfd);
	err |= put_user(i, &init->init.memfd);

	err |= put_user(0, &init->attrs);
	if (cmd == COMPAT_FASTRPC_IOCTL_INIT_ATTRS) {
		err |= get_user(i, &init32->attrs);
		err |= put_user(i, (compat_uptr_t *)&init->attrs);
	}

	err |= put_user(0, &init->siglen);
	if (cmd == COMPAT_FASTRPC_IOCTL_INIT_ATTRS) {
		err |= get_user(i, &init32->siglen);
		err |= put_user(i, (compat_uptr_t *)&init->siglen);
	}

	return err;
}

static int compat_put_fastrpc_ioctl_get_dsp_info(
	struct compat_fastrpc_ioctl_capability __user *info32,
	struct fastrpc_ioctl_capability __user *info)
{
	compat_uint_t u;
	int err = 0;

	err |= get_user(u, &info->capability);
	err |= put_user(u, &info32->capability);
	return err;
}

static int fastrpc_setmode(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	return filp->f_op->unlocked_ioctl(filp, cmd,
					(unsigned long)compat_ptr(arg));
}

static int compat_fastrpc_control(struct file *filp,
		unsigned long arg)
{
	int err = 0;
	struct compat_fastrpc_ioctl_control __user *ctrl32;
	struct fastrpc_ioctl_control __user *ctrl;
	compat_uptr_t p;

	ctrl32 = compat_ptr(arg);
	VERIFY(err, NULL != (ctrl = compat_alloc_user_space(
						sizeof(*ctrl))));
	if (err)
		return -EFAULT;
	VERIFY(err, 0 == compat_get_fastrpc_ioctl_control(ctrl32,
						ctrl));
	if (err)
		return err;
	err = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_CONTROL,
						(unsigned long)ctrl);
	if (err)
		return err;
	err = get_user(p, &ctrl32->req);
	if (err)
		return err;
	if (p == FASTRPC_CONTROL_KALLOC) {
		err = get_user(p, &ctrl->kalloc.kalloc_support);
		err |= put_user(p, &ctrl32->kalloc.kalloc_support);
	}
	return err;
}

static int compat_fastrpc_get_dsp_info(struct file *filp,
		unsigned long arg)
{
	struct compat_fastrpc_ioctl_capability __user *info32;
	struct fastrpc_ioctl_capability __user *info;
	compat_uint_t u;
	long ret;
	int err = 0;

	info32 = compat_ptr(arg);
	VERIFY(err, NULL != (info = compat_alloc_user_space(
						sizeof(*info))));
	if (err)
		return -EFAULT;

	err = get_user(u, &info32->domain);
	err |= put_user(u, &info->domain);

	err = get_user(u, &info32->attribute_ID);
	err |= put_user(u, &info->attribute_ID);

	if (err)
		return err;

	ret = filp->f_op->unlocked_ioctl(filp,
			FASTRPC_IOCTL_GET_DSP_INFO,
			(unsigned long)info);
	if (ret)
		return ret;

	err = compat_put_fastrpc_ioctl_get_dsp_info(info32, info);
	return err;
}

static inline long compat_fastrpc_mmap_device_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {
	case COMPAT_FASTRPC_IOCTL_MEM_MAP:
	{
		struct compat_fastrpc_ioctl_mem_map __user *map32;
		struct fastrpc_ioctl_mem_map __user *map;
		long ret;

		map32 = compat_ptr(arg);
		map = compat_alloc_user_space(sizeof(*map));
		if (map == NULL)
			return -EFAULT;

		err = compat_get_fastrpc_ioctl_mem_map(map32, map);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MEM_MAP,
							(unsigned long)map);
		if (ret)
			return ret;
		VERIFY(err, 0 == compat_put_fastrpc_ioctl_mem_map(map32, map));
		return err;
	}
	case COMPAT_FASTRPC_IOCTL_MEM_UNMAP:
	{
		struct compat_fastrpc_ioctl_mem_unmap __user *unmap32;
		struct fastrpc_ioctl_mem_unmap __user *unmap;
		long ret;

		unmap32 = compat_ptr(arg);
		unmap = compat_alloc_user_space(sizeof(*unmap));
		if (unmap == NULL)
			return -EFAULT;

		err = compat_get_fastrpc_ioctl_mem_unmap(unmap32, unmap);
		if (err)
			return err;

		ret = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MEM_UNMAP,
							(unsigned long)unmap);
		if (ret)
			return ret;
		return err;
	}
	case COMPAT_FASTRPC_IOCTL_MMAP:
	{
		struct compat_fastrpc_ioctl_mmap __user *map32;
		struct fastrpc_ioctl_mmap __user *map;
		long ret;

		map32 = compat_ptr(arg);
		VERIFY(err, NULL != (map = compat_alloc_user_space(
							sizeof(*map))));
		if (err)
			return -EFAULT;
		VERIFY(err, 0 == compat_get_fastrpc_ioctl_mmap(map32, map));
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MMAP,
							(unsigned long)map);
		if (ret)
			return ret;
		VERIFY(err, 0 == compat_put_fastrpc_ioctl_mmap(map32, map));
		return err;
	}
	case COMPAT_FASTRPC_IOCTL_MMAP_64:
	{
		struct compat_fastrpc_ioctl_mmap_64  __user *map32;
		struct fastrpc_ioctl_mmap __user *map;
		long ret;

		map32 = compat_ptr(arg);
		VERIFY(err, NULL != (map = compat_alloc_user_space(
							sizeof(*map))));
		if (err)
			return -EFAULT;
		VERIFY(err, 0 == compat_get_fastrpc_ioctl_mmap_64(map32, map));
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MMAP_64,
							(unsigned long)map);
		if (ret)
			return ret;
		VERIFY(err, 0 == compat_put_fastrpc_ioctl_mmap_64(map32, map));
		return err;
	}
	case COMPAT_FASTRPC_IOCTL_MUNMAP:
	{
		struct compat_fastrpc_ioctl_munmap __user *unmap32;
		struct fastrpc_ioctl_munmap __user *unmap;

		unmap32 = compat_ptr(arg);
		VERIFY(err, NULL != (unmap = compat_alloc_user_space(
							sizeof(*unmap))));
		if (err)
			return -EFAULT;
		VERIFY(err, 0 == compat_get_fastrpc_ioctl_munmap(unmap32,
							unmap));
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MUNMAP,
							(unsigned long)unmap);
	}
	default:
		return -ENOIOCTLCMD;
	}
}

long compat_fastrpc_device_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	int err = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_FASTRPC_IOCTL_INVOKE:
	case COMPAT_FASTRPC_IOCTL_INVOKE_FD:
	case COMPAT_FASTRPC_IOCTL_INVOKE_ATTRS:
	case COMPAT_FASTRPC_IOCTL_INVOKE_CRC:
	case COMPAT_FASTRPC_IOCTL_INVOKE_PERF:
	{
		return compat_fastrpc_ioctl_invoke(filp, cmd, arg);
	}
	case COMPAT_FASTRPC_IOCTL_INVOKE2:
	{
		return compat_fastrpc_ioctl_invoke2(filp, cmd, arg);
	}
	case COMPAT_FASTRPC_IOCTL_MUNMAP_64:
	{
		struct compat_fastrpc_ioctl_munmap_64 __user *unmap32;
		struct fastrpc_ioctl_munmap __user *unmap;

		unmap32 = compat_ptr(arg);
		VERIFY(err, NULL != (unmap = compat_alloc_user_space(
							sizeof(*unmap))));
		if (err)
			return -EFAULT;
		VERIFY(err, 0 == compat_get_fastrpc_ioctl_munmap_64(unmap32,
							unmap));
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_MUNMAP_64,
							(unsigned long)unmap);
	}
	case COMPAT_FASTRPC_IOCTL_INIT:
		/* fall through */
	case COMPAT_FASTRPC_IOCTL_INIT_ATTRS:
	{
		struct compat_fastrpc_ioctl_init_attrs __user *init32;
		struct fastrpc_ioctl_init_attrs __user *init;

		init32 = compat_ptr(arg);
		VERIFY(err, NULL != (init = compat_alloc_user_space(
							sizeof(*init))));
		if (err)
			return -EFAULT;
		VERIFY(err, 0 == compat_get_fastrpc_ioctl_init(init32,
							init, cmd));
		if (err)
			return err;
		return filp->f_op->unlocked_ioctl(filp,
			 FASTRPC_IOCTL_INIT_ATTRS, (unsigned long)init);
	}
	case FASTRPC_IOCTL_GETINFO:
	{
		compat_uptr_t __user *info32;
		uint32_t __user *info;
		compat_uint_t u;
		long ret;

		info32 = compat_ptr(arg);
		VERIFY(err, NULL != (info = compat_alloc_user_space(
							sizeof(*info))));
		if (err)
			return -EFAULT;
		err = get_user(u, info32);
		err |= put_user(u, info);
		if (err)
			return err;
		ret = filp->f_op->unlocked_ioctl(filp, FASTRPC_IOCTL_GETINFO,
							(unsigned long)info);
		if (ret)
			return ret;
		err = get_user(u, info);
		err |= put_user(u, info32);
		return err;
	}
	case FASTRPC_IOCTL_SETMODE:
		return fastrpc_setmode(filp, cmd, arg);
	case COMPAT_FASTRPC_IOCTL_CONTROL:
	{
		return compat_fastrpc_control(filp, arg);
	}
	case COMPAT_FASTRPC_IOCTL_GET_DSP_INFO:
	{
		return compat_fastrpc_get_dsp_info(filp, arg);
	}
	case COMPAT_FASTRPC_IOCTL_MEM_MAP:
		/* fall through */
	case COMPAT_FASTRPC_IOCTL_MEM_UNMAP:
		/* fall through */
	case COMPAT_FASTRPC_IOCTL_MMAP:
		/* fall through */
	case COMPAT_FASTRPC_IOCTL_MMAP_64:
		/* fall through */
	case COMPAT_FASTRPC_IOCTL_MUNMAP:
		return compat_fastrpc_mmap_device_ioctl(filp, cmd, arg);
	default:
		return -ENOTTY;
	}
}
