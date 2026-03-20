// SPDX-License-Identifier: GPL-2.0
/*
 * This file contains the procedures for the handling of select and poll
 *
 * Created for Linux based loosely upon Mathius Lattner's minix
 * patches by Peter MacDonald. Heavily edited by Linus.
 *
 *  4 February 1994
 *     COFF/ELF binary emulation. If the process has the STICKY_TIMEOUTS
 *     flag set in its personality we do *not* modify the given timeout
 *     parameter to reflect time remaining.
 *
 *  24 January 2000
 *     Changed sys_poll()/do_poll() to use PAGE_SIZE chunk-based allocation 
 *     of fds to overcome nfds < 16390 descriptors limit (Tigran Aivazian).
 */

#include <linux/compat.h>
#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/sched/rt.h>
#include <linux/syscalls.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/personality.h> /* for STICKY_TIMEOUTS */
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/fs.h>
#include <linux/rcupdate.h>
#include <linux/hrtimer.h>
#include <linux/freezer.h>
#include <net/busy_poll.h>
#include <linux/vmalloc.h>

#include <linux/uaccess.h>


/*
 * Estimate expected accuracy in ns from a timeval.
 * 估计时间精度的辅助函数，用于计算 select/poll 的超时"宽松值" (slack)
 *
 * After quite a bit of churning around, we've settled on
 * a simple thing of taking 0.1% of the timeout as the
 * slack, with a cap of 100 msec.
 * "nice" tasks get a 0.5% slack instead.
 *
 * Consider this comment an open invitation to come up with even
 * better solutions..
 */

#define MAX_SLACK	(100 * NSEC_PER_MSEC)

static long __estimate_accuracy(struct timespec64 *tv)
{
	long slack;
	int divfactor = 1000;

	if (tv->tv_sec < 0)
		return 0;

	if (task_nice(current) > 0)
		divfactor = divfactor / 5;

	if (tv->tv_sec > MAX_SLACK / (NSEC_PER_SEC/divfactor))
		return MAX_SLACK;

	slack = tv->tv_nsec / divfactor;
	slack += tv->tv_sec * (NSEC_PER_SEC/divfactor);

	if (slack > MAX_SLACK)
		return MAX_SLACK;

	return slack;
}

u64 select_estimate_accuracy(struct timespec64 *tv)
{
	u64 ret;
	struct timespec64 now;
	u64 slack = current->timer_slack_ns;

	if (slack == 0)
		return 0;

	ktime_get_ts64(&now);
	now = timespec64_sub(*tv, now);
	ret = __estimate_accuracy(&now);
	if (ret < slack)
		return slack;
	return ret;
}



struct poll_table_page {
	struct poll_table_page * next;
	struct poll_table_entry * entry;
	struct poll_table_entry entries[];
};

#define POLL_TABLE_FULL(table) \
	((unsigned long)((table)->entry+1) > PAGE_SIZE + (unsigned long)(table))

/*
 * Ok, Peter made a complicated, but straightforward multiple_wait() function.
 * I have rewritten this, taking some shortcuts: This code may not be easy to
 * follow, but it should be free of race-conditions, and it's practical. If you
 * understand what I'm doing here, then you understand how the linux
 * sleep/wakeup mechanism works.
 *
 * Two very simple procedures, poll_wait() and poll_freewait() make all the
 * work.  poll_wait() is an inline-function defined in <linux/poll.h>,
 * as all select/poll functions have to call it to add an entry to the
 * poll table.
 */
static void __pollwait(struct file *filp, wait_queue_head_t *wait_address,
		       poll_table *p);

void poll_initwait(struct poll_wqueues *pwq)
{
	init_poll_funcptr(&pwq->pt, __pollwait);
	pwq->polling_task = current;
	pwq->triggered = 0;
	pwq->error = 0;
	pwq->table = NULL;
	pwq->inline_index = 0;
}
EXPORT_SYMBOL(poll_initwait);

static void free_poll_entry(struct poll_table_entry *entry)
{
	remove_wait_queue(entry->wait_address, &entry->wait);
	fput(entry->filp);
}

void poll_freewait(struct poll_wqueues *pwq)
{
	struct poll_table_page * p = pwq->table;
	int i;
	for (i = 0; i < pwq->inline_index; i++)
		free_poll_entry(pwq->inline_entries + i);
	while (p) {
		struct poll_table_entry * entry;
		struct poll_table_page *old;

		entry = p->entry;
		do {
			entry--;
			free_poll_entry(entry);
		} while (entry > p->entries);
		old = p;
		p = p->next;
		free_page((unsigned long) old);
	}
}
EXPORT_SYMBOL(poll_freewait);

static struct poll_table_entry *poll_get_entry(struct poll_wqueues *p)
{
	struct poll_table_page *table = p->table;

	if (p->inline_index < N_INLINE_POLL_ENTRIES)
		return p->inline_entries + p->inline_index++;

	if (!table || POLL_TABLE_FULL(table)) {
		struct poll_table_page *new_table;

		new_table = (struct poll_table_page *) __get_free_page(GFP_KERNEL);
		if (!new_table) {
			p->error = -ENOMEM;
			return NULL;
		}
		new_table->entry = new_table->entries;
		new_table->next = table;
		p->table = new_table;
		table = new_table;
	}

	return table->entry++;
}

static int __pollwake(wait_queue_entry_t *wait, unsigned mode, int sync, void *key)
{
	struct poll_wqueues *pwq = wait->private;
	DECLARE_WAITQUEUE(dummy_wait, pwq->polling_task);

	/*
	 * Although this function is called under waitqueue lock, LOCK
	 * doesn't imply write barrier and the users expect write
	 * barrier semantics on wakeup functions.  The following
	 * smp_wmb() is equivalent to smp_wmb() in try_to_wake_up()
	 * and is paired with smp_store_mb() in poll_schedule_timeout.
	 */
	smp_wmb();
	pwq->triggered = 1;

	/*
	 * Perform the default wake up operation using a dummy
	 * waitqueue.
	 *
	 * TODO: This is hacky but there currently is no interface to
	 * pass in @sync.  @sync is scheduled to be removed and once
	 * that happens, wake_up_process() can be used directly.
	 */
	return default_wake_function(&dummy_wait, mode, sync, key);
}

static int pollwake(wait_queue_entry_t *wait, unsigned mode, int sync, void *key)
{
	struct poll_table_entry *entry;

	entry = container_of(wait, struct poll_table_entry, wait);
	if (key && !(key_to_poll(key) & entry->key))
		return 0;
	return __pollwake(wait, mode, sync, key);
}

/* Add a new entry */
static void __pollwait(struct file *filp, wait_queue_head_t *wait_address,
				poll_table *p)
{
	struct poll_wqueues *pwq = container_of(p, struct poll_wqueues, pt);
	struct poll_table_entry *entry = poll_get_entry(pwq);
	if (!entry)
		return;
	entry->filp = get_file(filp);
	entry->wait_address = wait_address;
	entry->key = p->_key;
	init_waitqueue_func_entry(&entry->wait, pollwake);
	entry->wait.private = pwq;
	add_wait_queue(wait_address, &entry->wait);
}

static int poll_schedule_timeout(struct poll_wqueues *pwq, int state,
			  ktime_t *expires, unsigned long slack)
{
	int rc = -EINTR;

	set_current_state(state);
	if (!pwq->triggered)
		rc = schedule_hrtimeout_range(expires, slack, HRTIMER_MODE_ABS);
	__set_current_state(TASK_RUNNING);

	/*
	 * Prepare for the next iteration.
	 *
	 * The following smp_store_mb() serves two purposes.  First, it's
	 * the counterpart rmb of the wmb in pollwake() such that data
	 * written before wake up is always visible after wake up.
	 * Second, the full barrier guarantees that triggered clearing
	 * doesn't pass event check of the next iteration.  Note that
	 * this problem doesn't exist for the first iteration as
	 * add_wait_queue() has full barrier semantics.
	 */
	smp_store_mb(pwq->triggered, 0);

	return rc;
}

/**
 * poll_select_set_timeout - helper function to setup the timeout value
 * @to:		pointer to timespec64 variable for the final timeout
 * @sec:	seconds (from user space)
 * @nsec:	nanoseconds (from user space)
 *
 * Note, we do not use a timespec for the user space value here, That
 * way we can use the function for timeval and compat interfaces as well.
 *
 * Returns -EINVAL if sec/nsec are not normalized. Otherwise 0.
 */
int poll_select_set_timeout(struct timespec64 *to, time64_t sec, long nsec)
{
	struct timespec64 ts = {.tv_sec = sec, .tv_nsec = nsec};

	if (!timespec64_valid(&ts))
		return -EINVAL;

	/* Optimize for the zero timeout value here */
	if (!sec && !nsec) {
		to->tv_sec = to->tv_nsec = 0;
	} else {
		ktime_get_ts64(to);
		*to = timespec64_add_safe(*to, ts);
	}
	return 0;
}

enum poll_time_type {
	PT_TIMEVAL = 0,
	PT_OLD_TIMEVAL = 1,
	PT_TIMESPEC = 2,
	PT_OLD_TIMESPEC = 3,
};

static int poll_select_finish(struct timespec64 *end_time,
			      void __user *p,
			      enum poll_time_type pt_type, int ret)
{
	struct timespec64 rts;

	restore_saved_sigmask_unless(ret == -ERESTARTNOHAND);

	if (!p)
		return ret;

	if (current->personality & STICKY_TIMEOUTS)
		goto sticky;

	/* No update for zero timeout */
	if (!end_time->tv_sec && !end_time->tv_nsec)
		return ret;

	ktime_get_ts64(&rts);
	rts = timespec64_sub(*end_time, rts);
	if (rts.tv_sec < 0)
		rts.tv_sec = rts.tv_nsec = 0;


	switch (pt_type) {
	case PT_TIMEVAL:
		{
			struct __kernel_old_timeval rtv;

			if (sizeof(rtv) > sizeof(rtv.tv_sec) + sizeof(rtv.tv_usec))
				memset(&rtv, 0, sizeof(rtv));
			rtv.tv_sec = rts.tv_sec;
			rtv.tv_usec = rts.tv_nsec / NSEC_PER_USEC;
			if (!copy_to_user(p, &rtv, sizeof(rtv)))
				return ret;
		}
		break;
	case PT_OLD_TIMEVAL:
		{
			struct old_timeval32 rtv;

			rtv.tv_sec = rts.tv_sec;
			rtv.tv_usec = rts.tv_nsec / NSEC_PER_USEC;
			if (!copy_to_user(p, &rtv, sizeof(rtv)))
				return ret;
		}
		break;
	case PT_TIMESPEC:
		if (!put_timespec64(&rts, p))
			return ret;
		break;
	case PT_OLD_TIMESPEC:
		if (!put_old_timespec32(&rts, p))
			return ret;
		break;
	default:
		BUG();
	}
	/*
	 * If an application puts its timeval in read-only memory, we
	 * don't want the Linux-specific update to the timeval to
	 * cause a fault after the select has completed
	 * successfully. However, because we're not updating the
	 * timeval, we can't restart the system call.
	 */

sticky:
	if (ret == -ERESTARTNOHAND)
		ret = -EINTR;
	return ret;
}

/*
 * Scalable version of the fd_set.
 */

typedef struct {
	unsigned long *in, *out, *ex;
	unsigned long *res_in, *res_out, *res_ex;
} fd_set_bits;

/*
 * How many longwords for "nr" bits?
 */
#define FDS_BITPERLONG	(8*sizeof(long))
#define FDS_LONGS(nr)	(((nr)+FDS_BITPERLONG-1)/FDS_BITPERLONG)
#define FDS_BYTES(nr)	(FDS_LONGS(nr)*sizeof(long))

/*
 * Use "unsigned long" accesses to let user-mode fd_set's be long-aligned.
 */
static inline
int get_fd_set(unsigned long nr, void __user *ufdset, unsigned long *fdset)
{
	nr = FDS_BYTES(nr);
	if (ufdset)
		return copy_from_user(fdset, ufdset, nr) ? -EFAULT : 0;

	memset(fdset, 0, nr);
	return 0;
}

static inline unsigned long __must_check
set_fd_set(unsigned long nr, void __user *ufdset, unsigned long *fdset)
{
	if (ufdset)
		return __copy_to_user(ufdset, fdset, FDS_BYTES(nr));
	return 0;
}

static inline
void zero_fd_set(unsigned long nr, unsigned long *fdset)
{
	memset(fdset, 0, FDS_BYTES(nr));
}

#define FDS_IN(fds, n)		(fds->in + n)
#define FDS_OUT(fds, n)		(fds->out + n)
#define FDS_EX(fds, n)		(fds->ex + n)

#define BITS(fds, n)	(*FDS_IN(fds, n)|*FDS_OUT(fds, n)|*FDS_EX(fds, n))

static int max_select_fd(unsigned long n, fd_set_bits *fds)
{
	unsigned long *open_fds;
	unsigned long set;
	int max;
	struct fdtable *fdt;

	/* handle last in-complete long-word first */
	set = ~(~0UL << (n & (BITS_PER_LONG-1)));
	n /= BITS_PER_LONG;
	fdt = files_fdtable(current->files);
	open_fds = fdt->open_fds + n;
	max = 0;
	if (set) {
		set &= BITS(fds, n);
		if (set) {
			if (!(set & ~*open_fds))
				goto get_max;
			return -EBADF;
		}
	}
	while (n) {
		open_fds--;
		n--;
		set = BITS(fds, n);
		if (!set)
			continue;
		if (set & ~*open_fds)
			return -EBADF;
		if (max)
			continue;
get_max:
		do {
			max++;
			set >>= 1;
		} while (set);
		max += n * BITS_PER_LONG;
	}

	return max;
}

#define POLLIN_SET (EPOLLRDNORM | EPOLLRDBAND | EPOLLIN | EPOLLHUP | EPOLLERR |\
			EPOLLNVAL)
#define POLLOUT_SET (EPOLLWRBAND | EPOLLWRNORM | EPOLLOUT | EPOLLERR |\
			 EPOLLNVAL)
#define POLLEX_SET (EPOLLPRI | EPOLLNVAL)

static inline void wait_key_set(poll_table *wait, unsigned long in,
				unsigned long out, unsigned long bit,
				__poll_t ll_flag)
{
	wait->_key = POLLEX_SET | ll_flag;
	if (in & bit)
		wait->_key |= POLLIN_SET;
	if (out & bit)
		wait->_key |= POLLOUT_SET;
}

static noinline_for_stack int do_select(int n, fd_set_bits *fds, struct timespec64 *end_time)
{
	/*
	 * do_select(): select/poll 的核心实现函数
	 *
	 * 功能说明：
	 *   这个函数是 select() 和 poll() 系统调用的核心执行逻辑。
	 *   它的作用是：监控多个文件描述符（fd），直到其中某个（或多个）变为"就绪"状态后才返回。
	 *
	 * 什么是"就绪"：
	 *   - POLLIN: 有数据可读（输入缓冲区有数据）
	 *   - POLLOUT: 可以写入数据（输出缓冲区有空间）
	 *   - POLLPRI: 有紧急数据可读
	 *
	 * 调用场景：
	 *   - 当程序需要同时等待多个 I/O 事件时（如同时读取多个网络连接或文件）
	 *   - 例如：视频流处理中等待摄像头数据、网络服务器等待客户端请求
	 *
	 * @n: 最大文件描述符编号 + 1（即最大 fd 值加 1）
	 * @fds: 包含感兴趣的文件描述符集合（输入）和就绪结果（输出）
	 *       - fds->in:  关心读事件的 fd 集合（输入）
	 *       - fds->out: 关心写事件的 fd 集合（输入）
	 *       - fds->ex:  关心异常事件的 fd 集合（输入）
	 *       - fds->res_in/out/ex: 就绪结果（输出）
	 * @end_time: 超时截止时间，如果为 NULL 则无限等待
	 *
	 * 返回值：
	 *   - > 0: 就绪的文件描述符数量
	 *   - 0:   超时，没有 fd 变为就绪
	 *   - < 0: 出错（如被信号中断）
	 *
	 * 工作流程：
	 *   1. 获取有效的最大 fd 编号
	 *   2. 初始化 poll_wqueues 结构（用于管理等待队列）
	 *   3. 循环遍历所有 fd，调用 vfs_poll() 检查每个 fd 的状态
	 *   4. 如果没有就绪的 fd，则调用 poll_schedule_timeout() 进入睡眠等待
	 *   5. 直到有 fd 就绪、超时、或收到信号才返回
	 *   6. 返回就绪的文件描述符数量
	 *
	 * 性能分析关键点：
	 *   - vfs_poll() 是实际调用底层驱动 poll 函数的地方
	 *   - poll_schedule_timeout() 是让进程进入睡眠的关键函数
	 *   这两个函数是分析 I/O 延迟的核心位置
	 */

	/* ========== 变量定义部分 ========== */
	ktime_t expire, *to = NULL;  /* expire: 超时到期时间, to: 指向超时的指针 */
	struct poll_wqueues table;   /* poll_wqueues: 管理等待队列的结构 */
	poll_table *wait;             /* wait: poll 表指针，用于文件就绪回调 */
	int retval, i, timed_out = 0; /* retval: 返回值, i: 循环计数, timed_out: 超时标志 */
	u64 slack = 0;                /* slack: 调度精度估算值 */
	__poll_t busy_flag = net_busy_loop_on() ? POLL_BUSY_LOOP : 0; /* busy_flag: 忙碌循环标志 */
	unsigned long busy_start = 0; /* busy_start: 忙碌循环开始时间 */
	ktime_t pselect_start_time = ktime_get(); /* 记录函数开始时间，用于性能跟踪 */
	ktime_t sleep_start_time = 0; /* 记录进入睡眠的开始时间 */
	ktime_t sleep_end_time = 0;   /* 记录从睡眠中唤醒的结束时间 */
	ktime_t sleep_duration = 0;   /* 实际睡眠持续时间 */

	/* ========== 第一步：获取有效的最大 fd 编号 ========== */
	rcu_read_lock();
	retval = max_select_fd(n, fds);  /* max_select_fd: 获取有效的最大 fd */
	rcu_read_unlock();

	if (retval < 0)
		return retval;
	n = retval;

	/* ========== 第二步：初始化 poll_wqueues 结构 ========== */
	poll_initwait(&table);  /* 初始化等待队列 */
	wait = &table.pt;

	/* ========== 第三步：处理零超时情况 ========== */
	if (end_time && !end_time->tv_sec && !end_time->tv_nsec) {
		wait->_qproc = NULL;  /* 禁用回调，直接轮询 */
		timed_out = 1;        /* 标记为立即超时 */
	}

	/* ========== 第四步：估算超时调度精度 ========== */
	if (end_time && !timed_out)
		slack = select_estimate_accuracy(end_time);  /* 估算合适的调度精度 */

	/* ========== 第五步：主循环开始 ========== */
	retval = 0;  /* 初始化返回值为 0 */
	for (;;) {
		unsigned long *rinp, *routp, *rexp, *inp, *outp, *exp;
		bool can_busy_loop = false;

		/* 指向 fd 集合的指针：in(输入)、out(输出)、ex(异常) */
		inp = fds->in; outp = fds->out; exp = fds->ex;
		/* 指向结果集合的指针 */
		rinp = fds->res_in; routp = fds->res_out; rexp = fds->res_ex;

		/* ========== 第六步：外层循环 - 遍历所有 fd 位图块 ========== */
		for (i = 0; i < n; ++rinp, ++routp, ++rexp) {
			unsigned long in, out, ex, all_bits, bit = 1, j;
			unsigned long res_in = 0, res_out = 0, res_ex = 0;
			__poll_t mask;

			/* 读取当前位图块 */
			in = *inp++; out = *outp++; ex = *exp++;
			all_bits = in | out | ex;  /* 合并所有感兴趣的位 */
			if (all_bits == 0) {
				i += BITS_PER_LONG;  /* 如果没有感兴趣的位，跳过整个块 */
				continue;
			}

			/* ========== 第七步：内层循环 - 遍历位图块中的每个 fd ========== */
			for (j = 0; j < BITS_PER_LONG; ++j, ++i, bit <<= 1) {
				struct fd f;
				if (i >= n)
					break;
				if (!(bit & all_bits))  /* 如果该位不在感兴趣集合中，跳过 */
					continue;

				/* 初始化 mask 为无效标志 */
				mask = EPOLLNVAL;
				/* 获取 fd 对应的文件结构 */
				f = fdget(i);
				if (fd_file(f)) {
					/* 设置等待键值，用于回调时判断哪些 fd 就绪 */
					wait_key_set(wait, in, out, bit, busy_flag);
					/* 调用 vfs_poll 检查文件状态，这是实际轮询的地方 */
					mask = vfs_poll(fd_file(f), wait);
					/* 释放 fd 引用 */
					fdput(f);
				}

				/* ========== 第八步：检查 POLLIN (输入就绪) ========== */
				if ((mask & POLLIN_SET) && (in & bit)) {
					res_in |= bit;   /* 标记该 fd 在输入集合中就绪 */
					retval++;         /* 增加就绪计数 */
					wait->_qproc = NULL;  /* 有 fd 就绪后禁用回调 */
				}
				/* ========== 第九步：检查 POLLOUT (输出就绪) ========== */
				if ((mask & POLLOUT_SET) && (out & bit)) {
					res_out |= bit;
					retval++;
					wait->_qproc = NULL;
				}
				/* ========== 第十步：检查 POLLEX (异常条件) ========== */
				if ((mask & POLLEX_SET) && (ex & bit)) {
					res_ex |= bit;
					retval++;
					wait->_qproc = NULL;
				}

				/* 第十一步：如果有 fd 就绪，停止忙碌轮询 */
				if (retval) {
					can_busy_loop = false;
					busy_flag = 0;
				} else if (busy_flag & mask)  /* 记住返回 POLL_BUSY_LOOP 的 socket */
					can_busy_loop = true;
			}

			/* ========== 第十二步：保存结果到位图 ========== */
			if (res_in)
				*rinp = res_in;
			if (res_out)
				*routp = res_out;
			if (res_ex)
				*rexp = res_ex;

			cond_resched();  /* 让出 CPU，允许其他进程运行 */
		}

		/* 第十三步：检查是否需要退出循环 */
		wait->_qproc = NULL;  /* 重置回调 */
		if (retval || timed_out || signal_pending(current))  /* 有 fd 就绪 或 超时 或 有信号 */
			break;
		if (table.error) {    /* 如果有错误 */
			retval = table.error;
			break;
		}

		/* 第十四步：处理忙碌循环模式 */
		if (can_busy_loop && !need_resched()) {
			if (!busy_start) {
				busy_start = busy_loop_current_time();
				continue;
			}
			if (!busy_loop_timeout(busy_start))
				continue;
		}
		busy_flag = 0;

		/* ========== 第十五步：设置超时时间 ========== */
		if (end_time && !to) {
			/* 首次循环时，将 timespec64 转换为 ktime_t */
			expire = timespec64_to_ktime(*end_time);
			to = &expire;
		}

		/* ========== 第十六步：进入睡眠等待 ========== */
		/* 记录进入睡眠前的时间 */
		sleep_start_time = ktime_get();
		/* 调用 poll_schedule_timeout 进入睡眠，直到有 fd 就绪、超时或被信号唤醒 */
		if (!poll_schedule_timeout(&table, TASK_INTERRUPTIBLE, to, slack))
			timed_out = 1;  /* 如果返回 false，说明超时了 */
		/* 记录从睡眠中唤醒后的时间 */
		sleep_end_time = ktime_get();
		/* 计算实际睡眠持续时间 */
		sleep_duration = ktime_sub(sleep_end_time, sleep_start_time);
	}

	/* ========== 第十七步：清理资源并返回 ========== */
	poll_freewait(&table);  /* 释放等待队列资源 */

	/* 记录函数结束时间和耗时 */
	ktime_t pselect_end_time = ktime_get();
	ktime_t pselect_duration = ktime_sub(pselect_end_time, pselect_start_time);

	/* 计算上下文切换开销时间 */
	ktime_t context_switch_time = ktime_sub(pselect_duration, sleep_duration);

	/* 获取第一个被监控的 fd 对应的文件名 */
	char fname[64] = {0};
	unsigned int fd_idx = 0;
	struct fd f;
	for (fd_idx = 0; fd_idx < n; fd_idx++) {
		if (fds->in && (fds->in[fd_idx / BITS_PER_LONG] & (1UL << (fd_idx % BITS_PER_LONG)))) {
			f = fdget(fd_idx);
			if (!fd_empty(f)) {
				struct file *file = fd_file(f);
				if (file && file->f_path.dentry) {
					char *name = (char *)file->f_path.dentry->d_name.name;
					strncpy(fname, name, sizeof(fname) - 1);
				}
			}
			fdput(f);
			break;
		}
	}

	/* 计算带有小数部分的时间戳 */
	u64 total_ns = ktime_to_ns(pselect_end_time);
	time64_t sec = total_ns / 1000000000;
	u32 nsec = total_ns % 1000000000;

	if (fname[0])
		trace_printk("pselect6: total=%lld ns, sleep=%lld ns, context=%lld ns, ret=%d, end_time=%lld.%09u s, fd=%u, file=%s\n",
			     ktime_to_ns(pselect_duration), ktime_to_ns(sleep_duration), ktime_to_ns(context_switch_time),
			     retval, sec, nsec, fd_idx, fname);
	else
		trace_printk("pselect6: total=%lld ns, sleep=%lld ns, context=%lld ns, ret=%d, end_time=%lld.%09u s\n",
			     ktime_to_ns(pselect_duration), ktime_to_ns(sleep_duration), ktime_to_ns(context_switch_time),
			     retval, sec, nsec);

	return retval;  /* 返回就绪的 fd 数量 */
}

/*
 * We can actually return ERESTARTSYS instead of EINTR, but I'd
 * like to be certain this leads to no problems. So I return
 * EINTR just for safety.
 *
 * Update: ERESTARTSYS breaks at least the xview clock binary, so
 * I'm trying ERESTARTNOHAND which restart only when you want to.
 */
int core_sys_select(int n, fd_set __user *inp, fd_set __user *outp,
			   fd_set __user *exp, struct timespec64 *end_time)
{
	fd_set_bits fds;
	void *bits;
	int ret, max_fds;
	size_t size, alloc_size;
	struct fdtable *fdt;
	/* Allocate small arguments on the stack to save memory and be faster */
	long stack_fds[SELECT_STACK_ALLOC/sizeof(long)];

	ret = -EINVAL;
	if (n < 0)
		goto out_nofds;

	/* max_fds can increase, so grab it once to avoid race */
	rcu_read_lock();
	fdt = files_fdtable(current->files);
	max_fds = fdt->max_fds;
	rcu_read_unlock();
	if (n > max_fds)
		n = max_fds;

	/*
	 * We need 6 bitmaps (in/out/ex for both incoming and outgoing),
	 * since we used fdset we need to allocate memory in units of
	 * long-words. 
	 */
	size = FDS_BYTES(n);
	bits = stack_fds;
	if (size > sizeof(stack_fds) / 6) {
		/* Not enough space in on-stack array; must use kmalloc */
		ret = -ENOMEM;
		if (size > (SIZE_MAX / 6))
			goto out_nofds;

		alloc_size = 6 * size;
		bits = kvmalloc(alloc_size, GFP_KERNEL);
		if (!bits)
			goto out_nofds;
	}
	fds.in      = bits;
	fds.out     = bits +   size;
	fds.ex      = bits + 2*size;
	fds.res_in  = bits + 3*size;
	fds.res_out = bits + 4*size;
	fds.res_ex  = bits + 5*size;

	if ((ret = get_fd_set(n, inp, fds.in)) ||
	    (ret = get_fd_set(n, outp, fds.out)) ||
	    (ret = get_fd_set(n, exp, fds.ex)))
		goto out;
	zero_fd_set(n, fds.res_in);
	zero_fd_set(n, fds.res_out);
	zero_fd_set(n, fds.res_ex);

	ret = do_select(n, &fds, end_time);

	if (ret < 0)
		goto out;
	if (!ret) {
		ret = -ERESTARTNOHAND;
		if (signal_pending(current))
			goto out;
		ret = 0;
	}

	if (set_fd_set(n, inp, fds.res_in) ||
	    set_fd_set(n, outp, fds.res_out) ||
	    set_fd_set(n, exp, fds.res_ex))
		ret = -EFAULT;

out:
	if (bits != stack_fds)
		kvfree(bits);
out_nofds:
	return ret;
}

static int kern_select(int n, fd_set __user *inp, fd_set __user *outp,
		       fd_set __user *exp, struct __kernel_old_timeval __user *tvp)
{
	struct timespec64 end_time, *to = NULL;
	struct __kernel_old_timeval tv;
	int ret;

	if (tvp) {
		if (copy_from_user(&tv, tvp, sizeof(tv)))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to,
				tv.tv_sec + (tv.tv_usec / USEC_PER_SEC),
				(tv.tv_usec % USEC_PER_SEC) * NSEC_PER_USEC))
			return -EINVAL;
	}

	ret = core_sys_select(n, inp, outp, exp, to);
	return poll_select_finish(&end_time, tvp, PT_TIMEVAL, ret);
}

SYSCALL_DEFINE5(select, int, n, fd_set __user *, inp, fd_set __user *, outp,
		fd_set __user *, exp, struct __kernel_old_timeval __user *, tvp)
{
	return kern_select(n, inp, outp, exp, tvp);
}

static long do_pselect(int n, fd_set __user *inp, fd_set __user *outp,
		       fd_set __user *exp, void __user *tsp,
		       const sigset_t __user *sigmask, size_t sigsetsize,
		       enum poll_time_type type)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		switch (type) {
		case PT_TIMESPEC:
			if (get_timespec64(&ts, tsp))
				return -EFAULT;
			break;
		case PT_OLD_TIMESPEC:
			if (get_old_timespec32(&ts, tsp))
				return -EFAULT;
			break;
		default:
			BUG();
		}

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = core_sys_select(n, inp, outp, exp, to);
	return poll_select_finish(&end_time, tsp, type, ret);
}

/*
 * Most architectures can't handle 7-argument syscalls. So we provide a
 * 6-argument version where the sixth argument is a pointer to a structure
 * which has a pointer to the sigset_t itself followed by a size_t containing
 * the sigset size.
 */
struct sigset_argpack {
	sigset_t __user *p;
	size_t size;
};

static inline int get_sigset_argpack(struct sigset_argpack *to,
				     struct sigset_argpack __user *from)
{
	// the path is hot enough for overhead of copy_from_user() to matter
	if (from) {
		if (can_do_masked_user_access())
			from = masked_user_access_begin(from);
		else if (!user_read_access_begin(from, sizeof(*from)))
			return -EFAULT;
		unsafe_get_user(to->p, &from->p, Efault);
		unsafe_get_user(to->size, &from->size, Efault);
		user_read_access_end();
	}
	return 0;
Efault:
	user_read_access_end();
	return -EFAULT;
}
/* 
 * pselect6() 系统调用入口
 * 
 * pselect6 系统调用是 POSIX select 的超集，提供以下特性：
 * 1. 支持纳秒级精度的 timespec 超时（而非 select 的 timeval 微秒级）
 * 2. 支持第6个参数同时传递信号掩码和掩码大小，实现原子操作
 * 
 * 参数:
 *   n      - 最大文件描述符编号 + 1
 *   inp    - 读取文件描述符集合 (fd_set *)
 *   outp   - 写入文件描述符集合
 *   exp    - 异常条件文件描述符集合
 *   tsp    - 超时时间 (struct __kernel_timespec *)
 *   sig    - 信号掩码参数 (包含 sigset_t 指针和 size_t 大小)
 * 
 * 返回值: 已就绪的文件描述符数量，0 表示超时，-1 表示错误
 */
SYSCALL_DEFINE6(pselect6, int, n, fd_set __user *, inp, fd_set __user *, outp,
		fd_set __user *, exp, struct __kernel_timespec __user *, tsp,
		void __user *, sig)
{
	struct sigset_argpack x = {NULL, 0};

	if (get_sigset_argpack(&x, sig))
		return -EFAULT;

	return do_pselect(n, inp, outp, exp, tsp, x.p, x.size, PT_TIMESPEC);
}

#if defined(CONFIG_COMPAT_32BIT_TIME) && !defined(CONFIG_64BIT)

SYSCALL_DEFINE6(pselect6_time32, int, n, fd_set __user *, inp, fd_set __user *, outp,
		fd_set __user *, exp, struct old_timespec32 __user *, tsp,
		void __user *, sig)
{
	struct sigset_argpack x = {NULL, 0};

	if (get_sigset_argpack(&x, sig))
		return -EFAULT;

	return do_pselect(n, inp, outp, exp, tsp, x.p, x.size, PT_OLD_TIMESPEC);
}

#endif

#ifdef __ARCH_WANT_SYS_OLD_SELECT
struct sel_arg_struct {
	unsigned long n;
	fd_set __user *inp, *outp, *exp;
	struct __kernel_old_timeval __user *tvp;
};

SYSCALL_DEFINE1(old_select, struct sel_arg_struct __user *, arg)
{
	struct sel_arg_struct a;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	return kern_select(a.n, a.inp, a.outp, a.exp, a.tvp);
}
#endif

struct poll_list {
	struct poll_list *next;
	unsigned int len;
	struct pollfd entries[] __counted_by(len);
};

#define POLLFD_PER_PAGE  ((PAGE_SIZE-sizeof(struct poll_list)) / sizeof(struct pollfd))

/*
 * Fish for pollable events on the pollfd->fd file descriptor. We're only
 * interested in events matching the pollfd->events mask, and the result
 * matching that mask is both recorded in pollfd->revents and returned. The
 * pwait poll_table will be used by the fd-provided poll handler for waiting,
 * if pwait->_qproc is non-NULL.
 */
static inline __poll_t do_pollfd(struct pollfd *pollfd, poll_table *pwait,
				     bool *can_busy_poll,
				     __poll_t busy_flag)
{
	int fd = pollfd->fd;
	__poll_t mask = 0, filter;
	struct fd f;

	if (fd < 0)
		goto out;
	mask = EPOLLNVAL;
	f = fdget(fd);
	if (!fd_file(f))
		goto out;

	/* userland u16 ->events contains POLL... bitmap */
	filter = demangle_poll(pollfd->events) | EPOLLERR | EPOLLHUP;
	pwait->_key = filter | busy_flag;
	mask = vfs_poll(fd_file(f), pwait);
	if (mask & busy_flag)
		*can_busy_poll = true;
	mask &= filter;		/* Mask out unneeded events. */
	fdput(f);

out:
	/* ... and so does ->revents */
	pollfd->revents = mangle_poll(mask);
	return mask;
}

static int do_poll(struct poll_list *list, struct poll_wqueues *wait,
		   struct timespec64 *end_time)
/*
 * do_poll(): poll() 系统调用的核心实现
 * @list: pollfd 链表，包含用户传递的所有文件描述符和感兴趣的事件
 * @wait: poll_wqueues 结构，用于管理等待队列
 * @end_time: 超时截止时间
 *
 * 与 do_select() 的区别：
 * - do_select(): 使用 fd_set 位图（select 系统调用使用）
 * - do_poll(): 使用 pollfd 数组（poll 系统调用使用）
 *
 * 工作流程与 do_select() 类似，但处理的是 pollfd 结构而非 fd_set
 */
{
	poll_table* pt = &wait->pt;
	ktime_t expire, *to = NULL;
	int timed_out = 0, count = 0;
	u64 slack = 0;
	__poll_t busy_flag = net_busy_loop_on() ? POLL_BUSY_LOOP : 0;
	unsigned long busy_start = 0;

	/* Optimise the no-wait case */
	if (end_time && !end_time->tv_sec && !end_time->tv_nsec) {
		pt->_qproc = NULL;
		timed_out = 1;
	}

	if (end_time && !timed_out)
		slack = select_estimate_accuracy(end_time);

	for (;;) {
		struct poll_list *walk;
		bool can_busy_loop = false;

		for (walk = list; walk != NULL; walk = walk->next) {
			struct pollfd * pfd, * pfd_end;

			pfd = walk->entries;
			pfd_end = pfd + walk->len;
			for (; pfd != pfd_end; pfd++) {
				/*
				 * Fish for events. If we found one, record it
				 * and kill poll_table->_qproc, so we don't
				 * needlessly register any other waiters after
				 * this. They'll get immediately deregistered
				 * when we break out and return.
				 */
				if (do_pollfd(pfd, pt, &can_busy_loop,
					      busy_flag)) {
					count++;
					pt->_qproc = NULL;
					/* found something, stop busy polling */
					busy_flag = 0;
					can_busy_loop = false;
				}
			}
		}
		/*
		 * All waiters have already been registered, so don't provide
		 * a poll_table->_qproc to them on the next loop iteration.
		 */
		pt->_qproc = NULL;
		if (!count) {
			count = wait->error;
			if (signal_pending(current))
				count = -ERESTARTNOHAND;
		}
		if (count || timed_out)
			break;

		/* only if found POLL_BUSY_LOOP sockets && not out of time */
		if (can_busy_loop && !need_resched()) {
			if (!busy_start) {
				busy_start = busy_loop_current_time();
				continue;
			}
			if (!busy_loop_timeout(busy_start))
				continue;
		}
		busy_flag = 0;

		/*
		 * If this is the first loop and we have a timeout
		 * given, then we convert to ktime_t and set the to
		 * pointer to the expiry value.
		 */
		if (end_time && !to) {
			expire = timespec64_to_ktime(*end_time);
			to = &expire;
		}

		if (!poll_schedule_timeout(wait, TASK_INTERRUPTIBLE, to, slack))
			timed_out = 1;
	}
	return count;
}

#define N_STACK_PPS ((sizeof(stack_pps) - sizeof(struct poll_list))  / \
			sizeof(struct pollfd))

static int do_sys_poll(struct pollfd __user *ufds, unsigned int nfds,
		struct timespec64 *end_time)
{
	struct poll_wqueues table;
	int err = -EFAULT, fdcount;
	/* Allocate small arguments on the stack to save memory and be
	   faster - use long to make sure the buffer is aligned properly
	   on 64 bit archs to avoid unaligned access */
	long stack_pps[POLL_STACK_ALLOC/sizeof(long)];
	struct poll_list *const head = (struct poll_list *)stack_pps;
 	struct poll_list *walk = head;
	unsigned int todo = nfds;
	unsigned int len;

	if (nfds > rlimit(RLIMIT_NOFILE))
		return -EINVAL;

	len = min_t(unsigned int, nfds, N_STACK_PPS);
	for (;;) {
		walk->next = NULL;
		walk->len = len;
		if (!len)
			break;

		if (copy_from_user(walk->entries, ufds + nfds-todo,
					sizeof(struct pollfd) * walk->len))
			goto out_fds;

		if (walk->len >= todo)
			break;
		todo -= walk->len;

		len = min(todo, POLLFD_PER_PAGE);
		walk = walk->next = kmalloc(struct_size(walk, entries, len),
					    GFP_KERNEL);
		if (!walk) {
			err = -ENOMEM;
			goto out_fds;
		}
	}

	poll_initwait(&table);
	fdcount = do_poll(head, &table, end_time);
	poll_freewait(&table);

	if (!user_write_access_begin(ufds, nfds * sizeof(*ufds)))
		goto out_fds;

	for (walk = head; walk; walk = walk->next) {
		struct pollfd *fds = walk->entries;
		unsigned int j;

		for (j = walk->len; j; fds++, ufds++, j--)
			unsafe_put_user(fds->revents, &ufds->revents, Efault);
  	}
	user_write_access_end();

	err = fdcount;
out_fds:
	walk = head->next;
	while (walk) {
		struct poll_list *pos = walk;
		walk = walk->next;
		kfree(pos);
	}

	return err;

Efault:
	user_write_access_end();
	err = -EFAULT;
	goto out_fds;
}

static long do_restart_poll(struct restart_block *restart_block)
{
	struct pollfd __user *ufds = restart_block->poll.ufds;
	int nfds = restart_block->poll.nfds;
	struct timespec64 *to = NULL, end_time;
	int ret;

	if (restart_block->poll.has_timeout) {
		end_time.tv_sec = restart_block->poll.tv_sec;
		end_time.tv_nsec = restart_block->poll.tv_nsec;
		to = &end_time;
	}

	ret = do_sys_poll(ufds, nfds, to);

	if (ret == -ERESTARTNOHAND)
		ret = set_restart_fn(restart_block, do_restart_poll);

	return ret;
}

SYSCALL_DEFINE3(poll, struct pollfd __user *, ufds, unsigned int, nfds,
		int, timeout_msecs)
{
	struct timespec64 end_time, *to = NULL;
	int ret;

	if (timeout_msecs >= 0) {
		to = &end_time;
		poll_select_set_timeout(to, timeout_msecs / MSEC_PER_SEC,
			NSEC_PER_MSEC * (timeout_msecs % MSEC_PER_SEC));
	}

	ret = do_sys_poll(ufds, nfds, to);

	if (ret == -ERESTARTNOHAND) {
		struct restart_block *restart_block;

		restart_block = &current->restart_block;
		restart_block->poll.ufds = ufds;
		restart_block->poll.nfds = nfds;

		if (timeout_msecs >= 0) {
			restart_block->poll.tv_sec = end_time.tv_sec;
			restart_block->poll.tv_nsec = end_time.tv_nsec;
			restart_block->poll.has_timeout = 1;
		} else
			restart_block->poll.has_timeout = 0;

		ret = set_restart_fn(restart_block, do_restart_poll);
	}
	return ret;
}

SYSCALL_DEFINE5(ppoll, struct pollfd __user *, ufds, unsigned int, nfds,
		struct __kernel_timespec __user *, tsp, const sigset_t __user *, sigmask,
		size_t, sigsetsize)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		if (get_timespec64(&ts, tsp))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = do_sys_poll(ufds, nfds, to);
	return poll_select_finish(&end_time, tsp, PT_TIMESPEC, ret);
}

#if defined(CONFIG_COMPAT_32BIT_TIME) && !defined(CONFIG_64BIT)

SYSCALL_DEFINE5(ppoll_time32, struct pollfd __user *, ufds, unsigned int, nfds,
		struct old_timespec32 __user *, tsp, const sigset_t __user *, sigmask,
		size_t, sigsetsize)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		if (get_old_timespec32(&ts, tsp))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = do_sys_poll(ufds, nfds, to);
	return poll_select_finish(&end_time, tsp, PT_OLD_TIMESPEC, ret);
}
#endif

#ifdef CONFIG_COMPAT
#define __COMPAT_NFDBITS       (8 * sizeof(compat_ulong_t))

/*
 * Ooo, nasty.  We need here to frob 32-bit unsigned longs to
 * 64-bit unsigned longs.
 */
static
int compat_get_fd_set(unsigned long nr, compat_ulong_t __user *ufdset,
			unsigned long *fdset)
{
	if (ufdset) {
		return compat_get_bitmap(fdset, ufdset, nr);
	} else {
		zero_fd_set(nr, fdset);
		return 0;
	}
}

static
int compat_set_fd_set(unsigned long nr, compat_ulong_t __user *ufdset,
		      unsigned long *fdset)
{
	if (!ufdset)
		return 0;
	return compat_put_bitmap(ufdset, fdset, nr);
}


/*
 * This is a virtual copy of sys_select from fs/select.c and probably
 * should be compared to it from time to time
 */

/*
 * pselect6 系统调用的内核实现
 * 
 * 本文件实现了 Linux 的 select/poll 机制，是 I/O 多路复用的核心组件。
 * pselect6 的核心功能是：
 *   - 监视多个文件描述符（fd），等待其变为可读/可写/异常
 *   - 支持 nanosecond 精度的超时
 *   - 支持原子地设置信号掩码
 * 
 * 调用层次（从外到内）：
 *   1. SYSCALL_DEFINE6(pselect6)    - 系统调用入口
 *   2. do_pselect()                 - 核心处理函数
 *   3. core_sys_select()            - 内存分配和数据准备
 *   4. do_select()                  - 实际轮询/等待
 *   5. vfs_poll()                   - VFS 层调用具体文件系统的 poll
 */
static int compat_core_sys_select(int n, compat_ulong_t __user *inp,
	compat_ulong_t __user *outp, compat_ulong_t __user *exp,
	struct timespec64 *end_time)
{
	fd_set_bits fds;
	void *bits;
	int size, max_fds, ret = -EINVAL;
	struct fdtable *fdt;
	long stack_fds[SELECT_STACK_ALLOC/sizeof(long)];

	if (n < 0)
		goto out_nofds;

	/* max_fds can increase, so grab it once to avoid race */
	rcu_read_lock();
	fdt = files_fdtable(current->files);
	max_fds = fdt->max_fds;
	rcu_read_unlock();
	if (n > max_fds)
		n = max_fds;

	/*
	 * We need 6 bitmaps (in/out/ex for both incoming and outgoing),
	 * since we used fdset we need to allocate memory in units of
	 * long-words.
	 */
	size = FDS_BYTES(n);
	bits = stack_fds;
	if (size > sizeof(stack_fds) / 6) {
		bits = kmalloc_array(6, size, GFP_KERNEL);
		ret = -ENOMEM;
		if (!bits)
			goto out_nofds;
	}
	fds.in      = (unsigned long *)  bits;
	fds.out     = (unsigned long *) (bits +   size);
	fds.ex      = (unsigned long *) (bits + 2*size);
	fds.res_in  = (unsigned long *) (bits + 3*size);
	fds.res_out = (unsigned long *) (bits + 4*size);
	fds.res_ex  = (unsigned long *) (bits + 5*size);

	if ((ret = compat_get_fd_set(n, inp, fds.in)) ||
	    (ret = compat_get_fd_set(n, outp, fds.out)) ||
	    (ret = compat_get_fd_set(n, exp, fds.ex)))
		goto out;
	zero_fd_set(n, fds.res_in);
	zero_fd_set(n, fds.res_out);
	zero_fd_set(n, fds.res_ex);

	ret = do_select(n, &fds, end_time);

	if (ret < 0)
		goto out;
	if (!ret) {
		ret = -ERESTARTNOHAND;
		if (signal_pending(current))
			goto out;
		ret = 0;
	}

	if (compat_set_fd_set(n, inp, fds.res_in) ||
	    compat_set_fd_set(n, outp, fds.res_out) ||
	    compat_set_fd_set(n, exp, fds.res_ex))
		ret = -EFAULT;
out:
	if (bits != stack_fds)
		kfree(bits);
out_nofds:
	return ret;
}

static int do_compat_select(int n, compat_ulong_t __user *inp,
	compat_ulong_t __user *outp, compat_ulong_t __user *exp,
	struct old_timeval32 __user *tvp)
{
	struct timespec64 end_time, *to = NULL;
	struct old_timeval32 tv;
	int ret;

	if (tvp) {
		if (copy_from_user(&tv, tvp, sizeof(tv)))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to,
				tv.tv_sec + (tv.tv_usec / USEC_PER_SEC),
				(tv.tv_usec % USEC_PER_SEC) * NSEC_PER_USEC))
			return -EINVAL;
	}

	ret = compat_core_sys_select(n, inp, outp, exp, to);
	return poll_select_finish(&end_time, tvp, PT_OLD_TIMEVAL, ret);
}

COMPAT_SYSCALL_DEFINE5(select, int, n, compat_ulong_t __user *, inp,
	compat_ulong_t __user *, outp, compat_ulong_t __user *, exp,
	struct old_timeval32 __user *, tvp)
{
	return do_compat_select(n, inp, outp, exp, tvp);
}

struct compat_sel_arg_struct {
	compat_ulong_t n;
	compat_uptr_t inp;
	compat_uptr_t outp;
	compat_uptr_t exp;
	compat_uptr_t tvp;
};

COMPAT_SYSCALL_DEFINE1(old_select, struct compat_sel_arg_struct __user *, arg)
{
	struct compat_sel_arg_struct a;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	return do_compat_select(a.n, compat_ptr(a.inp), compat_ptr(a.outp),
				compat_ptr(a.exp), compat_ptr(a.tvp));
}

static long do_compat_pselect(int n, compat_ulong_t __user *inp,
	compat_ulong_t __user *outp, compat_ulong_t __user *exp,
	void __user *tsp, compat_sigset_t __user *sigmask,
	compat_size_t sigsetsize, enum poll_time_type type)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		switch (type) {
		case PT_OLD_TIMESPEC:
			if (get_old_timespec32(&ts, tsp))
				return -EFAULT;
			break;
		case PT_TIMESPEC:
			if (get_timespec64(&ts, tsp))
				return -EFAULT;
			break;
		default:
			BUG();
		}

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_compat_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = compat_core_sys_select(n, inp, outp, exp, to);
	return poll_select_finish(&end_time, tsp, type, ret);
}

struct compat_sigset_argpack {
	compat_uptr_t p;
	compat_size_t size;
};
static inline int get_compat_sigset_argpack(struct compat_sigset_argpack *to,
					    struct compat_sigset_argpack __user *from)
{
	if (from) {
		if (!user_read_access_begin(from, sizeof(*from)))
			return -EFAULT;
		unsafe_get_user(to->p, &from->p, Efault);
		unsafe_get_user(to->size, &from->size, Efault);
		user_read_access_end();
	}
	return 0;
Efault:
	user_read_access_end();
	return -EFAULT;
}

COMPAT_SYSCALL_DEFINE6(pselect6_time64, int, n, compat_ulong_t __user *, inp,
	compat_ulong_t __user *, outp, compat_ulong_t __user *, exp,
	struct __kernel_timespec __user *, tsp, void __user *, sig)
{
	struct compat_sigset_argpack x = {0, 0};

	if (get_compat_sigset_argpack(&x, sig))
		return -EFAULT;

	return do_compat_pselect(n, inp, outp, exp, tsp, compat_ptr(x.p),
				 x.size, PT_TIMESPEC);
}

#if defined(CONFIG_COMPAT_32BIT_TIME)

COMPAT_SYSCALL_DEFINE6(pselect6_time32, int, n, compat_ulong_t __user *, inp,
	compat_ulong_t __user *, outp, compat_ulong_t __user *, exp,
	struct old_timespec32 __user *, tsp, void __user *, sig)
{
	struct compat_sigset_argpack x = {0, 0};

	if (get_compat_sigset_argpack(&x, sig))
		return -EFAULT;

	return do_compat_pselect(n, inp, outp, exp, tsp, compat_ptr(x.p),
				 x.size, PT_OLD_TIMESPEC);
}

#endif

#if defined(CONFIG_COMPAT_32BIT_TIME)
COMPAT_SYSCALL_DEFINE5(ppoll_time32, struct pollfd __user *, ufds,
	unsigned int,  nfds, struct old_timespec32 __user *, tsp,
	const compat_sigset_t __user *, sigmask, compat_size_t, sigsetsize)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		if (get_old_timespec32(&ts, tsp))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_compat_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = do_sys_poll(ufds, nfds, to);
	return poll_select_finish(&end_time, tsp, PT_OLD_TIMESPEC, ret);
}
#endif


/*
 * pselect6 系统调用完整注释
 * ========================
 *
 * 1. 功能描述：
 *    pselect6 是 POSIX select 的扩展版本，主要特性：
 *    - 支持纳秒级精度的超时（相比 select 的微秒级）
 *    - 支持原子地设置信号掩码（避免竞态条件）
 *    - 用于 I/O 多路复用，监视多个文件描述符的可读/可写/异常状态
 *
 * 2. 组成部分：
 *    - SYSCALL_DEFINE6(pselect6): 系统调用入口，参数处理和转换
 *    - do_pselect(): 核心处理函数，设置信号掩码和超时
 *    - core_sys_select(): 分配内存，复制用户数据到内核
 *    - do_select(): 实际执行 select 逻辑，轮询文件描述符
 *    - vfs_poll(): VFS 层调用具体文件系统的 poll 实现
 *
 * 3. 数据结构：
 *    - fd_set_bits: 包含6个位图（输入/输出的 in/out/except）
 *    - poll_wqueues: 管理等待队列的结构
 *    - poll_table/poll_table_entry: 用于文件描述符等待的表
 *
 * 4. 调用流程：
 *    用户空间 -> glibc -> syscall -> SYSCALL_DEFINE6(pselect6)
 *    -> do_pselect() -> core_sys_select() -> do_select() -> vfs_poll()
 *
 * 5. 时间统计关键点（用于性能分析）：
 *    - do_select() 入口：记录开始时间（ktime_get()）
 *    - poll_schedule_timeout() 前后：记录等待时间
 *    - do_select() 出口：记录结束时间，计算总耗时
 *
 *    推荐使用 trace_printk() 输出时间差，便于后续通过 ftrace 分析
 */

/* New compat syscall for 64 bit time_t*/
COMPAT_SYSCALL_DEFINE5(ppoll_time64, struct pollfd __user *, ufds,
	unsigned int,  nfds, struct __kernel_timespec __user *, tsp,
	const compat_sigset_t __user *, sigmask, compat_size_t, sigsetsize)
{
	struct timespec64 ts, end_time, *to = NULL;
	int ret;

	if (tsp) {
		if (get_timespec64(&ts, tsp))
			return -EFAULT;

		to = &end_time;
		if (poll_select_set_timeout(to, ts.tv_sec, ts.tv_nsec))
			return -EINVAL;
	}

	ret = set_compat_user_sigmask(sigmask, sigsetsize);
	if (ret)
		return ret;

	ret = do_sys_poll(ufds, nfds, to);
	return poll_select_finish(&end_time, tsp, PT_TIMESPEC, ret);
}

#endif
