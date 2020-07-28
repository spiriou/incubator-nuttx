/****************************************************************************
 * vfs/fs_eventfd.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>

#include <debug.h>

#include <sys/ioctl.h>
#include <sys/eventfd.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EVENTFD_FMT    "/dev/efd%d"

#ifndef CONFIG_EVENT_FD_NPOLLWAITERS
/* Maximum number of threads than can be waiting for POLL events */
#  define CONFIG_EVENT_FD_NPOLLWAITERS 2
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the internal state of the driver */

struct eventfd_priv_s
{
  sem_t     exclsem;        /* Enforces exclusive access to device counter */
  sem_t     rdsem;          /* Empty buffer - Reader waits for data write */
  sem_t     wrsem;          /* Full buffer - Writer waits for data read */
  uint8_t   crefs;          /* References counts on eventfd (limited to 255) */
  uint8_t   minor;          /* eventfd minor number */
  uint8_t   mode_semaphore; /* eventfd mode (semaphore or counter) */
  eventfd_t counter;        /* eventfd counter */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifdef CONFIG_EVENT_FD_POLL
  FAR struct pollfd *d_fds[CONFIG_EVENT_FD_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int eventfd_open(FAR struct file *filep);
static int eventfd_close(FAR struct file *filep);

static ssize_t eventfd_do_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t eventfd_do_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);
static int eventfd_do_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#ifdef CONFIG_EVENT_FD_POLL
static int eventfd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);
#endif

static int eventfd_alloc_dev(eventfd_t counter, int flags);

#ifdef CONFIG_EVENT_FD_POLL
static void eventfd_pollnotify(FAR struct eventfd_priv_s *dev,
                               pollevent_t eventset);
#endif

static int get_unique_minor(void);
static void release_minor(int minor);

static FAR struct eventfd_priv_s *eventfd_allocdev(void);
static void eventfd_destroy(FAR struct eventfd_priv_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_eventfd_fops =
{
  eventfd_open,     /* open */
  eventfd_close,    /* close */
  eventfd_do_read,  /* read */
  eventfd_do_write, /* write */
  0,                /* seek */
  eventfd_do_ioctl  /* ioctl */
#ifdef CONFIG_EVENT_FD_POLL
  , eventfd_poll    /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

FAR struct eventfd_priv_s *eventfd_allocdev(void)
{
  FAR struct eventfd_priv_s *dev;

  dev = (FAR struct eventfd_priv_s *)
    kmm_malloc(sizeof(struct eventfd_priv_s));
  if (dev)
    {
      /* Initialize the private structure */

      memset(dev, 0, sizeof(struct eventfd_priv_s));
      nxsem_init(&dev->exclsem, 0, 0);
      nxsem_init(&dev->rdsem, 0, 0);
      nxsem_init(&dev->wrsem, 0, 0);

      /* The read/write wait semaphores are used for signaling
       * and should not have priority inheritance enabled.
       */

      nxsem_set_protocol(&dev->rdsem, SEM_PRIO_NONE);
      nxsem_set_protocol(&dev->wrsem, SEM_PRIO_NONE);
    }

  return dev;
}

void eventfd_destroy(FAR struct eventfd_priv_s *dev)
{
  nxsem_destroy(&dev->exclsem);
  nxsem_destroy(&dev->rdsem);
  nxsem_destroy(&dev->wrsem);
  kmm_free(dev);
}

#ifdef CONFIG_EVENT_FD_POLL
void eventfd_pollnotify(FAR struct eventfd_priv_s *dev,
                               pollevent_t eventset)
{
  FAR struct pollfd *fds;
  int i;

  for (i = 0; i < CONFIG_EVENT_FD_NPOLLWAITERS; i++)
    {
      fds = dev->d_fds[i];
      if (fds)
        {
          fds->revents |= eventset & fds->events;

          if (fds->revents != 0)
            {
              nxsem_post(fds->sem);
            }
        }
    }
}
#endif

int get_unique_minor(void)
{
  static int minor = 0;

  /* TODO reimplement this with minor bit map ? */

  return (minor++) & 511;
}

void release_minor(int minor)
{
}

int eventfd_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return -1;
    }

  finfo("crefs: %d <%s>\n", priv->crefs, inode->i_name);

  if (priv->crefs >= 255)
    {
      /* More than 255 opens; uint8_t would overflow to zero */

      ret = -EMFILE;
    }
  else
    {
      /* Save the new open count on success */

      priv->crefs += 1;
      ret = OK;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

int eventfd_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;
  char devpath[16];
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return -errno;
    }

  finfo("crefs: %d <%s>\n", priv->crefs, inode->i_name);

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (priv->crefs > 1)
    {
      /* Just decrement the reference count and release the semaphore */

      priv->crefs -= 1;
      nxsem_post(&priv->exclsem);
      return OK;
    }

  /* Re-create the path to the driver. */

  finfo("destroy\n");
  ret = sprintf(devpath, EVENTFD_FMT, priv->minor);
  if (ret < 0)
    {
      ferr("ERROR: Failed to allocate the driver path\n");
      nxsem_post(&priv->exclsem);
      return -ENOMEM;
    }

  /* Will be unregistered later after close is done */

  unregister_driver(devpath);

  DEBUGASSERT(priv->exclsem.semcount == 0);
  release_minor(priv->minor);
  eventfd_destroy(priv);

  return OK;
}

ssize_t eventfd_do_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  ssize_t ret;
  int sval;

  if (len < sizeof(eventfd_t) || buffer == NULL)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return -errno;
    }

  /* Wait for an incoming event */

  while (dev->counter == 0)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxsem_post(&dev->exclsem);
          return -EAGAIN;
        }

      /* Wait for a writer to notify */

      sched_lock();
      nxsem_post(&dev->exclsem);
      ret = nxsem_wait(&dev->rdsem);
      sched_unlock();

      if (ret < 0 || (ret = nxsem_wait(&dev->exclsem)) < 0)
        {
          return ret;
        }
    }

  /* Device ready for read */

  if (dev->mode_semaphore)
    {
      *(FAR eventfd_t *)buffer = 1;
      dev->counter -= 1;
    }
  else
    {
      *(FAR eventfd_t *)buffer = dev->counter;
      dev->counter = 0;
    }

  /* Notify all waiting writers that counter have been decremented */

  while (nxsem_get_value(&dev->wrsem, &sval) == 0 && sval < 0)
    {
      nxsem_post(&dev->wrsem);
    }

#ifdef CONFIG_EVENT_FD_POLL
  /* Notify all poll/select waiters */

  eventfd_pollnotify(dev, POLLOUT);
#endif

  nxsem_post(&dev->exclsem);
  return sizeof(eventfd_t);
}

ssize_t eventfd_do_write(FAR struct file *filep, FAR const char *buffer,
  size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  ssize_t ret;
  int sval;
  eventfd_t new_counter;

  if (len < sizeof(eventfd_t) || buffer == NULL ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)-1) ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)0))
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return -errno;
    }

  while ((new_counter = dev->counter + *(FAR eventfd_t *)buffer)
    < dev->counter)
    {
      /* Overflow detected */

      if (filep->f_oflags & O_NONBLOCK)
        {
          nxsem_post(&dev->exclsem);
          return -EAGAIN;
        }

      /* Wait for eventfd reader */

      sched_lock();
      nxsem_post(&dev->exclsem);
      ret = nxsem_wait(&dev->wrsem);
      sched_unlock();

      if (ret < 0 || (ret = nxsem_wait(&dev->exclsem)) < 0)
        {
          return ret;
        }
    }

  /* Ready to write, update counter */

  dev->counter = new_counter;

  /* Notify all of the waiting readers */

  while (nxsem_get_value(&dev->rdsem, &sval) == 0 && sval < 0)
    {
      nxsem_post(&dev->rdsem);
    }

#ifdef CONFIG_EVENT_FD_POLL
  /* Notify all poll/select waiters */

  eventfd_pollnotify(dev, POLLIN);
#endif

  nxsem_post(&dev->exclsem);
  return sizeof(eventfd_t);
}

int eventfd_do_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;

  /* Only one ioctl implemented that returns device minor number */

  return priv->minor;
}

#ifdef CONFIG_EVENT_FD_POLL
int eventfd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  int ret;
  int i;
  pollevent_t eventset;

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return -errno;
    }

  ret = OK;

  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
      goto errout;
    }

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_EVENT_FD_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!dev->d_fds[i])
        {
          /* Bind the poll structure and this slot */

          dev->d_fds[i] = fds;
          fds->priv     = &dev->d_fds[i];
          break;
        }
    }

  if (i >= CONFIG_EVENT_FD_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto errout;
    }

  /* Notify the POLLOUT event if the pipe is not full, but only if
   * there is readers.
   */

  eventset = 0;
  if (dev->counter < (eventfd_t)-1)
    {
      eventset |= POLLOUT;
    }

  /* Notify the POLLIN event if the pipe is not empty */

  if (dev->counter > 0)
    {
      eventset |= POLLIN;
    }

  if (eventset)
    {
      eventfd_pollnotify(dev, eventset);
    }

errout:
  nxsem_post(&dev->exclsem);
  return ret;
}
#endif

int eventfd_alloc_dev(eventfd_t init_counter, int flags)
{
  int ret;
  int new_fd;
  unsigned int new_minor;
  FAR struct eventfd_priv_s *new_dev;
  char devpath[16];

  /* Allocate instance data for this driver */

  new_dev = eventfd_allocdev();
  if (new_dev == NULL)
    {
      /* Failed to allocate new device */

      return -ENOMEM;
    }

  new_dev->counter = init_counter;
  new_dev->mode_semaphore = !!(flags & EFD_SEMAPHORE);

  /* Request a unique minor device number */

  new_minor = get_unique_minor();

  if (new_minor < 0)
    {
      ferr("Cannot get minor\n");
      ret = -EMFILE;
      goto exit_free_new_dev;
    }

  new_dev->minor = new_minor;

  /* Get device path */

  if ((ret = sprintf(devpath, EVENTFD_FMT, new_minor)) < 0)
    {
      ret = -ENOMEM;
      goto exit_release_minor;
    }

  /* Register the driver */

  ret = register_driver(devpath, &g_eventfd_fops, 0666, new_dev);
  if (ret < 0)
    {
      ferr("Failed to register new device %s: %d\n", devpath, ret);
      ret = -ENOMEM;
      goto exit_release_minor;
    }

  /* Device is ready for use */

  nxsem_post(&new_dev->exclsem);

  /* Try open new device */

  new_fd = open(devpath, O_RDWR | (flags & (EFD_NONBLOCK | EFD_SEMAPHORE)));

  if (new_fd < 0)
    {
      ret = -EMFILE;
      goto exit_unregister_driver;
    }

  return new_fd;

  exit_unregister_driver:
    unregister_driver(devpath);
  exit_release_minor:
    release_minor(new_minor);
  exit_free_new_dev:
    eventfd_destroy(new_dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int eventfd(unsigned int count, int flags)
{
  int fd = eventfd_alloc_dev(count, flags);
  if (fd < 0)
    {
      set_errno(-fd);
      return -1;
    }

  return fd;
}

int eventfd_read(int fd, FAR eventfd_t *value)
{
  return read(fd, value, sizeof (eventfd_t)) != sizeof (eventfd_t) ? -1 : 0;
}

int eventfd_write(int fd, eventfd_t value)
{
  return write(fd, &value,
      sizeof (eventfd_t)) != sizeof (eventfd_t) ? -1 : 0;
}

int eventfd_get_minor(int fd)
{
  return ioctl(fd, 0, NULL);
}
