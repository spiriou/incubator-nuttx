/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <poll.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/net.h>
#include <socket/socket.h>

// FIXME
#define CONFIG_NET_SOCKETPAIR_STREAM 1
#define CONFIG_NET_SOCKETPAIR_DGRAM 1
#define CONFIG_NET_SOCKETPAIR_CBUF_SIZE 64
#define CONFIG_NET_SOCKETPAIR_NPOLLWAITERS 4

typedef struct spair_waiter_sem_s
{
  sem_t sem;
  struct spair_waiter_sem_s *next;
} spair_waiter_sem_t;

typedef struct spair_rdwr_req_s
{
  size_t len;
  uint8_t *data;
  struct spair_rdwr_req_s *next;
} spair_rdwr_req_t;

struct spair_buffer_s
{
  struct   spair_buffer_s *peer;
  sem_t    exclsem;           /* Enforces device exclusive access */
  spair_waiter_sem_t *rdsems; /* List of blocking readers */
  spair_waiter_sem_t *wrsems; /* List of blocking writers */
  FAR struct pollfd *fds[CONFIG_NET_SOCKETPAIR_NPOLLWAITERS];
  spair_rdwr_req_t write_bufs;
  spair_rdwr_req_t read_bufs;
  uint8_t  crefs;             /* References counts on eventfd (max: 255) */
  uint8_t cbuf_full;
  uint16_t read_idx;
  uint16_t write_idx;
  uint8_t  cbuf[CONFIG_NET_SOCKETPAIR_CBUF_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        spair_setup(FAR struct socket *psock, int protocol);
static sockcaps_t spair_sockcaps(FAR struct socket *psock);
static void       spair_addref(FAR struct socket *psock);
static int        spair_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);
static int        spair_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen);

static int        spair_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup);
static ssize_t    spair_send(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags);
static ssize_t    spair_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen);
static int        spair_close(FAR struct socket *psock);
static ssize_t    spair_recvfrom(FAR struct socket *psock, FAR void *buf,
                    size_t len, int flags, FAR struct sockaddr *from,
                    FAR socklen_t *fromlen);

static ssize_t    spair_stream_recvfrom(FAR struct socket *psock, FAR void *buf,
                    size_t len, int flags, FAR struct sockaddr *from,
                    FAR socklen_t *fromlen);
static ssize_t    spair_dgram_recvfrom(FAR struct socket *psock, FAR void *buf,
                    size_t len, int flags, FAR struct sockaddr *from,
                    FAR socklen_t *fromlen);

/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct sock_intf_s spair_sockif =
{
  spair_setup,       /* si_setup */
  spair_sockcaps,    /* si_sockcaps */
  spair_addref,      /* si_addref */
  NULL,              /* si_bind */
  spair_getsockname, /* si_getsockname */
  spair_getpeername, /* si_getpeername */
  NULL,              /* si_listen */
  NULL,              /* si_connect */
  NULL,              /* si_accept */
  spair_poll,        /* si_poll */
  spair_send,        /* si_send */
  spair_sendto,      /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,              /* si_sendfile */
#endif
  spair_recvfrom,    /* si_recvfrom */
#ifdef CONFIG_NET_CMSG
  NULL,              /* si_recvmsg */
  NULL,              /* si_sendmsg */
#endif
  spair_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int spair_blocking_io(FAR struct spair_buffer_s *conn,
                             spair_waiter_sem_t *sem,
                             FAR spair_waiter_sem_t **slist)
{
  int ret;
  sem->next = *slist;
  *slist = sem;

  nxsem_post(&conn->exclsem);

  /* Wait for eventfd to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      /* Interrupted wait, unregister semaphore
       * TODO ensure that exclsem wait does not fail (ECANCELED)
       */

      nxsem_wait_uninterruptible(&conn->exclsem);

      spair_waiter_sem_t *cur_sem = *slist;

      if (cur_sem == sem)
        {
          *slist = sem->next;
        }
      else
        {
          while (cur_sem)
            {
              if (cur_sem->next == sem)
                {
                  cur_sem->next = sem->next;
                  break;
                }
            }
        }

      nxsem_post(&conn->exclsem);
      return ret;
    }

  return nxsem_wait(&conn->exclsem);
}

static void spair_pollnotify(FAR struct spair_buffer_s *conn,
                             pollevent_t eventset)
{
  FAR struct pollfd *fds;
  int i;

  for (i = 0; i < CONFIG_NET_SOCKETPAIR_NPOLLWAITERS; i++)
    {
      fds = conn->fds[i];
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

static int spair_setup(FAR struct socket *psock, int protocol)
{
  _err("entry\n");
  return 0;
}

static sockcaps_t spair_sockcaps(FAR struct socket *psock)
{
  _err("entry\n");
  return SOCKCAP_NONBLOCKING;
}

static void spair_addref(FAR struct socket *psock)
{
  _err("entry\n");
}

static int spair_getsockname(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  _err("entry\n");
  return 0;
}

static int spair_getpeername(FAR struct socket *psock,
                    FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  _err("entry\n");
  return 0;
}

static int spair_poll(FAR struct socket *psock,
                    FAR struct pollfd *fds, bool setup)
{
  int ret;
  int i;
  pollevent_t eventset;
  FAR struct spair_buffer_s *conn;

  _err("entry %d\n", setup);

  conn = (FAR struct spair_buffer_s*)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  ret = nxsem_wait(&conn->exclsem);
  if (ret < 0)
    {
      return ret;
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

  for (i = 0; i < CONFIG_NET_SOCKETPAIR_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!conn->fds[i])
        {
          /* Bind the poll structure and this slot */

          conn->fds[i] = fds;
          fds->priv   = &conn->fds[i];
          break;
        }
    }

  if (i >= CONFIG_NET_SOCKETPAIR_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto errout;
    }

  /* Notify the POLLOUT event if peer buffer is not full */

  // FIXME check peer
  eventset = 0;
  if (!conn->peer->cbuf_full)
    {
      eventset |= POLLOUT;
    }

  /* Notify the POLLIN event if buffer is not empty */

  if (conn->write_idx != conn->read_idx || conn->cbuf_full)
    {
      eventset |= POLLIN;
    }

  if (eventset)
    {
      spair_pollnotify(conn, eventset);
    }

errout:
  nxsem_post(&conn->exclsem);
  return ret;
}

static ssize_t spair_send(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags)
{
  _err("entry %d 0x%x\n", len, flags);
  ssize_t ret;
  FAR struct spair_buffer_s *conn;

  conn = (FAR struct spair_buffer_s*)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  conn = conn->peer;

  ret = nxsem_wait(&conn->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (conn->crefs <= 0)
    {
      ret = -ENOTCONN;
      goto exit_post_sem;
    }

  // TODO DGRAM ??

  ssize_t write_len = 0;

  if (len <= 0)
    {
      goto write_done;
    }

  if (conn->cbuf_full)
    {
      if (_SS_ISNONBLOCK(psock->s_flags) || (flags & MSG_DONTWAIT) != 0)
        {
          /* Non blocking io */

          ret = -EWOULDBLOCK;
          goto exit_post_sem;
        }

      spair_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);
      nxsem_set_protocol(&sem.sem, SEM_PRIO_NONE);
      do
        {
          ret = spair_blocking_io(conn, &sem, &conn->wrsems);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              goto exit_post_sem;
            }
        }
      while (conn->cbuf_full);

      nxsem_destroy(&sem.sem);
    }

  /* Process write */

  if (conn->write_idx >= conn->read_idx)
    {
      /* handle cbuf overflow */

      ssize_t avail = CONFIG_NET_SOCKETPAIR_CBUF_SIZE-conn->write_idx;

      if (len >= avail)
        {
          if (conn->read_idx == 0)
            {
              conn->cbuf_full = 1;
            }
          write_len = avail;
        }
      else
        {
          write_len = len;
        }

      memcpy(&conn->cbuf[conn->write_idx], buf, write_len);
      len -= write_len;
      buf += write_len;
      conn->write_idx = (conn->write_idx+write_len) % CONFIG_NET_SOCKETPAIR_CBUF_SIZE;
    }

  if (conn->cbuf_full)
    {
      goto write_done;
    }

  if (len > 0)
    {
      /* Handle remaining write */

      ssize_t avail = conn->read_idx-conn->write_idx;
      ssize_t write_len2;

      if (len >= avail)
        {
          conn->cbuf_full = 1;
          write_len2 = avail;
        }
      else
        {
          write_len2 = len;
        }

      memcpy(&conn->cbuf[conn->write_idx], buf, write_len2);
      len -= write_len2;
      conn->write_idx = (conn->write_idx+write_len2) % CONFIG_NET_SOCKETPAIR_CBUF_SIZE;
      write_len += write_len2;
    }

write_done:
  ret = write_len;

  /* Notify all of the waiting readers */

  spair_waiter_sem_t *cur_sem = conn->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  conn->rdsems = NULL;

  /* Notify all poll/select waiters */

  spair_pollnotify(conn, POLLIN);

exit_post_sem:
  nxsem_post(&conn->exclsem);
  return ret;
}

static ssize_t spair_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen)
{
  _err("entry\n");
  return spair_send(psock, buf, len, flags);
}

static int spair_close(FAR struct socket *psock)
{
  _err("entry\n");
  return 0;
}

static ssize_t spair_stream_recvfrom(FAR struct socket *psock, FAR void *buf,
                    size_t len, int flags, FAR struct sockaddr *from,
                    FAR socklen_t *fromlen)
{
  _err("entry %d 0x%x\n", len, flags);
  ssize_t ret;
  FAR struct spair_buffer_s *conn;

  conn = (FAR struct spair_buffer_s*)psock->s_conn;
  DEBUGASSERT(conn != NULL);

  ret = nxsem_wait(&conn->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (conn->crefs <= 0)
    {
      ret = -ENOTCONN;
      goto exit_post_sem;
    }

  ssize_t read_len = 0;

  if (len <= 0)
    {
        goto read_done;
    }

  _err("read %d %d\n", conn->read_idx, conn->write_idx);

  if (conn->read_idx == conn->write_idx && !conn->cbuf_full)
    {
      /* Buffer empty */

      if (_SS_ISNONBLOCK(psock->s_flags) || (flags & MSG_DONTWAIT) != 0)
        {
          /* Non blocking io */

          ret = -EWOULDBLOCK;
          goto exit_post_sem;
        }

      spair_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);
      nxsem_set_protocol(&sem.sem, SEM_PRIO_NONE);

      do
        {
          ret = spair_blocking_io(conn, &sem, &conn->rdsems);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (conn->read_idx == conn->write_idx && !conn->cbuf_full);

      nxsem_destroy(&sem.sem);
    }

  /* Process read */

  if (conn->read_idx >= conn->write_idx)
    {
      /* handle cbuf overflow */

      conn->cbuf_full = 0;

      ssize_t avail = CONFIG_NET_SOCKETPAIR_CBUF_SIZE-conn->read_idx;
      read_len = len > avail ? avail : len;

      memcpy(buf, &conn->cbuf[conn->read_idx], read_len);
      len -= read_len;
      buf += read_len;
      conn->read_idx = (conn->read_idx+read_len) % CONFIG_NET_SOCKETPAIR_CBUF_SIZE;
    }

  if (len > 0)
    {
      /* Handle remaining read */

      ssize_t avail = conn->write_idx-conn->read_idx;
      if (avail == 0)
        {
          goto read_done;
        }

      ssize_t read_len2 = len > avail ? avail : len;

      memcpy(buf, &conn->cbuf[conn->read_idx], read_len2);
      len -= read_len2;
      conn->read_idx = (conn->read_idx+read_len2) % CONFIG_NET_SOCKETPAIR_CBUF_SIZE;
      read_len += read_len2;
    }

read_done:
  ret = read_len;

  /* Notify all waiting writers that counter have been decremented */

  spair_waiter_sem_t *cur_sem = conn->wrsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  conn->wrsems = NULL;

  /* Notify all poll/select waiters */

  // FIXME check peer
  spair_pollnotify(conn->peer, POLLOUT);

exit_post_sem:
  nxsem_post(&conn->exclsem);
  return ret;
}

static ssize_t spair_dgram_recvfrom(FAR struct socket *psock, FAR void *buf,
                    size_t len, int flags, FAR struct sockaddr *from,
                    FAR socklen_t *fromlen)
{
  _err("entry\n");
  return -EOPNOTSUPP;
}

static ssize_t spair_recvfrom(FAR struct socket *psock, FAR void *buf,
                       size_t len, int flags, FAR struct sockaddr *from,
                       FAR socklen_t *fromlen)
{
  DEBUGASSERT(psock && psock->s_conn && buf);

  /* Check for a stream socket */

#ifdef CONFIG_NET_SOCKETPAIR_STREAM
  if (psock->s_type == SOCK_STREAM)
    {
      return spair_stream_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else
#endif

#ifdef CONFIG_NET_SOCKETPAIR_DGRAM
  if (psock->s_type == SOCK_DGRAM)
    {
      return spair_dgram_recvfrom(psock, buf, len, flags, from, fromlen);
    }
  else
#endif
    {
      DEBUGPANIC();
      nerr("ERROR: Unrecognized socket type: %d\n", psock->s_type);
      return -EINVAL;
    }
}

static int spair_alloc_socket(int domain, int type, int protocol)
{
  FAR struct socket *psock;
  int errcode;
  int sockfd;

  /* Allocate a socket descriptor */

  sockfd = sockfd_allocate(0);
  if (sockfd < 0)
    {
      nerr("ERROR: Failed to allocate a socket descriptor\n");
      errcode = ENFILE;
      goto errout;
    }

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);
  if (!psock)
    {
      errcode = ENOSYS; /* should not happen */
      goto errout_with_sockfd;
    }

  /* Initialize the socket structure */

  psock->s_crefs  = 1;
  psock->s_domain = domain;
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) || defined(CONFIG_NET_UDP_WRITE_BUFFERS)
  psock->s_sndcb  = NULL;
#endif

  if (type & SOCK_CLOEXEC)
    {
      psock->s_flags |= _SF_CLOEXEC;
    }

  if (type & SOCK_NONBLOCK)
    {
      psock->s_flags |= _SF_NONBLOCK;
    }

  type            &= SOCK_TYPE_MASK;
  psock->s_type   = type;

  /* Get the socket interface */

  psock->s_sockif = &spair_sockif;

  /* The socket has been successfully initialized */

  return sockfd;

errout_with_sockfd:
  sockfd_release(sockfd);

errout:
  set_errno(errcode);
  return ERROR;
}

static int spair_ctx_init(struct spair_buffer_s *buf, FAR struct spair_buffer_s *peer)
{
  int ret;
  
  ret = nxsem_init(&buf->exclsem, 0, 1);
  if (ret != 0)
    {
      return ret;
    }

  buf->peer = peer;
  buf->rdsems = NULL;
  buf->wrsems = NULL;
  buf->read_idx = 0;
  buf->write_idx = 0;
  buf->cbuf_full = 0;

  /* One socket is connected to this endpoint */

  buf->crefs = 1;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int socketpair(int domain, int type, int protocol, int sv[2])
{
  int ret;
  int sock1, sock2;
  FAR struct socket *psock1;
  FAR struct socket *psock2;
  FAR struct spair_buffer_s *ctx;

  _info("entry\n");
  if (domain != PF_LOCAL)
    {
      ret = -EINVAL;
      goto errout;
    }
  if (type != SOCK_STREAM && type != SOCK_DGRAM)
    {
      ret = -EINVAL;
      goto errout;
    }

  sock1 = spair_alloc_socket(domain, type, protocol);
  if (sock1 < 0)
    {
      ret = sock1;
      goto errout;
    }

  sock2 = spair_alloc_socket(domain, type, protocol);
  if (sock2 < 0)
    {
      /* close sock1 */
      ret = sock2;
      goto errout;
    }

  _err("OK sock1 sock2 %d %d\n", sock1, sock2);

  psock1 = sockfd_socket(sock1);
  psock2 = sockfd_socket(sock2);

  ctx = (FAR struct spair_buffer_s *)
    kmm_malloc(2*sizeof(struct spair_buffer_s));
  if (ctx == NULL)
    {
      goto exit_release_socket;
    }

  spair_ctx_init(&ctx[0], &ctx[1]);
  spair_ctx_init(&ctx[1], &ctx[0]);

  psock1->s_conn = (FAR void*) &ctx[0];
  psock2->s_conn = (FAR void*) &ctx[1];
  psock1->s_flags |= _SF_INITD;
  psock2->s_flags |= _SF_INITD;

  sv[0] = sock1;
  sv[1] = sock2;
  return 0;

exit_release_socket:
  sockfd_release(sock1);
  sockfd_release(sock2);

errout:
  set_errno(-ret);
  return ERROR;
}