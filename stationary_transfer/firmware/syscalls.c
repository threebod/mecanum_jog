/* ARM GCC/newlib support. Keil uses its own C runtime. */
#if defined(__GNUC__)
#include <sys/stat.h>
#include <errno.h>
#include <stddef.h>
#include "platform.h"
extern char __heap_start__,__heap_end__;
void *_sbrk(ptrdiff_t increment) {
    static char *cursor;char *old;
    if(!cursor)cursor=&__heap_start__;
    if(increment<0 || increment>(&__heap_end__-cursor)){errno=ENOMEM;return (void*)-1;}
    old=cursor;cursor+=increment;return old;
}
int _close(int fd){(void)fd;errno=EBADF;return -1;}
int _fstat(int fd,struct stat *s){(void)fd;s->st_mode=S_IFCHR;return 0;}
int _isatty(int fd){return fd>=0&&fd<=2;}
int _lseek(int fd,int offset,int whence){(void)fd;(void)offset;(void)whence;errno=ESPIPE;return -1;}
int _read(int fd,char *buffer,int length){(void)fd;(void)buffer;(void)length;errno=EAGAIN;return -1;}
int _write(int fd,char *buffer,int length){(void)fd;(void)buffer;(void)length;errno=EBADF;return -1;}
int _getpid(void){return 1;}
int _kill(int pid,int sig){(void)pid;(void)sig;errno=EINVAL;return -1;}
void _exit(int status){(void)status;platform_stop();for(;;){}}
#endif
