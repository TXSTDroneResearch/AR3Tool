// This entire file is a total hack
// This makes MinGW static libs compatible with msvcr140
#define printf __msu_printf
#define fprintf __msu_fprintf
#define vprintf __msu_vprintf
#define vfprintf __msu_vfprintf
#define sscanf __msu_sscanf

#include <memory>
#include <stdarg.h>
#include <stdio.h>
#include <cstdint>

#undef printf
#undef fprintf
#undef vprintf
#undef vfprintf
#undef sscanf

extern "C" {

/*
struct _iobuf_VS2012 {
  char* _ptr;
  int _cnt;
  char* _base;
  int _flag;
  int _file;
  int _charbuf;
  int _bufsiz;
  char* _tmpfname;
};

_iobuf_VS2012* __cdecl _iob_func() {
  static _iobuf_VS2012 __iob[3];
  std::memcpy(&__iob[0], stdin, sizeof(_iobuf_VS2012));
  std::memcpy(&__iob[1], stdout, sizeof(_iobuf_VS2012));
  std::memcpy(&__iob[2], stderr, sizeof(_iobuf_VS2012));

  return __iob;
}
extern void* __imp___iob_func = _iob_func;

int __cdecl fprintf(FILE* file, const char* format, ...) {
  va_list va;
  va_start(va, format);
  int ret = _vfprintf_l(file, format, nullptr, va);
  va_end(va);

  return ret;
}

int __cdecl vfprintf(FILE* file, const char* format, va_list arg) {
  return _vfprintf_l(file, format, nullptr, arg);
}

int __cdecl vprintf(const char* format, va_list arg) {
  return __msu_vprintf(format, arg);
}

int __cdecl printf(const char* format, ...) {
  va_list va;
  va_start(va, format);
  int ret = __msu_vprintf(format, va);
  va_end(va);

  return ret;
}

int __cdecl sscanf(const char* buffer, const char* format, ...) {
  va_list va;
  va_start(va, format);
  int ret = vsscanf(buffer, format, va);
  va_end(va);

  return ret;
}

int __cdecl __ms_vsnprintf(char* d, size_t n, const char* format, va_list arg) {
  return vsnprintf(d, n, format, arg);
}

long long ftello64(FILE* file) { return _ftelli64(file); }
int fseeko64(FILE* file, long long offset, int origin) {
  return _fseeki64(file, offset, origin);
}

// TODO: This is where I got lazy
int asprintf(char** buf, const char* format, ...) { return -1; }
int vasprintf(char** buf, const char* format, va_list arg) { return -1; }
uint32_t sleep(int sec) { return -1; }
*/

}  // extern "C"