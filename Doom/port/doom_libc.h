/**
 * @file  doom_libc.h
 * @brief The C library as the Doom engine sees it, on this board.
 *
 * Included first in every Doom source file (-include). The engine is written
 * against stdio and malloc; here, files are FatFs files on the SD card,
 * console output goes to the UART log line by line, and memory comes from a
 * heap of its own in SDRAM (doom_libc.c). Only the Doom task uses them.
 */
#ifndef DOOM_LIBC_H
#define DOOM_LIBC_H

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

typedef struct doom_file doom_file_t;

doom_file_t *doom_fopen(const char *path, const char *mode);
int          doom_fclose(doom_file_t *f);
size_t       doom_fread(void *buf, size_t size, size_t count, doom_file_t *f);
size_t       doom_fwrite(const void *buf, size_t size, size_t count, doom_file_t *f);
int          doom_fgetc(doom_file_t *f);
int          doom_fseek(doom_file_t *f, long offset, int whence);
long         doom_ftell(doom_file_t *f);
int          doom_feof(doom_file_t *f);
int          doom_fflush(doom_file_t *f);
int          doom_fprintf(doom_file_t *f, const char *fmt, ...) __attribute__((format(__printf__, 2, 3)));
int          doom_vfprintf(doom_file_t *f, const char *fmt, va_list ap);
int          doom_fscanf(doom_file_t *f, const char *fmt, ...);
int          doom_printf(const char *fmt, ...) __attribute__((format(__printf__, 1, 2)));
int          doom_puts(const char *s);
int          doom_putchar(int c);
int          doom_remove(const char *path);
int          doom_mkdir(const char *path);

void *doom_malloc(size_t size);
void *doom_calloc(size_t count, size_t size);
void *doom_realloc(void *ptr, size_t size);
void  doom_free(void *ptr);
char *doom_strdup(const char *s);

char *doom_getenv(const char *name);
int   doom_system(const char *command);
void  doom_exit(int status) __attribute__((noreturn));

/* stdout / stderr: the log */
#undef stdin
#undef stdout
#undef stderr
#define stdout ((doom_file_t *)1)
#define stderr ((doom_file_t *)2)

#undef putchar
#undef feof
#undef fgetc
#undef puts
#define FILE         doom_file_t
#define fopen        doom_fopen
#define fclose       doom_fclose
#define fread        doom_fread
#define fwrite       doom_fwrite
#define fgetc        doom_fgetc
#define fseek        doom_fseek
#define ftell        doom_ftell
#define feof         doom_feof
#define fflush       doom_fflush
#define fprintf      doom_fprintf
#define vfprintf     doom_vfprintf
#define fscanf       doom_fscanf
#define printf       doom_printf
#define puts         doom_puts
#define putchar      doom_putchar
#define remove       doom_remove
#define mkdir(p, ...) doom_mkdir(p)

#define malloc       doom_malloc
#define calloc       doom_calloc
#define realloc      doom_realloc
#define free         doom_free
#define strdup       doom_strdup

#define getenv       doom_getenv
#define system       doom_system
#define exit         doom_exit

#endif /* DOOM_LIBC_H */
