/**
 * @file  workspace.h
 * @brief One large SDRAM area for the big buffers that only one app needs at a
 *        time, instead of each keeping its own: the video player's (while a
 *        video is open) and the Files app's (folder list, picture viewer).
 *
 * An app takes the workspace before it uses it, carves its buffers out of it
 * with workspace_alloc(), and gives it back when it is done, at the latest in
 * its hide(): the apps are never on screen together, so it is always free when
 * the next one wants it. Taking it again starts with an empty area.
 * GUI task only.
 */
#ifndef WORKSPACE_H
#define WORKSPACE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Size: the largest user's needs (the video player and its controls) */
#define WORKSPACE_SIZE (3840U * 1024U)

typedef enum
{
  WORKSPACE_NONE,
  WORKSPACE_VIDEO,
  WORKSPACE_FILES,
} workspace_user_t;

/** Take the workspace for @p user (empty); false if another user holds it. */
bool workspace_take(workspace_user_t user);

/** @p size bytes (32-byte aligned) of the workspace held by @p user; NULL if it does not hold it or it is full. */
void *workspace_alloc(workspace_user_t user, size_t size);

/** Give the workspace back (nothing if @p user does not hold it). */
void workspace_give(workspace_user_t user);

#ifdef __cplusplus
}
#endif

#endif /* WORKSPACE_H */
