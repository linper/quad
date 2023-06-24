/**
 * @file glist.c
 * @brief Implementation of generic list data structure interface
 * @author Linas Perkauskas
 * @date 2022-02-20
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stddef.h>

#include "glist.h"
#include "log.h"

/**
 * @brief Doubles capacity of list and copies entries to new array
 * @param[in, out] *lst	Pointer to list to extend
 * @return Void
 */
static void extend_storage(glist_t *lst)
{
	void **new_array;
	new_array = calloc(lst->cap * 2, sizeof(void *));

	if (!new_array) {
		FATAL(ERR_MALLOC_FAIL);
	}

	memcpy(new_array, lst->array, sizeof(void *) * lst->cap);
	free(lst->array);
	lst->array = new_array;
	lst->cap *= 2;
}

/**
 * @brief Enables indexing by negative indices
 * @param[in] *lst			Pointer to to retrieve modefied index from
 * @param[in, out] *index 	Index to be conferted to non negative
 * @return 0 if successful
 */
static int convert_index_glist(glist_t *lst, int *index)
{
	int count = (int)lst->count;
	int index_val = *index;

	if ((count + index_val) < 0) {
		return 1;
	}

	if (index_val >= 0) {
		return 0;
	}

	*index = count + index_val;
	return 0;
}

glist_t *glist_new(int cap)
{
	void **array;
	glist_t *lst;

	if (cap < 1 || cap % 2 == 1) {
		cap = 16;
	}

	lst = calloc(1, sizeof(glist_t));
	if (!lst) {
		FATAL(ERR_MALLOC_FAIL);
	}

	lst->cap = cap;

	array = calloc(cap, sizeof(void *));
	if (!array) {
		FATAL(ERR_MALLOC_FAIL);
	}

	lst->array = array;

	return lst;
}

void glist_clear(glist_t *lst)
{
	for (size_t i = 0; i < lst->count; i++) {
		if (lst->free_cb != NULL) {
			lst->free_cb(lst->array[i]);
		} else {
			free(lst->array[i]);
		}
	}
	lst->count = 0;
}

void glist_clear_shallow(glist_t *lst)
{
	if (lst) {
		lst->count = 0;
	}
}

void glist_free(glist_t *lst)
{
	if (lst) {
		glist_clear(lst);
		free(lst->array);
		free(lst);
	}
}

void glist_free_shallow(glist_t *lst)
{
	if (lst) {
		free(lst->array);
		free(lst);
	}
}

void glist_extend(glist_t *dst, glist_t *src)
{
	while ((dst->count + src->count) > dst->cap) {
		extend_storage(dst);
	}

	for (size_t i = 0; i < src->count; ++i) {
		dst->array[dst->count++] = src->array[i];
	}
}

int glist_push(glist_t *lst, void *value)
{
	if (lst->count == lst->cap) {
		extend_storage(lst);
	}

	lst->array[lst->count++] = value;
	return 0;
}

void *glist_get(glist_t *lst, int index)
{
	int idx = index;
	if (convert_index_glist(lst, &idx) || (size_t)idx >= lst->count) {
		return NULL;
	}

	return lst->array[idx];
}

void *glist_remove(struct glist *lst, int index)
{
	int idx = index;
	if (convert_index_glist(lst, &idx) || (size_t)idx >= lst->count) {
		return NULL;
	}

	void *value = lst->array[idx];

	for (size_t i = index; i < lst->count - 1; i++) {
		lst->array[i] = lst->array[i + 1];
	}

	lst->count--;

	return value;
}

int glist_copy_to(glist_t *src, glist_t *dst)
{
	while (src->count + dst->count >= dst->cap) {
		extend_storage(dst);
	}

	for (size_t i = 0; i < src->count; i++) {
		dst->array[dst->count++] = src->array[i];
	}

	return 0;
}

int glist_move_n_to(glist_t *src, glist_t *dst, size_t soff, size_t doff,
					size_t n)
{
	if (soff + n > n) {
		ERR(ERR_INVALID_INPUT);
		return 1;
	}

	while (dst->count + n >= dst->cap) {
		extend_storage(dst);
	}

	// make space for new entries
	memmove(dst->array + doff + n, dst->array + doff,
			sizeof(void *) * (dst->count - doff));
	// copy entries
	memcpy(dst->array + doff, src->array + soff, sizeof(void *) * n);
	// remove leftover empty space in src
	memmove(src->array + soff, src->array + soff + n,
			sizeof(void *) * (src->count - soff - n));
	memset(src->array + src->count - n, 0, sizeof(void *) * n);

	src->count -= n;
	dst->count += n;

	return 0;
}

size_t glist_count(glist_t *lst)
{
	if (lst)
		return lst->count;
	return 0;
}

void glist_set_free_cb(glist_t *lst, void (*cb)(void *))
{
	if (lst)
		lst->free_cb = cb;
}

