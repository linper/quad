/**
 * @file glist.h
 * @brief Description of generic list data structure interface
 * @author Linas Perkauskas
 * @date 2022-02-20
 */

#pragma once

#include <stdlib.h>
#include <stdbool.h>

enum gl_fl {
	GLF_NO_ST = 0,
	GLF_SELF_ST = 1 << 0,
	GLF_ARR_ST = 1 << 1,
	GLF_ELEM_ST = 1 << 2,
	GLF_FULL_ST = GLF_SELF_ST | GLF_ARR_ST | GLF_ELEM_ST,
};

/**
 * @brief Implementation of generic list data structure
 */
typedef struct glist {
	/**Internal array to store data pointers*/
	void **array;
	/**Number of data entries currently stored*/
	size_t count;
	/**Maximum capacity of internal array*/
	size_t cap;
	enum gl_fl flags;
	/**Callback function to be called for each entry when freeing or clearing*/
	void (*free_cb)(void *);
} glist_t;

/**
 * @brief Creates glist struct with specified initial capacity
 * @param[in] cap 	Initial capacity. It must be power of 2 
 * and highier than 0, if not defaults to 16
 * @return Pointer to created glist struct if successful, NULL otherwise  
 */
glist_t *glist_new(int cap);

/**
 * @brief Cleares glist struct and and frees its entries by issuing
 * free() for each of them or free callback function instead.
 * If it was registered to list instance
 * @param[in, out] *lst	Pointer to list to clear
 * @return Void
 */
void glist_clear(glist_t *lst);

/**
 * @brief Cleares glist struct by seting its item count to 0.
 * Does not free its entries
 * @param[in, out] *lst Pointer to list to clear
 * @return Void
 */
void glist_clear_shallow(glist_t *lst);

/**
 * @brief Frees glist struct and its entries by issuing
 * free() for each of them or free callback function instead.
 * If it was registered to list instance
 * @param[in] *lst 	Pointer to list to free
 * @return Void
 */
void glist_free(glist_t *lst);

/**
 * @brief Frees glist struct but leaves its entries intact
 * @param[in] *lst 	Pointer to list to free
 * @return Void
 */
void glist_free_shallow(glist_t *lst);

/**
 * @brief Appends item  at the end of list
 * @param[in] *lst 		List pointer to append item to
 * @param[in] *value 	Item poiinter to append to list
 * @return 0 if successful
 */
int glist_push(glist_t *lst, void *value);

/**
 * @brief Appends liist at the end of other list
 * @param[in, out] 	*dst 		List pointer to append item to
 * @param[in] 		*src 		List pointer to append item to
 * @return Void
 */
void glist_extend(glist_t *dst, glist_t *src);
/**
 * @brief Gets item from list at specific index
 * @param[in] *lst 		List pointer to retrieve item from
 * @param[in] index 	Index to get item at
 * @return Pointer to value or NULL on failure.
 */
void *glist_get(glist_t *lst, int index);

/**
 * @brief Function copies entriies from one list to another
 * @param[in] *src 			Pointer to list to copy entries from 
 * @param[in, out] *dst 	Pointer to list to copy entries to
 * @return 0 if successful
 */
int glist_copy_to(glist_t *src, glist_t *dst);

/**
 * @brief Gets nubmer of items stored in glist
 * @param[in] *lst 	List pointer to get entry count from
 * @return Number of entries currently stored
 */
size_t glist_count(glist_t *lst);

/**
 * @brief Sets callback function for list to be called instead
 * of free() for every data entry when freeing list (or similar)
 * @param[in] *lst 	List pointer to add callback to
 * @param[in] *cb 	Callback function, free() replacement.
 * @return Void
 */
void glist_set_free_cb(glist_t *lst, void (*cb)(void *));

/**
 * @brief Iterator for glist struct
 * @param[out] *item Entry pointer of current iteration
 * @param[in] *list Pointer to list to iterate
 */
#define glist_foreach(item, list)                                              \
	for (int keep = 1, count = 0, size = list->count; keep && count != size;   \
		 keep = !keep, count++)                                                \
		for (item = *(list->array + count); keep; keep = !keep)

/* TODO:  <24-02-23, yourname> */
//creates shallow clone unless clone_cb is set
glist_t *clone_glist(glist_t *lst);
//copies len bytes from value and appends it at the end of list
int push_glist2(glist_t *lst, void *value, size_t len);
//inserts value at specified index
int get_idx_glist(glist_t *lst, void *value);
void *remove_glist(glist_t *lst, int index);
//deletes and frees element at index
int delete_glist(glist_t *lst, int index);
//same as delete_glist but does not free element at index
//same as remove_glist but does not return element at index
int forget_glist(glist_t *lst, int index);
void **get_array_glist(glist_t *lst);
//sets callback for every element for clone_glist
//cb(void **<pointer to clone data pointer>, void *<source data pointer>)
void set_clone_cb_glist(glist_t *lst, void (*cb)(void **, void *));

