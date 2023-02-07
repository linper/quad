/**
 * @file json_helper.h
 * @brief Helper for partial json parsing
 * @author Linas Perkauskas
 * @date 2023-02-04
 */
#ifndef JSON_HELPER_H
#define JSON_HELPER_H

/**
 * @brief Struct that holds json analysis results
 */
struct json_status {
	const char *start; ///< 	Pointer to first found opening char
	const char *inner; ///< 	Pointer to inner table or array
	int n; ///< 		Length of table of array in bytes
	int rank; ///< 		Rank of json
};

/**
 * @brief Analyzes json like string.
 * @param[in] 	*str 	NULL terminated json-like string
 * @return 0 - on sucess, -1 otherwise 
 */
int json_help_analyze(const char *str, struct json_status *st);

/**
 * @brief Goes to start of table/array.
 * @param[in] 	*str 	NULL terminated json-like string
 * @return Pointer to the start of the table/array, NULL - on failure.
 */
const char *json_help_to_start(const char *str);

/**
 * @brief Steps 1 rank lower.
 * @param[in] 	*str 	NULL terminated json-like string
 * @return Pointer to the start of highier rank, NULL - on failure.
 */
const char *json_help_step_in(const char *str);

/**
 * @brief Steps 1 rank highier.
 * @param[in] 	*str 	NULL terminated json-like string
 * @return Pointer to the start of highier rank, NULL - on failure.
 */
const char *json_help_step_in(const char *str);

#endif // JSON_HELPER_H
