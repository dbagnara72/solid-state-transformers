#ifndef IGBT_TEMPERATURE_ESTIMATION_H
#define IGBT_TEMPERATURE_ESTIMATION_H

/********************************
 * Includes
 *********************************/

#include <stdint.h>

/********************************
 * Macros
 *********************************/

/********************************
 * Type-definitions
 *********************************/
/*!
 * \brief igbt_temperature_estimation config structure.
 */
typedef struct igbt_temperature_estimation_config_s {
	
} igbt_temperature_estimation_config_t;

/*!
 * brief igbt_temperature_estimation input structure.
 */
typedef struct igbt_temperature_estimation_in_s {
	
} igbt_temperature_estimation_in_t;

/*!
 * brief igbt_temperature_estimation output structure.
 */
typedef struct igbt_temperature_estimation_out_s {
	
} igbt_temperature_estimation_out_t;

/*!
 * brief igbt_temperature_estimation structure.
 */
typedef struct igbt_temperature_estimation_s {
	igbt_temperature_estimation_config_t		config;					/*!< Config structure. */
	igbt_temperature_estimation_in_t			input;					/*!< Input structure. */
	igbt_temperature_estimation_out_t			output;					/*!< Output structure. */
} igbt_temperature_estimation_t;

/*!
 * brief igbt_temperature_estimation process function pointer type.
 */
typedef void (*igbt_temperature_estimation_process_t)(igbt_temperature_estimation_t *const igbt_temperature_estimation);

/********************************
 * Function declarations
 *********************************/
/*!
 * \brief Initialize a igbt_temperature_estimation object.
 * \param igbt_temperature_estimation		Pointer to a igbt_temperature_estimation object structure.
 * \param config		Pointer to a igbt_temperature_estimation config structure.
 * \param input			Pointer to a igbt_temperature_estimation input structure.
 *
 * This function is used to initialize a igbt_temperature_estimation object.
 */
void IgbtTemperatureEstimation_Init(igbt_temperature_estimation_t *const igbt_temperature_estimation, const igbt_temperature_estimation_in_t *const input, const igbt_temperature_estimation_config_t *const config);

/*!
 * \brief Reset the igbt_temperature_estimation object.
 * \param igbt_temperature_estimation		Pointer to a igbt_temperature_estimation object structure.
 *
 */
void IgbtTemperatureEstimation_Reset(igbt_temperature_estimation_t *const igbt_temperature_estimation);

/*!
 * \brief Process igbt_temperature_estimation object.
 * \param igbt_temperature_estimation		Pointer to a igbt_temperature_estimation object structure.
 *
 */
void IgbtTemperatureEstimation_Process(igbt_temperature_estimation_t *const igbt_temperature_estimation);

#endif /* IGBT_TEMPERATURE_ESTIMATION_H */
