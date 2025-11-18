/*
 * Copyright (C) 2013-2022 by LeitDrive. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#ifndef SRC_BASIC_OBSERVER_DERIVATIVE_OBSERVER_H_
#define SRC_BASIC_OBSERVER_DERIVATIVE_OBSERVER_H_

#include <math_f.h>

/********************************
 * Type-definitions
 *********************************/
/*!
 * \brief Derivative observer dependencies structure.
 */
typedef struct derivative_obs_deps_s {
    const float *timebase; /*!< Task timebase */
} derivative_obs_deps_t;

/*!
 * \brief Derivative observer config structure.
 */
typedef struct derivative_obs_config_s {
    const float *f_nom;       /*!< Nominal frequency used for time normalization. t_nom = 1/f_nom. */
    const float *l_x;         /*!< Observer gain for state estimation */
    const float *l_x_dot;     /*!< Observer gain for first state derivative estimation */
    const float *l_x_dot_dot; /*!< Observer gain for second state derivative estimation */
} derivative_obs_config_t;

/*!
 * \brief Derivative observer structure.
 */
typedef struct derivative_obs_s {
    derivative_obs_config_t config;
    derivative_obs_deps_t   dependencies;
    float                   x1;            /*!< Internal state of integrator 1 */
    float                   x2;            /*!< Internal state of integrator 2 */
    float                   x_hat;         /*!< Estimated input */
    float                   x_dot_hat;     /*!< Estimated derivative of input */
    float                   x_dot_dot_hat; /*!< Estimated second derivative of input */
} derivative_obs_t;

/*!
 * \brief Derivative observer output structure.
 */
typedef struct derivative_obs_out_s {
    float dot_hat;     /*!< Estimated derivative of input */
    float dot_dot_hat; /*!< Estimated second derivative of input */
} derivative_obs_out_t;

/********************************
 * Function declarations
 *********************************/
/*!
 * \brief Initialize a derivative observer object.
 * \param observer			Pointer to a derivative observer object structure.
 * \param dependencies		Pointer to Dependencies-Structure that holds parameter dependent from other modules.
 * \param config			Pointer to Configuration-Structure that holds configuration parameters.
 *
 * This function is used to initialize a rate observer object.
 */
void DerivativeObserver_Init(
    derivative_obs_t *const observer, const derivative_obs_deps_t *const dependencies,
    const derivative_obs_config_t *const config
);

/*!
 * \brief Reset a derivative observer object.
 * \param observer			Pointer to a derivative observer object structure.
 *
 * This function is used to reset a derivative observer object.
 */
void DerivativeObserver_Reset(derivative_obs_t *const observer);

/*!
 * \brief Reset a derivative observer object.
 * \param observer			Pointer to a derivative observer object structure.
 * \param input				Input signal
 *
 * This function is used to process a derivative observer object.
 * This module estimates the first (dot_hat) and second (dot_dot_hat) time derivative of the given input signal.
 * Attention: Due to numerical effects, it is recommended to keep the input range between -2 and 2.
 * 			  Otherwise the second derivative can have a higher noise level.
 */
derivative_obs_out_t DerivativeObserver_Process(derivative_obs_t *const observer, const float input);

#endif /* SRC_BASIC_OBSERVER_DERIVATIVE_OBSERVER_H_ */
