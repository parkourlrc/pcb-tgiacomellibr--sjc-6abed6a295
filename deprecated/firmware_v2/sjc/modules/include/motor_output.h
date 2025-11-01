/**
 * @file motor_output.h
 * @brief Controla o sinal digital de habilitação da saída de potência.
 *
 * Autor: Tiago
 * Data: Set 9, 2023
 */

#ifndef SJC__MODULES__MOTOR_OUTPUT_H_
#define SJC__MODULES__MOTOR_OUTPUT_H_

#include "system_types/include/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Inicializa a interface de controle de saída de potência.
     */
    void MotorOutput_init(void);

    /**
     * @brief Habilita a saída de potência (ativa o sinal digital).
     */
    void MotorOutput_enable(void);

    /**
     * @brief Desabilita a saída de potência.
     */
    void MotorOutput_disable(void);

    /**
     * @brief Retorna o estado atual da saída (habilitada ou não).
     *
     * @param state Ponteiro onde será armazenado o estado.
     * @return SJC_RET_OK se bem-sucedido, SJC_RET_NOK se ponteiro for nulo.
     */
    eSjcReturnType_t MotorOutput_getState(bool *state);

#ifdef __cplusplus
}
#endif

#endif /* SJC__MODULES__MOTOR_OUTPUT_H_ */
