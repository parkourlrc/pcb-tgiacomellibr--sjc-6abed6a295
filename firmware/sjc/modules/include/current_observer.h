#ifndef SJC__MODULES__CURRENT_OBSERVER_H_
#define SJC__MODULES__CURRENT_OBSERVER_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "system_types/include/types.h"

#include <stdbool.h>

typedef struct
{
    float A[2][2];
    float B[2];
    float C[2];
    float L[2];
    float x_hat[2];
    bool initialized;
} CurrentObserver_t;

/**
 * @brief Inicializa o observador de corrente com os parâmetros fornecidos.
 */
void CurrentObserver_init(CurrentObserver_t *observer, const float A[2][2], const float B[2], const float C[2], const float L[2]);

/**
 * @brief Atualiza o estado estimado com base na entrada (u) e saída medida (y_meas).
 */
void CurrentObserver_update(CurrentObserver_t *observer, float u, float y_meas);

/**
 * @brief Retorna a estimativa da corrente.
 */
static inline float CurrentObserver_get_current(const CurrentObserver_t *observer)
{
    return observer->x_hat[0];
}

/**
 * @brief Retorna a estimativa da derivada da corrente.
 */
static inline float CurrentObserver_get_current_derivative(const CurrentObserver_t *observer)
{
    return observer->x_hat[1];
}

#ifdef __cplusplus
}
#endif

#endif // SJC__MODULES__CURRENT_OBSERVER_H_
