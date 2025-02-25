/* rasp_communicator.h
 *==============================================================================
 * Author: Aaiza A. Khan and Shruthi P. Kunnon
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Aaiza A. Khan
 * Description: Communicator between STM32 and Raspberry pi.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef SRC_USER_CODE_RASPBERRYPI_COMMUNICATOR_RASP_COMMUNICATOR_H_
#define SRC_USER_CODE_RASPBERRYPI_COMMUNICATOR_RASP_COMMUNICATOR_H_

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include <cstdint>

namespace stm32_code {
namespace raspberrypi_communicator {

class RaspCommunicator {
public:
	explicit RaspCommunicator(UART_HandleTypeDef *huart5);
	void TransmitData();
	void TransmitData(UART_HandleTypeDef *huart);
	void ReceiveData();
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

private:
	UART_HandleTypeDef *huart5_;
	uint8_t tx_buff[10] = { 65, 66, 67, 68, 69, 70, 71, 72, 73, 74 }; // ABCDEFGHIJ in ASCII
	uint8_t rx_buff[10]; // Buffer for receiving data
};
} /* namespace raspberrypi_communicator */
} /* namespace stm32_code */

#endif /* SRC_USER_CODE_RASPBERRYPI_COMMUNICATOR_RASP_COMMUNICATOR_H_ */
