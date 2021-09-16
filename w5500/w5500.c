#include "w5500.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"
		uint8_t status_reg = 0; 
typedef enum{
							TCP_SOCK,
							UDP_SOCK
}socket_t;
uint8_t msg_for_wiznet[21]; 
extern InitStructW5500_t Settings;
HAL_StatusTypeDef InitW5500(InitStructW5500_t Settings);
HAL_StatusTypeDef OpenSocket(uint8_t num_sock);
HAL_StatusTypeDef CloseSocket(uint8_t num_sock);
HAL_StatusTypeDef SetTypeSocket(uint8_t num_sock, uint8_t type_socket);
HAL_StatusTypeDef CreateMsg(uint8_t* msg_for_wiznet);
HAL_StatusTypeDef Reset_SW_W5500(void);
extern SPI_HandleTypeDef hspi2;

HAL_StatusTypeDef Reset_SW_W5500(void)
{
		HAL_StatusTypeDef result;
	
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
	
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
	
		offset_1_byte = MODE_REG >> 8;
		offset_2_byte = MODE_REG;
		
		bsb = WSCR << 3| WRITE_CHIP | OM_VDLM;
		
		uint8_t data_to_spi[4]   = {
																 offset_1_byte,offset_2_byte,bsb,RESET_CHIP
																};
		
		result =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);
																 
		return result;
}

HAL_StatusTypeDef InitW5500(InitStructW5500_t Settings)
{
		HAL_StatusTypeDef result;
		
		//Reset_SW_W5500();
	
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
	
		offset_1_byte = GAR0 >> 8;
		offset_2_byte = GAR0;
		
		bsb = WSCR << 3| WRITE_CHIP | OM_VDLM;
		
		uint8_t data_to_spi[21]   = {
																 offset_1_byte,offset_2_byte,bsb,
			
																 Settings.gway[0],Settings.gway[1],Settings.gway[2],Settings.gway[3],
			
																 Settings.mask_subnet[0],Settings.mask_subnet[1],Settings.mask_subnet[2],Settings.mask_subnet[3],
			
																 Settings.mac_addr[0],Settings.mac_addr[1],Settings.mac_addr[2],Settings.mac_addr[3],Settings.mac_addr[4],Settings.mac_addr[5],
			
																 Settings.ip_addr[0],Settings.ip_addr[1],Settings.ip_addr[2],Settings.ip_addr[3]
																};
		
		result =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);
		
    HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);																
		
	  return result;
}

HAL_StatusTypeDef OpenSocket(uint8_t num_sock)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		offset_1_byte = SOCK_x_CR >> 8;
		offset_2_byte = SOCK_x_CR;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_1;
		
		uint8_t data_to_spi[4]   = {
																offset_1_byte,offset_2_byte,bsb,OPEN_SOCKET
																};
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);					

																
		/*проверим состояние сокета*/
		offset_1_byte = SOCK_x_SR >> 8;
		offset_2_byte = SOCK_x_SR;
		/*получить количество свободного места в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| READ_CHIP | OM_DLM_1;
		
		data_to_spi[0]   = offset_1_byte;
		data_to_spi[1]   = offset_2_byte;
		data_to_spi[2]   = bsb;

		//state_msg = HAL_SPI_TransmitReceive(&hspi2,data_to_spi,&status_reg,sizeof(data_to_spi),100); 
	  state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);		
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);																
		
		return state_msg;
}

HAL_StatusTypeDef CloseSocket(uint8_t num_sock)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		offset_1_byte = SOCK_x_CR >> 8;
		offset_2_byte = SOCK_x_CR;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_1;
		
		uint8_t data_to_spi[4]   = {
																offset_1_byte,offset_2_byte,bsb,CLOSE_SOCKET
																};
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);														
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);																
		
		return state_msg;	
}

HAL_StatusTypeDef SetTypeSocket(uint8_t num_sock, uint8_t type_socket)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		offset_1_byte = SOCK_x_MR >> 8;
		offset_2_byte = SOCK_x_MR;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_1;
		
		uint8_t data_to_spi[4]   = {
																offset_1_byte,offset_2_byte,bsb,type_socket
															 };
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);														
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);															 
		
		return state_msg;		
}

HAL_StatusTypeDef SetSourcePort(uint16_t port,uint8_t num_sock)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		uint8_t port_h = 0;
		uint8_t port_l = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		port_h = port >> 8;
		port_l = port;
		
		offset_1_byte = SOCK_x_PORT0 >> 8;
		offset_2_byte = SOCK_x_PORT0;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_2;
		
		uint8_t data_to_spi[5]   = {
																offset_1_byte,offset_2_byte,bsb,port_h,port_l
															 };
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);														
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);															 
		
    return state_msg;	
}

HAL_StatusTypeDef SetIPDest(uint8_t ip[],uint8_t num_sock)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		offset_1_byte = SOCK_x_DIPR0 >> 8;
		offset_2_byte = SOCK_x_DIPR0;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_4;
		
		uint8_t data_to_spi[7]   = {
																offset_1_byte,offset_2_byte,bsb,ip[0],ip[1],ip[2],ip[3]
															 };
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);														
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);															 
		
    return state_msg;		
}

HAL_StatusTypeDef SetPortDest(uint16_t port,uint8_t num_sock)
{
		HAL_StatusTypeDef state_msg;
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;
		uint8_t port_l = 0;
		uint8_t port_h = 0;
		 
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		port_h = port;
		port_l = port >> 8;
		
		offset_1_byte = SOCK_x_DPORT0 >> 8;
		offset_2_byte = SOCK_x_DPORT0;
		
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_2;
		
		uint8_t data_to_spi[5]   = {
																offset_1_byte,offset_2_byte,bsb,port_l,port_h
															 };
		
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_spi,sizeof(data_to_spi),100);														
		
		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);
		
    return state_msg;
}

HAL_StatusTypeDef SendDataSocket(uint8_t data[],uint8_t num_sock)
{  
		HAL_StatusTypeDef state_msg;
		uint8_t freesize[2] = {1,2};
		uint8_t ptr_tx[2] = {1,2};
		uint16_t bsb_sockets[8] = {SS0R,SS1R,SS2R,SS3R,SS4R,SS5R,SS6R,SS7R};
		uint8_t offset_1_byte = 0;
		uint8_t offset_2_byte = 0;
		uint8_t bsb = 0;

		HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_RESET);
		
		offset_1_byte = SOCK_x_TX_FSR0 >> 8;
		offset_2_byte = SOCK_x_TX_FSR0;
		/*получить количество свободного места в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| READ_CHIP | OM_DLM_2;
		
		uint8_t data_to_spi[5]   = {
																offset_1_byte,offset_2_byte,bsb,0,0
															 };
		
		state_msg = HAL_SPI_TransmitReceive(&hspi2,data_to_spi,freesize,sizeof(data_to_spi),100); 
		/*запишем данные в буфер Tx FIFO*/


	
		offset_1_byte = SOCK_x_TX_RD0 >> 8;
		offset_2_byte = SOCK_x_TX_RD0;
		/*получить указатель на смещение в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| READ_CHIP | OM_DLM_2;
		
		uint8_t a_ptr_tx[5]   = {
																offset_1_byte,offset_2_byte,bsb,0,0
															 };

		state_msg = HAL_SPI_TransmitReceive(&hspi2,a_ptr_tx,ptr_tx,sizeof(a_ptr_tx),100); 
		
														 
		uint8_t data_to_out[13] ;//SS1TxB
	  
		offset_1_byte = 0x0000 >> 8;
		offset_2_byte = 0x00;
		
		/*получить количество свободного места в буфере Tx FIFO*/ 
		bsb = SS1TxB << 3| WRITE_CHIP | OM_VDLM;
															 
		data_to_out[0] = offset_1_byte;
		data_to_out[1] = offset_2_byte;
		data_to_out[2] = bsb;															 

		for(int i = 3 ; i < 13; i++)
		{
			data_to_out[i] =  data[i - 3];
		}															 
		
		state_msg =  HAL_SPI_Transmit(&hspi2,data_to_out,sizeof(data_to_out),100);
	
	  /*запишем количество данных в буфер Tx FIFO*/
		
		offset_1_byte = SOCK_x_TX_WR0 >> 8;
		offset_2_byte = SOCK_x_TX_WR0;
		/*получить указатель на смещение в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_2;
		
		uint8_t wr_ptr_tx[5]   = {
																offset_1_byte,offset_2_byte,bsb,0x00,0x0A
															 };

		state_msg = HAL_SPI_Transmit(&hspi2,wr_ptr_tx,sizeof(wr_ptr_tx),100); 


		offset_1_byte = SOCK_x_TX_WR0 >> 8;
		offset_2_byte = SOCK_x_TX_WR0;
		/*получить указатель на смещение в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| READ_CHIP | OM_DLM_2;
		
		uint8_t nwr_ptr_tx[5]   = {
																offset_1_byte,offset_2_byte,bsb,0xff,0xff
															 };

		//state_msg = HAL_SPI_Transmit(&hspi2,wr_ptr_tx,sizeof(wr_ptr_tx),100); 
		state_msg = HAL_SPI_TransmitReceive(&hspi2,nwr_ptr_tx,ptr_tx,sizeof(nwr_ptr_tx),100); 

    /*установим бит SEND*/

		offset_1_byte = SOCK_x_CR >> 8;
		offset_2_byte = SOCK_x_CR;
		/*получить количество свободного места в буфере Tx FIFO*/ 
		bsb = bsb_sockets[num_sock] << 3| WRITE_CHIP | OM_DLM_1;
		
		uint8_t data_send[4]   = {
																offset_1_byte,offset_2_byte,bsb,SOCKET_SEND
															 };
		
		state_msg = HAL_SPI_Transmit(&hspi2,data_send,sizeof(data_send),100); 
HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);															 
		//HAL_GPIO_WritePin(W5500_SCS_GPIO_Port,W5500_SCS_Pin,GPIO_PIN_SET);						
		return state_msg;
	
}
