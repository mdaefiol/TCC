// MAQUINA DE ESTADOS DO FOGUETE
switch (current_state){
case PAUSADO:
	if (pin_state1 == GPIO_PIN_RESET && pin_state2 == GPIO_PIN_RESET && pin_button == GPIO_PIN_SET ){
		current_state = AGUARDANDO_LANCAMENTO;
		FRAM_state = FRAM_IDLE;
																// ***** JA FOI INICIADA A AQUISIÇÃO DE DADOS *****
																// CARGAS ESTÃO CONECTADAS, ENTRE EM ESTADO DE AGUARDANDO LANÇAMENTO
	}
	else {
		current_state = PAUSADO;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 	// LED VERMELHO para indicar PAUSA
	}
break;
case AGUARDANDO_LANCAMENTO:
	if (EstadoAceleracao == ACCEL_HIGH_POSITIVE){
		current_state = LANCADO;
	}
	else if (EstadoAceleracao == ACCEL_NEAR_ZERO && EstadoAltitude == ESTADO_ESTACIONARIO){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 	// DESLIGA LED VERMELHO
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 	// BUZZER INDICA AGUARDANDO LANÇAMENTO
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); 	// LED AMARELO INDICAÇÃO AGUARDANDO LANÇAMENTO
		current_state = AGUARDANDO_LANCAMENTO;
	}
break;
case LANCADO:
	if (EstadoAltitude == ESTADO_SUBIDA && EstadoAceleracao == ACCEL_HIGH_POSITIVE){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// DESLIGA BUZZER
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	// DESLIGA LED AMARELO
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); 		// LIGA LED VERDE DE LANÇADO
		current_state = LANCADO;
	}
	else if(EstadoAltitude == ESTADO_SUBIDA && EstadoAceleracao == ACCEL_LOW_NEGATIVE){
		current_state = DETECCAO_APOGEU ;
	}

	break;
	case DETECCAO_APOGEU:
		if (EstadoAceleracao == ACCEL_NEAR_G && EstadoAltitude == ESTADO_DESCIDA ){
			if(data_vehicle.pressMIN > data_vehicle.pressao)
				data_vehicle.pressMIN =  data_vehicle.pressao ;
			current_state = DETECCAO_APOGEU;
		}
		else if ((EstadoAceleracao == ACCEL_LOW_NEGATIVE || EstadoAceleracao == ACCEL_HIGH_NEGATIVE) && EstadoAltitude ==  ESTADO_DESCIDA){
			current_state = PARAQUEDAS_ACIONADO;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // ACIONA CARGA 1 = 1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // ACIONA CARGA 2 = 1; paraquedas on
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); // DESLIGA O LED VERDE
		}
		break;
	case PARAQUEDAS_ACIONADO:

		break;
default:
		break;
}