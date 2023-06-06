     // Verifica a tendência da aceleraçao
	 data_vehicle.accel = simulated_data.accel_z;
	 data_vehicle.accelTemp = data_vehicle.accel;

     diffAceleracao = data_vehicle.accel - data_vehicle.accel_anterior;

     if (data_vehicle.accel > -0.1 && data_vehicle.accel < 0.1) {
         estadoAceleracao = ACCEL_NEAR_ZERO;
     } else if (data_vehicle.accel > -9.5 && data_vehicle.accel < -10.5) {
         estadoAceleracao = ACCEL_NEAR_G;
     } else if (diffAceleracao > 0.5) {
         estadoAceleracao = ACCEL_HIGH_POSITIVE;
     } else if (diffAceleracao < -0.5) {
         estadoAceleracao = ACCEL_HIGH_NEGATIVE;
     } else {
         estadoAceleracao = ACCEL_LOW_NEGATIVE;
     }

     
     // Verifica a tendência da altitude
	 data_vehicle.altitude = simulated_data.pressao;
	 data_vehicle.altTemp = data_vehicle.altitude;

     if (altitudeAtual > altitudeAnterior){
         contSubida++;
         contDescida = 0;
     } else if (altitudeAtual < altitudeAnterior) {
         contDescida++;
         contSubida = 0;
     } else {
         contSubida = 0;
         contDescida = 0;
     }

     switch (estadoAltitude) {
         case ESTADO_INICIAL:
             if (contSubida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_SUBIDA;
             } else if (contDescida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_DESCIDA;
             } else if (contSubida == 0 && contDescida == 0) {
                 estadoAltitude = ESTADO_ESTACIONARIO;
             }
             break;

         case ESTADO_SUBIDA:
             if (contDescida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_DESCIDA;
             } else if (contSubida == 0 && contDescida == 0) {
            	 estadoAltitude = ESTADO_ESTACIONARIO;
             }
             break;

         case ESTADO_DESCIDA:
             if (contSubida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_SUBIDA;
             } else if (contSubida == 0 && contDescida == 0) {
                 estadoAltitude = ESTADO_ESTACIONARIO;
             }
             break;

         case ESTADO_ESTACIONARIO:
             if (contSubida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_SUBIDA;
             } else if (contDescida >= NUM_AMOSTRAS) {
                 estadoAltitude = ESTADO_DESCIDA;
             }
             break;
     }

	  data_vehicle.altitude_anterior = data_vehicle.altTemp ;
	  data_vehicle.accel_anterior = data_vehicle.accelTemp;


	  // MAQUINA DE ESTADOS DO FOGUETE
	switch (current_state){
		case PAUSADO:
			if (pin_state1 == GPIO_PIN_RESET && pin_state2 == GPIO_PIN_RESET && pin_button == GPIO_PIN_SET ){
				current_state = AGUARDANDO_LANCAMENTO;
	        	FRAM_state = FRAM_READ_ID;								// LEITURA DE ID PARA HABILITAR FUNCIONAMENTO FRAM
	          															// ***** JA FOI INICIADA A AQUISIÇÃO DE DADOS *****
	        															// CARGAS ESTÃO CONECTADAS, ENTRE EM ESTADO DE AGUARDANDO LANÇAMENTO
			}
			else {
				current_state = PAUSADO;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 	// LED VERMELHO para indicar PAUSA
			}
		break;

		case AGUARDANDO_LANCAMENTO:
			if (estadoAceleracao == ACCEL_HIGH_POSITIVE){
			// adicionar a verificaçao do ID no IF
			// *************** ADICIONAR GRAVAÇAO ****************
	        // gravaçao de dados pela FRAM
				current_state = LANCADO;
			}
			else if (estadoAceleracao == ACCEL_NEAR_ZERO && estadoAltitude == ESTADO_ESTACIONARIO){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 	// BUZZER INDICA AGUARDANDO LANÇAMENTO
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); 	// LED AMARELO INDICAÇÃO AGUARDANDO LANÇAMENTO
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 	// DESLIGA LED VERMELHO
				current_state = AGUARDANDO_LANCAMENTO;
			}
		break;

		case LANCADO:
			if (estadoAltitude == ESTADO_SUBIDA && estadoAceleracao == ACCEL_HIGH_POSITIVE){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// DESLIGA BUZZER
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	// DESLIGA LED AMARELO
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); 		// LIGA LED VERDE DE LANÇADO
	          	current_state = VOANDO_ACELERADO;
			}
			break;

	        case VOANDO_ACELERADO:
	          if (estadoAceleracao == ACCEL_NEAR_G) {
	        	  current_state = DETECCAO_APOGEU;
	          }
	          else if(estadoAceleracao == ACCEL_HIGH_POSITIVE && estadoAltitude == ESTADO_SUBIDA){
	        	  current_state = VOANDO_ACELERADO;
	          }
	          break;


	     case DETECCAO_APOGEU:
	          if (estadoAceleracao == ACCEL_NEAR_G && estadoAltitude == ESTADO_ESTACIONARIO ){
	        	  data_vehicle.altMAX =  data_vehicle.altitude ;
	        	  current_state = DETECCAO_APOGEU;
	          }
	          else if (estadoAceleracao == ACCEL_LOW_NEGATIVE ||  estadoAceleracao == ACCEL_HIGH_NEGATIVE ){
	        	  current_state = PARAQUEDAS_ACIONADO;
	          }
	         break;


	     case PARAQUEDAS_ACIONADO:
	    	 if (estadoAceleracao == ACCEL_HIGH_NEGATIVE && estadoAltitude = ESTADO_DESCIDA)
	    	 {
	    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); 	// DESLIGA O LED VERDE
	    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // ACIONA CARGA 1 = 1;
	    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // ACIONA CARGA 2 = 1; paraquedas on
	    		 current_state = VOANDO_RETARDADO; 
	    	 }
	    	 else if(estadoAceleracao == ACCEL_LOW_NEGATIVE){
	        	  	  
	    	 }
	    	 else if (data_vehicle.altitude_atual == data_vehicle.altitude_calibracao && data_vehicle.aceleracao_atual == data_vehicle.aceleracao_calibracao && pin_state1 == GPIO_PIN_SET &&  pin_state2 == GPIO_PIN_SET)
	    	 {
	    		 current_state = PAUSADO;
	          	// finaliza gravaçao de dados pela FRAM
	    		 return 0;
	          }
	         break;
	  	default:
	  		break;
	  }