- select an available timer (ex: TIM11)
	- mode: Activated (Channel1 Disabled)
	- set parameters in order to match the fastest note (PRE=1050-1 Counter=10000-1)
	- in NVIC (Nested Vectored Interrupt Control) settings: enable "TIM1 trigger and communication interrupts and TIM11 global interrupt"

- in main in "USER CODE 2" space start the timer with:  HAL_TIM_Base_Start_IT(&htim11);
- create 2 global variables 'i' and 'tempo':
	- i: index of the note in the song that is being played
	- tempo: the portion of the note that we already sang
	
- create function in order to start playing a note (disable channel - set channel - re-enable channel): 
	void playNextNote(NotePentagram note) {
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		setChannel(note.note.period, note.note.period / 2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	}

- create callback function:
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		// Check which version of the timer triggered this callback
		if (htim == &htim11) {
			// if song finished stop speaker and return immediately
			if (i > sizeof(song) / sizeof(song[0])) {
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				return;
			}

			// if song just started, play first note
			if (i == 0 && tempo == 1) {
				playNextNote(song[i]);
			}

			// increment the tempo; if it's greater than the note duration go to the next note
			tempo++;
			if (tempo > song[i].duration) {
				tempo = 1;
				i++;
				playNextNote(song[i]);
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // for debugging purposes
			}
		}
	}