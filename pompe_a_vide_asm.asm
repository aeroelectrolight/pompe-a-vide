;************************************************************************
;* Fichier : pompe_a_vide.asm						                	*
;* Titre: 	 POMPE A VIDE   											*
;* Version: 1.0					                	                    *
;* Auteur: ludomercet                 				                	*
;* Date: 	28/02/2011			        	                			*
;* Date: ok le 06/04/2011 v1.0											*
;* Date: ok le 07/04/2011 v1.1 ajout d'un ecran demarrage				*
;* date: ok le 13/11/2011 v1.5 ajout d'un réglage d'hystérésis			*
;* 																		*
;* date: 																*
;*								                                    	*	
;* 																		*
;*   																	*
;* 	  																	*
;*									                                    *
;************************************************************************


.include "m8def.inc"	; nom du fichiers de références des registres


;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Déclaration des Constantes		                 	
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

.equ lcd_port 	= portd
.equ retro		= 3
.equ ventile	= 1
.equ vide		= 0
.equ chauffe	= 2

;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Déclaration des Variables		                 	
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

.def seconde = R1
.def minute = R2
.def heure = R3
.def affich = R4
.def rebond = R5
.def SAUVREG = R16
.def a = r17
.def b = r18
.def c = r19
.def d = r20
.def res0 = r19
.def res1 = r20
.def res2 = r21
.def res3 = r22
.def addr = r23

.def	mc16uL	=r17		;multiplicand low byte ->	a
.def	mc16uH	=r18		;multiplicand high byte		b
.def	mp16uL	=r19		;multiplier low byte ->		c
.def	mp16uH	=r20		;multiplier high byte		d
.def	m16u0	=r19		;result byte 0 (LSB) ->		res0
.def	m16u1	=r20		;result byte 1				res1
.def	m16u2	=r21		;result byte 2		->		res2
.def	m16u3	=r22		;result byte 3 (MSB)		res3

.def	drem16uL=r21		; reste -> 	res2
.def	drem16uH=r22		;			res3
.def	dres16uL=r17		; resultat -> 	a
.def	dres16uH=r18		;				b		
.def	dd16uL	=r17		; dividende ->	a
.def	dd16uH	=r18		;				b
.def	dv16uL	=r19		; diviseur ->	c
.def	dv16uH	=r20		;				d

.def sub1l = r17
.def sub1h = r18
.def sub2l = r29
.def sub2h = r20

;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;MACRO	                 	
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

.MACRO	validation

		rcall delay1ms
		sbi lcd_port,7
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		cbi lcd_port,7

.ENDMACRO

;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Déclaration des zones de variables dans la sram
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

.DSEG
	aff_heure: 	.BYTE 6		; heure en ASCII
	data:		.BYTE 8		; valeur de pression, température et hystérésis en ASCII reel
	valeur:		.BYTE 3		; valeur de pression, température et hystérésis reel en binaire
	data_cons:	.BYTE 8		; valeur de pression, température et hystérésis en ASCII de consigne

;.ORG 0x0060

;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Déclaration des Vecteurs d'interruptions
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

 
.CSEG

.ORG 0x000 rjmp RESET ; Reset Handler
;$001 rjmp EXT_INT0 ; IRQ0 Handler
;$002 rjmp EXT_INT1 ; IRQ1 Handler
;$003 rjmp TIM2_COMP ; Timer2 Compare Handler
;$004 rjmp TIM2_OVF ; Timer2 Overflow Handler
;$005 rjmp TIM1_CAPT ; Timer1 Capture Handler
;$006 rjmp TIM1_COMPA ; Timer1 CompareA Handler
;$007 rjmp TIM1_COMPB ; Timer1 CompareB Handler
.ORG 0x008 rjmp TIM1_OVF ; Timer1 Overflow Handler
.ORG 0x009 rjmp TIM0_OVF ; Timer0 Overflow Handler
;$00a rjmp SPI_STC ; SPI Transfer Complete Handler
;$00b rjmp USART_RXC ; USART RX Complete Handler
;$00c rjmp USART_UDRE ; UDR Empty Handler
;$00d rjmp USART_TXC ; USART TX Complete Handler
;$00e rjmp ADC ; ADC Conversion Complete Handler
;$00f rjmp EE_RDY ; EEPROM Ready Handler
.org 0x010 rjmp ANA_COMP ; Analog Comparator Handler
;$011 rjmp TWSI ; Two-wire Serial Interface Handler
;$012 rjmp SPM_RDY ; Store Program Memory Ready Handler
;

	
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Routines de traitement des interruptions
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

ANA_COMP:		reti

;------------------------------
; INTERRUPTION POUR ANTI REBOND
;------------------------------

TIM0_OVF:		push SAUVREG	; sauvegarde sur la pile
				push a

				dec rebond 
				brne out_timer0
				in a,timsk
				cbr a,0x01
				out timsk,a

out_timer0:
			pop a
			pop SAUVREG



				reti

;----------------------------------------------
; INTERRUPTION TOUTES LES SECONDES POUR L'HEURE
;----------------------------------------------

TIM1_OVF: 	push SAUVREG
			push a
			push b
			
			tst seconde		; on test si les seconde sont à zero 
			brne second1
			tst minute		; test si c'est la findu temps
			brne second2
			tst heure
			breq out_fin_timer1	; 
second2:	ldi a,60
			mov seconde,a
second1:	dec seconde
			mov a,seconde
			rcall bin_bcd	; convertion 
			ldi xl,low(aff_heure)
			ldi xh,high(aff_heure)
			adiw XH:XL,4
			rcall conv_sauvH ; sauvegarde pour affichage
			ldi a,59
			cpse seconde,a	
			rjmp out_timer1

			tst minute		; on est à 0
			brne minu
			ldi a,60
			mov minute,a
minu:		dec minute 
			mov a,minute
			rcall bin_bcd	; convertion 
			subi XL,4
			rcall conv_sauvH ; sauvegarde pour affichage
			ldi a,59
			cpse minute,a	
			rjmp out_timer1

			tst heure		; on est à 0
			brne heur
			ldi a,60
			mov heure,a
heur:		dec heure
			mov a,heure
			rcall bin_bcd	; convertion 
			subi XL,4
			rcall conv_sauvH ; sauvegarde pour affichage
			ldi a,59
			cpse heure,a	
			rjmp out_timer1


out_fin_timer1:	in a,TIMSK
				cbr a,0b00000100		; toie1 à 0
				out TIMSK,a
				cbi portb,vide
				cbi portb,chauffe
				cbi portb,ventile
				
			
out_timer1:	ldi a,0xc2
			out tcnt1h,a	; interruption toutes les secondes
			ldi a,0xf8
			out tcnt1l,a
			pop b
			pop a
			pop SAUVREG

			reti
			


;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Programme de RESET
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


RESET:		ldi	a,low(RAMEND)
			out	SPL,a		; Initialisation de la pile à l'état bas
			ldi a,high(RAMEND)
			out SPH,a

; initialisation des port 
;-------------------------  
/*
	PORTB 	PB0 = out étuve
			PB1 = out ventil
			PB2 = out chauff
			PB3 = out retroéclairage LCD
			PB4 = in bouton +
			PB5 = in
			PB6 = in
			PB7 = in
			
	PORTC 	PC0 = in ADC pression MPX2200
			PC1 = in ADC température étuve lm75
			PC2 = in ADC température chauffage résistance 
			PC3 = in bouton -
			PC4 = in bouton set
			PC5 = in bouton on

	PORTD 	PD0 = out lcd
			PD1 = out lcd
			PD2 = out lcd
			PD3 = 
			PD4 = out lcd
			PD5 = out lcd
			PD6 = out lcd
			PD7 = out lcd enable
*/ 
		    ldi a,0xf0
			out portb,a
			ldi a,0x0f		; initialisation portB
			out ddrb,a
			clr a
			out ddrc,a
			ldi a,0xf8
			out portc,a
			ser a
			out ddrd,a
			clr a
			out portd,a


; initialisation du timer0
;--------------------------

			ldi a,(1<<CS02)|(1<<CS00)	
			out TCCR0,a

; initialisation du timer1
;--------------------------

			ldi a,(1<<CS12)	
			out TCCR1B,a
			ldi a,0x7a
			out tcnt1h,a	; interruption toutes les secondes
			ldi a,0x12
			out tcnt1l,a

; initialisation ADC
;-------------------
			
			ldi a,(1<<ADEN)
			out ADCSRA,a

; initialisation LCD
;--------------------

			ldi a,0x00		; clear lcd
			out lcd_port,a
			validation
			ldi a,0x01		; clear lcd
			out lcd_port,a
			validation
			ldi a,0x00		; clear lcd
			out lcd_port,a
			validation
			ldi a,0x00		; clear lcd
			out lcd_port,a
			validation
			ldi a,0x01		; clear lcd
			out lcd_port,a
			validation				; macro de validation
			ldi a,0x03		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation ;
			ldi a,0x03		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation ;
			ldi a,0x03		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation 
			ldi a,0x02		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation ;
			ldi a,0x02		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation ;
			ldi a,0x08		; mode 8 bit puis 4 bit
			out lcd_port,a
			validation ;
			ldi a,0x00		; selection du décalage curseur id s
			out lcd_port,a
			validation ;
			ldi a,0x0c
			out lcd_port,a
			validation;
			ldi a,0x00		; deplacement vers la gauche
			out lcd_port,a
			validation
			ldi a,0x06
			out lcd_port,a
			validation

;---------------
; initialisation 
; retroéclairage on off

			ldi a,high(retro_led)	; lecture de l'eeprom
			ldi b,low(retro_led)
			rcall eeprom_read
			tst c					; test si la valeur à été initialisé
			breq ini_retro
			ldi c,0x01
ini_retro:	ldi YL,low(data)
			ldi YH,high(data)
			adiw YH:YL,7
			st Y,c
			tst c
			//breq ini_val
			sbi portb,3

;-----------------
; initialisation
; des valeurs heure, dépression, hystérésis

; test d'initialisation des valeur eeprom

ini_heure:	ldi a,high(horloge)		; heure
			ldi b,low(horloge)
			rcall eeprom_read
			mov heure,c
			ldi b,low(horloge+1)
			rcall eeprom_read
			mov minute,c
			ldi b,low(horloge+2)
			rcall eeprom_read
			mov seconde,c
; test
			mov a,heure
			rcall ini_val
			 
			
ini_hyst:	ldi a,high(hysteresis)
			ldi b,low(hysteresis)
			rcall eeprom_read
			//mov hyst

init_heure:	ldi xl,low(aff_heure)		; initialisation valeur ASCII
			ldi xh,high(aff_heure)
			mov a,heure
			rcall bin_bcd
			rcall conv_sauvH
			mov a,minute
			rcall bin_bcd
			rcall conv_sauvH
			mov a,seconde
			rcall bin_bcd
			rcall conv_sauvH

ini_press:	ldi a,high(pression)
			ldi b,low(pression)
			rcall eeprom_read
			ldi YL,low(data)
			ldi YH,high(data)
			st Y,c
			ldi YL,low(data_cons)
			ldi YH,high(data_cons)
			adiw YH:YL,3
			mov a,c
			rcall bin_bcd
			ldi c,0b00110000
			add a,c
			st -Y,a
			mov a,b
			rcall bin_bcd
			add a,c
			add b,c
			st -Y,a
			st -Y,b
				
ini_temp:	ldi a,high(temperature)
			ldi b,low(temperature)
			rcall eeprom_read
			ldi YL,low(data)
			ldi YH,high(data)
			adiw YH:YL,1
			st Y,c
			ldi YL,low(data_cons)
			ldi YH,high(data_cons)
			adiw YH:YL,3
			mov a,c
			rcall bin_bcd
			ldi c,0x30
			add a,c
			add b,c
			st Y+,b
			st Y,a


			sei		; INTERRUPTION ON


;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Programme principal
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

;*****
;INTRO
;*****

intro:	wdr
		ldi ZH,high(2*bonjour)		; chargement bonjour lcd
		ldi ZL,low(2*bonjour)
		ldi b,0x84
		rcall add_ddram
		rcall affich_Ztxt
		rcall delay1s
		rcall clr_lcd
		ldi ZL,low(2*version) 		; chargement version lcd
		ldi ZH,high(2*version)
		clr b
		rcall add_ddram
		rcall affich_Ztxt
		ldi ZL,low(2*nom)			;chargement nom lcd
		ldi ZH,high(2*nom)
		ldi b,0xc3
		rcall add_ddram
		rcall affich_Ztxt
		rcall delay1s
		rcall aquisition
		rcall clr_lcd

;******
; MAIN
;******

;-----------------
; ECRAN PRINCIPAL

aff_A_bt:
			rcall anti_rebond	
			rcall clr_lcd
			rcall curseur_off

		wdr					; affichage de la pression
		ldi a,0
		mov affich,a
		ldi ZL,low(2*press)	; de l'heure et de la température
		ldi ZH,high(2*press)
		clr b 
		rcall add_ddram
		rcall affich_Ztxt
		ldi ZL,low(2*temp)
		ldi ZH,high(2*temp)
		ldi b,0x40
		rcall add_ddram
		rcall affich_Ztxt
aff_A:	ldi YL,low(data)
		ldi YH,high(data)
		ldi b,0x08			; pression instantanée
		rcall add_ddram
		ldi b,3
		rcall affich_Ytxt
		ldi b,0x4D			; température étuve
		rcall add_ddram
		ldi b,2
		rcall affich_Ytxt
		ldi YL,low(aff_heure)
		ldi YH,high(aff_heure)
		ldi b,0x40			; heure
		rcall add_ddram
		ldi b,2
		rcall affich_Ytxt
		ldi b,0x43			; minute
		rcall add_ddram
		ldi b,2
		rcall affich_Ytxt
		ldi b,0x46			; seconde
		rcall add_ddram
		ldi b,2
		rcall affich_Ytxt		
		rcall aquisition	; aquisition de données
		in a,timsk
		sbrc a,toie1		; test si fonctionement de la pompe
		rjmp marche

reb1:	tst rebond
		brne reb1
		sbis pinb,5		; test si bouton + appuyé
		rjmp aff_B_bt
		sbis pinc,3		; test si bouton - appuyé
		rjmp aff_A_bt
		sbis pinc,4		; test si bouton set appuyé
		rjmp aff_C_bt
		sbic pinc,5		; test si bouton valid appuyé
		rjmp aff_A

route:	rcall anti_rebond
		sbi portb,ventile	; mise en route étuve + vide
		sbi portb,chauffe
		sbi portb,vide
		ldi a,0xc2
		out tcnt1h,a	; interruption toutes les secondes
		ldi a,0xf8
		out tcnt1l,a
		in a,timsk			; mise en route compteur
		sbr a,0x04
		out timsk,a
		rcall clr_lcd
		ldi ZH,high(2*demarre)
		ldi ZL,low(2*demarre)
		rcall affich_Ztxt
		rcall delay1s
		rjmp aff_A_bt
		
;---------------------------
; MARCHE DE LA POMPE A VIDE

marche:		ldi XH,high(valeur)			; test si pression superieur 
			ldi XL,low(valeur)			; à la valeur de référence + ou OCIE1B 3 57 - 5
			ldi a,high(pression)
			ldi b,low(pression)
			rcall eeprom_read
			ld a,X+
			ldi b,5						; calcul de l'hysteresis
			add c,b
			cp c,a
			brmi off_press
			subi c,10
			cp c,a	
			brmi etuve
on_press:	sbi portb,vide
			rjmp etuve
off_press:	cbi portb,vide				; arret de la pompe à vide

etuve :									; test température de l'étuve			 
										; à la valeur de référence + ou - 5°
			ldi a,high(temperature)		; si le chauffage est < à 90°
			ldi b,low(temperature)
			rcall eeprom_read
			ld a,X+
			ldi b,2
			;push a
			;push c
			add c,b
			cp c,a
			brmi off_temp
			;pop c
			;pop a
			subi c,3
			cp c,a
			brmi test_chauff
			sbi portb,chauffe

test_chauff:						; test si température du chauffage > 90°
			ld a,X
			ldi b,90
			cp a,b
			brmi test_touche			;non on passe au test des touches


off_temp:	cbi portb,chauffe						; arret ou marche du chauffage

test_touche: 
			tst affich
			brne aff_B

reb2:		tst rebond
			brne reb2
			sbis pinb,5		; test si bouton + appuyé
			rjmp aff_B_bt
			sbis pinc,3		; test si bouton - appuyé
			rjmp aff_A_bt
			sbic pinc,5		; test si bouton valid appuyé
			rjmp aff_A

test_arret:	rcall anti_rebond
			sbrs a,toie1		; test si fonctionement de la pompe a vide
			rjmp route

;---------------------------
; ARRET DE LA POMPE A VIDE
;
			rcall clr_lcd		; oui on éteint
			ldi ZL,low(2*arret)
			ldi ZH,high(2*arret)
			rcall affich_Ztxt
			ldi ZL,low(2*oui_non)
			ldi ZH,high(2*oui_non)
			ldi b,0x40
			rcall add_ddram
			rcall affich_Ztxt

reb21:	tst rebond
		brne reb21
		sbis pinb,5		; test si bouton + appuyé
		rjmp aff_B_bt
		sbis pinc,3		; test si bouton - appuyé
		rjmp aff_A_bt
		sbis pinc,4		; test si bouton set appuyé
		rjmp aff_A_bt
		sbic pinc,5		; test si bouton valid appuyé
		rjmp reb21
		in a,timsk
		cbr a,0x04
		out timsk,a
		cbi portb,vide
		cbi portb,chauffe
		cbi portb,ventile
		rjmp aff_A_bt

;------------------
; ECRAN PRINCIPAL 2

aff_B_bt:
			rcall anti_rebond	
			rcall clr_lcd

			wdr 
			rcall aquisition
			ldi a,1
			mov affich,a
			ldi ZL,low(2*chauffage)	; de l'heure et de la température
			ldi ZH,high(2*chauffage)
			clr b 
			rcall add_ddram
			rcall affich_Ztxt
			ldi ZL,low(2*eclairage)
			ldi ZH,high(2*eclairage)
			ldi b,0x40
			rcall add_ddram
			rcall affich_Ztxt
aff_B:		ldi YL,low(data)
			ldi YH,high(data)
			adiw YH:YL,5
			ldi b,0x0B
			rcall add_ddram
			ldi b,2
			rcall affich_Ytxt
			ld a,Y
			tst a
			brne aff_on
			ldi ZL,low(2*off)
			ldi ZH,high(2*off)
			ldi b,0x4B
			rcall add_ddram
			rcall affich_Ztxt
			rjmp test_toucheB
aff_on:		ldi ZL,low(2*on)
			ldi ZH,high(2*on)
			ldi b,0x4B
			rcall add_ddram
			rcall affich_Ztxt


test_toucheB: 
			rcall aquisition	; aquisition de données
reb3:		tst rebond
			brne reb3
			sbis pinb,5		; test si bouton + appuyé
			rjmp aff_B_bt
			sbis pinc,3		; test si bouton - appuyé
			rjmp aff_A_bt
			sbis pinc,4		; test si bouton set appuyé
			rjmp ecl_inv
			sbis pinc,5		; test si bouton on appuyé
			rjmp test_arret
			in a,timsk	
			sbrc a,toie1		; test si fonctionement de la pompe
			rjmp marche
			rjmp aff_B


; marche arret du rétroéclairage

ecl_inv:	ldi b,0x01
			ldi YL,low(data)
			ldi YH,high(data)
			adiw YH:YL,7
			ld a,Y
			eor a,b
			st Y,a
			tst a
			brne retro_on
			cbi portb,retro
			mov c,a
			ldi a,high(retro_led)
			ldi b,low(retro_led)
			rcall eeprom_write
			rjmp ecl_inv1

retro_on:	sbi portb,retro
			mov c,a
			ldi a,high(retro_led)
			ldi b,low(retro_led)
			rcall eeprom_write

ecl_inv1:	rcall anti_rebond
			rjmp aff_B

;-------------------------------------
; ECRAN DE SELECTION DE LA TEMPERATURE


aff_C_bt:	rcall anti_rebond	
			rcall clr_lcd
			wdr 

			ldi a,2
			mov affich,a
			ldi ZL,low(2*sel_temp)	; de l'heure et de la température
			ldi ZH,high(2*sel_temp)
			clr b 
			rcall add_ddram
			rcall affich_Ztxt
			ldi ZL,low(2*sel_etuve)
			ldi ZH,high(2*sel_etuve)
			ldi b,0x43
			rcall add_ddram
			rcall affich_Ztxt
			ldi b,0x4a
			rcall add_ddram
			ldi b,2
			ldi XL,low(valeur)
			ldi XH,high(valeur)
			adiw XH:XL,0
			ldi YL,low(data_cons)
			ldi YH,high(data_cons)
			adiw YH:YL,3
			rcall affich_Ytxt
			ldi b,0x4a
			mov addr,b
			subi YL,2
			rcall add_ddram
			rcall curseur_on
			rjmp reb4

aff_C:		mov b,addr
			rcall add_ddram
			ldi b,1	
			rcall affich_Ytxt
			dec YL
			mov b,addr
			rcall add_ddram

reb4:		tst rebond
			brne reb4
			sbis pinb,5		; test si bouton + appuyé
			rjmp inc_temp
			sbis pinc,3		; test si bouton - appuyé
			rjmp dec_temp
			sbis pinc,4		; test si bouton set appuyé
			rjmp inc_addC
			sbis pinc,5		; test si bouton on appuyé
			rjmp aff_D_bt	
			rjmp reb4	

inc_addC:	rcall anti_rebond
			cpi addr,0x4a
			brne dec_addC
			inc YL
			inc addr
			rjmp aff_C	
dec_addC:	dec YL
			dec addr
			rjmp aff_C

inc_temp:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinc,3
			rjmp store_C
			rcall anti_rebond
			ld a,Y
			inc a
			cpi a,0x3a
			brne st_incC
			ldi a,0x30
st_incC:	st Y,a
			rjmp aff_C
			
dec_temp:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinb,5
			rjmp store_C
			rcall anti_rebond
			ld a,Y
			dec a
			cpi a,0x2f
			brne st_decC
			ldi a,0x39
st_decC:	st Y,a
			rjmp aff_C

store_C:	ldi YL,low(data_cons)
			clr c
			adiw YH:YL,3
			ld b,Y+
			subi b,0x30
			ldi a,10
			tst b
			breq st_c1
st_c:		add c,a
			dec b
			brne st_c
st_c1:		ld b,Y
			subi b,0x30
			add c,b
			ldi a,high(temperature)
			ldi b,low(temperature)
			st X,c
			rcall eeprom_write
			rcall curseur_off
			ldi ZH,high(2*enregi)
			ldi Zl,low(2*enregi)
			rcall clr_lcd
			rcall affich_Ztxt
			rcall delay1s
			
;----------------------------------
; ECRAN DE SELECTION DE LA PRESSION

aff_D_bt:	rcall anti_rebond
			rcall curseur_off	
			rcall clr_lcd

			wdr 
			ldi ZL,low(2*sel_dep)	; pression
			ldi ZH,high(2*sel_dep)
			clr b 
			rcall add_ddram
			rcall affich_Ztxt
			ldi ZL,low(2*sel_vide)
			ldi ZH,high(2*sel_vide)
			ldi b,0x41
			rcall add_ddram
			rcall affich_Ztxt
			ldi b,0x48
			rcall add_ddram
			ldi b,3
			ldi XL,low(valeur)
			ldi XH,high(valeur)
			ldi YL,low(data_cons)
			ldi YH,high(data_cons)
			rcall affich_Ytxt
			ldi b,0x48
			mov addr,b
			ldi YL,low(data_cons)
			rcall add_ddram
			rcall curseur_on
			rjmp reb5
			 
aff_D:		mov b,addr
			rcall add_ddram
			ldi b,1	
			rcall affich_Ytxt
			dec YL
			mov b,addr
			rcall add_ddram

reb5:		tst rebond
			brne reb5
			sbis pinb,5		; test si bouton + appuyé
			rjmp inc_press
			sbis pinc,3		; test si bouton - appuyé
			rjmp dec_press
			sbis pinc,4		; test si bouton set appuyé
			rjmp inc_addD
			sbis pinc,5		; test si bouton on appuyé
			rjmp aff_E_bt	
			rjmp reb5

inc_addD:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinc,3
			rjmp store_D
			rcall anti_rebond	; deplacement du curseur de selection vers la droite
			inc YL
			inc addr
			cpi addr,0x4b
			breq dec_addD
			rjmp aff_D	
dec_addD:	ldi YL,low(data_cons)	; si trop, on revient
			ldi addr,0x48
			rjmp aff_D

inc_press:	rcall anti_rebond	; incremente le chiffre avec +
			ld a,Y
			inc a
			cpi a,0x3a
			brne st_incD
			ldi a,0x30
st_incD:	st Y,a
			rjmp aff_D

dec_press:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinb,5
			rjmp store_D
			rcall anti_rebond	; decremente le chiffer avec -
			ld a,Y
			dec a
			cpi a,0x2f
			brne st_decD
			ldi a,0x39
st_decD:	st Y,a
			rjmp aff_D

store_D:	ldi YL,low(data_cons)	; valide : on enregistre la variable
			clr c					; en eeprom,
			ld b,Y+					; ex Y ='2', Y+1 ='5', Y+2 ='5' -> eeprom = 0xFF
			subi b,0x30				; <- on retire la valeur ASCII 0
			ldi a,100
			tst b
			breq st_D1
st_D:		add c,a
			dec b
			brne st_D
			ldi a,10
st_D1:		ld b,Y+
			subi b,0x30
			tst b
			breq st_D2
st_D11:		add c,a
			dec b
			brne st_D11
st_D2:		ld b,Y
			subi b,0x30
			add c,b
			ldi a,high(pression)
			ldi b,low(pression)
			st X,c
			rcall eeprom_write
			rcall curseur_off
			ldi ZH,high(2*enregi)
			ldi Zl,low(2*enregi)
			rcall clr_lcd
			rcall affich_Ztxt
			rcall delay1s

;-----------------------------------
; ECRAN DE SELECTION DE L'HEURE

aff_E_bt:
			rcall anti_rebond
			rcall curseur_off	
			rcall clr_lcd

			wdr 
			ldi ZL,low(2*sel_sel)	; de l'heure et de la température
			ldi ZH,high(2*sel_sel)
			ldi b,0x00
			rcall add_ddram
			rcall affich_Ztxt
			ldi ZL,low(2*sel_heure)
			ldi ZH,high(2*sel_heure)
			ldi b,0x40
			rcall add_ddram
			rcall affich_Ztxt
			ldi addr,0x46
			ldi c,3
			ldi YL,low(aff_heure)
			ldi YH,high(aff_heure)
aff_Eh:		mov b,addr
			rcall add_ddram
			ldi b,2
			add addr,b
			rcall affich_Ytxt
			inc addr
			dec c
			brne aff_Eh
			ldi b,0x46
			mov addr,b
			ldi YL,low(aff_heure)
			rcall add_ddram
			rcall curseur_on
			rjmp reb6
			
aff_E:		mov b,addr
			rcall add_ddram
			ldi b,1	
			rcall affich_Ytxt
			dec YL
			mov b,addr
			rcall add_ddram

reb6:		tst rebond
			brne reb6
			sbis pinb,5		; test si bouton + appuyé
			rjmp inc_heure
			sbis pinc,3		; test si bouton - appuyé
			rjmp dec_heure
			sbis pinc,4		; test si bouton set appuyé
			rjmp inc_addE
			sbis pinc,5		; test si bouton valide appuyé
			rjmp aff_F_bt	
			rjmp reb6

inc_addE:	rcall anti_rebond	; deplacement du curseur de selection vers la droite
			inc YL
inc_addrE:	inc addr
			cpi addr,0x4e
			breq dec_addE	
			cpi addr,0x4b
			breq inc_addrE
			cpi addr,0x48
			breq inc_addrE
			rjmp dec_add_ex
dec_addE:	ldi YL,low(aff_heure)	; si trop, on revient
			ldi addr,0x46
dec_add_ex:	rjmp aff_E

inc_heure:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinc,3
			rjmp store_E
			rcall anti_rebond	; incremente le chiffre avec +
			ldi b,0x3a			
			cpi addr,0x4c		; si on est sur le chiffre de dizaine 
			brne tst_addh		; les chiffre vont de 1 à 6
			ldi b,0x36
tst_addh:	cpi addr,0x49		
			brne inc_nombre
			ldi b,0x36
inc_nombre:	ld a,Y
			inc a
			cp a,b
			brne st_incE
			ldi a,0x30
st_incE:	st Y,a
			rjmp aff_E

dec_heure:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinb,5
			rjmp store_E
			rcall anti_rebond	; decremente le chiffer avec -
			ldi b,0x39			
			cpi addr,0x4c		; si on est sur le chiffre de dizaine 
			brne tst_addh1		; les chiffre vont de 1 à 6
			ldi b,0x35
tst_addh1:	cpi addr,0x49
			brne dec_nombre
			ldi b,0x35
dec_nombre:	ld a,Y
			dec a
			cpi a,0x2f
			brne st_decE
			mov a,b
st_decE:	st Y,a
			rjmp aff_E

store_E:	ldi YL,low(aff_heure)	; valide, on enregistre la variable
			rcall ch_nbr
			ldi a,high(horloge)
			ldi b,low(horloge)
			mov heure,c
			rcall eeprom_write
			rcall ch_nbr
			ldi a,high(horloge)
			ldi b,low(horloge+1)
			mov minute,c
			rcall eeprom_write
			rcall ch_nbr
			ldi a,high(horloge)
			ldi b,low(horloge+2)
			mov seconde,c
			rcall eeprom_write
			rcall curseur_off
			ldi ZH,high(2*enregi)
			ldi Zl,low(2*enregi)
			rcall clr_lcd
			rcall affich_Ztxt
			rcall delay1s

;--------------------------------------------------
; ECRAN DE SELECTION DE L'HYSTERESIS DE 0 A 29 MBAR


aff_F_bt:	rcall anti_rebond	
			rcall clr_lcd
			wdr 

			ldi a,2
			mov affich,a
			ldi ZL,low(2*sel_sel)	; affichage du texte hysteresis
			ldi ZH,high(2*sel_sel)
			clr b 
			rcall add_ddram
			rcall affich_Ztxt
			ldi ZL,low(2*sel_hyst)
			ldi ZH,high(2*sel_hyst)
			ldi b,0x40
			rcall add_ddram
			rcall affich_Ztxt
			ldi b,0x4d
			rcall add_ddram
			ldi b,2
			ldi XL,low(valeur)		; affichage de la valeur hysteresis
			ldi XH,high(valeur)
			adiw XH:XL,6
			ldi YL,low(data_cons)
			ldi YH,high(data_cons)
			adiw YH:YL,6
			rcall affich_Ytxt
			ldi b,0x4d
			mov addr,b
			subi YL,2
			rcall add_ddram
			rcall curseur_on
			rjmp reb7

aff_F:		mov b,addr
			rcall add_ddram
			ldi b,1	
			rcall affich_Ytxt
			dec YL
			mov b,addr
			rcall add_ddram

reb7:		tst rebond
			brne reb7
			sbis pinb,5		; test si bouton + appuyé
			rjmp inc_hyst
			sbis pinc,3		; test si bouton - appuyé
			rjmp dec_hyst
			sbis pinc,4		; test si bouton set appuyé
			rjmp inc_addF
			sbis pinc,5		; test si bouton on/off appuyé
			rjmp aff_A_bt	
			rjmp reb7	

inc_addF:	rcall anti_rebond
			cpi addr,0x4d
			brne dec_addF
			inc YL
			inc addr
			rjmp aff_F	
dec_addF:	dec YL
			dec addr
			rjmp aff_F

inc_hyst:	rcall delay5ms		; si + et - appuyer, on enregistre
			sbis pinc,3
			rjmp store_F
			rcall anti_rebond
			ldi b,0x3a			
			cpi addr,0x4d		; si on est sur le chiffre de dizaine 
			brne inc_n_h		; les chiffre vont de 0 à 1
			ldi b,0x33
inc_n_h:	ld a,Y
			inc a
			cp a,b
			brne st_incF
			ldi a,0x30
st_incF:	st Y,a
			rjmp aff_F
			
dec_hyst:	rcall delay5ms		; - : on décrémente l'affichage
			sbis pinb,5
			rjmp store_F
			rcall anti_rebond
			ldi b,0x39
			cpi addr,0x4d
			brne dec_n_h
			ldi b,0x32
dec_n_h:	ld a,Y
			dec a
			cpi a,0x2f
			brne st_decF
			mov a,b
st_decF:	st Y,a
			rjmp aff_F

store_F:	ldi YL,low(data_cons) ; si + et - on enregistre
			clr c
			adiw YH:YL,6
			ld b,Y+
			subi b,0x30
			ldi a,10
			tst b
			breq st_f1
st_f:		add c,a
			dec b
			brne st_f
st_f1:		ld b,Y
			subi b,0x30
			add c,b
			ldi a,high(hysteresis)
			ldi b,low(hysteresis)
			st X,c
			rcall eeprom_write
			rcall curseur_off
			ldi ZH,high(2*enregi)
			ldi Zl,low(2*enregi)
			rcall clr_lcd
			rcall affich_Ztxt
			rcall delay1s
			rjmp aff_A_bt

;*******
; TEXTE
;*******

bonjour:	.DB "BONJOUR",0x00,0x00
version:	.DB "POMPE A VIDE 1.5",0x00,0x00
nom:		.DB "LUDO MERCET",0x00,0x00
press:		.DB "PRESS= -    mbar",0x00,0x00
temp:		.DB "  :  :    T=   ",0xDF,0x00
chauffage:	.DB "CHAUFFAGE:   ",0xDF,0x00
eclairage:	.DB "ECLAIRAGE: ",0x00,0x00
on:			.DB "ON ",0x00,0x00
off:		.DB "OFF",0x00,0x00
sel_temp:	.DB "SEL TEMPERATURE",0x00
sel_etuve:	.DB "ETUVE:   ",0xDF,0x00
sel_dep:	.DB "SEL DEPRESSION",0x00
sel_vide:	.DB "VIDE: -   mbar",0x00
sel_sel:	.DB "   SELECTION",0x00,0x00
sel_heure:	.DB "HEURE   :  :  ",0x00
sel_hyst:	.DB "HYSTERESIS :   ",0x00,0x00
enregi:		.DB "ENREGISTREMENT",0x00,0x00
demarre:	.DB " DEMARRAGE !!!",0x00
arret:		.DB " ARRET !!!!!!!",0x00,0x00
oui_non:	.DB " VALIDE -> oui ",0x00,0x00



	
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Sous-Routines
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

;--------------------
; INIT VALEUR

ini_val: 	subi a,0x30
			brne ini_suite
			subi a,0x40
			brne ini_suite

ini_suite:	ret
			

;--------------------
; ANTI REBOND

anti_rebond :
		
		ldi a,0x05
		mov rebond,a
		clr a
		out tcnt0,a
		in a,timsk
		sbr a,0x01
		out timsk,a
		ret 

;-------------------
; DELAY 
; tempo de 1 ms
; tempo de 5 ms
; tempo de 1s

delay1s:	ldi b,0xff
			ldi a,0x1f
del3:		push a			
del2:		ldi a,0xaf	
del1:		dec a
			brne del1
			dec b
			brne del2
			pop a
			dec a
			brne del3

			ret

delay5ms:	ldi b,0xff
del4:		ldi a,0xff
del5:		dec a
			brne del5
			dec b
			brne del4
			ret

delay1ms:	push b
			ldi b,0x06
delay11:	ldi a,0xdf
delay1:		dec a
			brne delay1
			dec b
			brne delay11
			pop b
			ret

;--------------------------------
; CONVERTION CHIFFRE VERS NOMBRE
; ex : Y = 1 Y+1 = 3 -> c = 13

ch_nbr:	
			clr c
			ld b,Y+
			subi b,0x30
			ldi a,10
			tst b
			breq st_E1
st_E:		add c,a
			dec b
			brne st_E
st_E1:		ld b,Y+
			subi b,0x30
			add c,b
			ret

;---------------------------------------
; CONVERTION BINAIRE vers BCD
; a doit contenire la valeur à modifier.
; En sortie, a contient les unitées
; b contient le reste
; ex : a = 123 -> a = 3 , b = 12

bin_bcd: 	clr b
conv1:		cpi a,10
			brcs conv2
			subi a,10
			inc b
			rjmp conv1
conv2:		ret

;----------------------------------------------
; CONVERTION ASCII ET SAUVEGARDE DE L'HEURE
; transformation de a et b en ASCII 
; et sauvegarde avec le pointeur X en (affheure) 
 
conv_sauvH:	ldi c,0b00110000; 0 en ASCII
			add a,c			; convertion ASCII
			add b,c
			st X+,b			; sauvegarde de la valeur lcd à afficher
			st X+,a
			ret

;-------------------------------------------------------------------
; MULTIPLICATION 16 bit * 16 bit
; mp16uH:mp16uL  	x	 mc16uH:mc16uL	=	m16u3:m16u2:m16u1:m16u0
;    	X			x      		Y		=       resultat

mul16:	clr	m16u3		;clear 2 highest bytes of result
		clr	m16u2	
		lsr	mp16uH		;rotate multiplier Low
		ror	mp16uL		;rotate multiplier High

		brcc	noadd0		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd0:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd1		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd1:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd2		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd2:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd3		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd3:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd4		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd4:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd5		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd5:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd6		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd6:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd7		;if carry sett
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd7:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd8		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd8:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noadd9		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noadd9:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad10		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad10:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad11		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad11:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad12		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad12:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad13		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad13:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad14		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad14:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		brcc	noad15		;if carry set
		add	m16u2,mc16uL	;    add multiplicand Low to byte 2 of res
		adc	m16u3,mc16uH	;    add multiplicand high to byte 3 of res
noad15:	ror	m16u3		;shift right result byte 3
		ror	m16u2		;rotate right result byte 2
		ror	m16u1		;rotate result byte 1 and multiplier High
		ror	m16u0		;rotate result byte 0 and multiplier Low

		ret

;---------------------------------------------------------------------------------
; DIVISION 16 BITS
; dd16uH:dd16uL	/ 	dv16uH:dv16uL 	=	(dres16uH:dres16uL)	: (drem16uH:drem16uL)
; dividende		/	  diviseur 		=  	      résultat 	:           reste 


div16:	clr	drem16uL	;clear remainder Low byte
		sub	drem16uH,drem16uH;clear remainder High byte and carry

		rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_1		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_2		;else
d16u_1:	sec			;    set carry to be shifted into result

d16u_2:	rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_3		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_4		;else
d16u_3:	sec			;    set carry to be shifted into result

d16u_4:	rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_5		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_6		;else
d16u_5:	sec			;    set carry to be shifted into result

d16u_6:	rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_7		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_8		;else
d16u_7:	sec			;    set carry to be shifted into result

d16u_8:	rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_9		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_10		;else
d16u_9:	sec			;    set carry to be shifted into result

d16u_10:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_11		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_12		;else
d16u_11:sec			;    set carry to be shifted into result

d16u_12:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_13		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_14		;else
d16u_13:sec			;    set carry to be shifted into result

d16u_14:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_15		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_16		;else
d16u_15:sec			;    set carry to be shifted into result

d16u_16:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_17		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_18		;else
d16u_17:	sec			;    set carry to be shifted into result

d16u_18:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_19		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_20		;else
d16u_19:sec			;    set carry to be shifted into result

d16u_20:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_21		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_22		;else
d16u_21:sec			;    set carry to be shifted into result

d16u_22:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_23		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_24		;else
d16u_23:sec			;    set carry to be shifted into result

d16u_24:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_25		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_26		;else
d16u_25:sec			;    set carry to be shifted into result

d16u_26:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_27		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_28		;else
d16u_27:sec			;    set carry to be shifted into result

d16u_28:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_29		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_30		;else
d16u_29:sec			;    set carry to be shifted into result

d16u_30:rol	dd16uL		;shift left dividend
		rol	dd16uH
		rol	drem16uL	;shift dividend into remainder
		rol	drem16uH
		sub	drem16uL,dv16uL	;remainder = remainder - divisor
		sbc	drem16uH,dv16uH	;
		brcc	d16u_31		;if result negative
		add	drem16uL,dv16uL	;    restore remainder
		adc	drem16uH,dv16uH
		clc			;    clear carry to be shifted into result
		rjmp	d16u_32		;else
d16u_31:sec			;    set carry to be shifted into result

d16u_32:rol	dd16uL		;shift left dividend
		rol	dd16uH
		ret


;------------------------------------------------
; 
; "sub16" - Soustraction 16+16
;
; sub1h:sub1l	- 	sub2h:sub2l		=	 sub1h:sub1l
; 
;   r19:r18	-	  r21:r22		=	   r19:r18
;
; Nombre de mots		:3
; Nombre de  cycles	:6
; Registre Bas utilisés	:Aucun
; Registre Haut utilisés  	:4



sub16:	sub	sub1l,sub2l		;Subtract low bytes
		sbc	sub1h,sub2h		;Add high byte with carry
		ret

;-------------------------------------
; AQUISISION TEMPERATURE ET PRESSION
; aquisision avec le convertisseur A/N
; et sauvegarde avec le pointeur Y

aquisition:						; aquisition de la pression
			clr a
			out admux,a
			sbi adcsra,adsc
aqui1:		sbic adcsra,adsc	; attente de la fin de la convertion
			rjmp aqui1
			in a,adcl			; 
			in b,adch

			ldi c,26			;modification de la valeur
			clr d				; ((X*25)/47)-14
			rcall mul16
			mov a,res0
			mov b,res1
			ldi c,97
			clr d
			rcall div16
			subi dres16ul,14

			ldi XH,high(valeur)
			ldi XL,low(valeur)
			st X+,a
			ldi YH,high(data)
			ldi YL,low(data)
			adiw YH:YL,3
			rcall bin_bcd	;on modifie la valeur pour la rendre affichable
			ldi c,0x30		; 0 en ASCII
			add a,c
			st -Y,a			; store chiffre des unitées
			mov a,b
			rcall bin_bcd	;
			add a,c
			add b,c
			st -Y,a
			st -Y,b

			ldi a,0x01
			out admux,a
			sbi adcsra,adsc		; aquisition température de l'étuve
aqui2:		sbic adcsra,adsc	; attente de la fin de la convertion
			rjmp aqui2
			in a,adcl			; 
			in b,adch			;

			ldi c,125			; modification de la  valeur
			clr d				; ((X*100)/255)
			rcall mul16
			mov a,res0
			mov b,res1
			ldi c,255
			clr d
			rcall div16
	
			st X+,a
			adiw YH:YL,3
			rcall bin_bcd
			ldi c,0x30
			add a,c
			add b,c
			st Y+,b
			st Y+,a
			ldi a,0x02
			out admux,a
			sbi adcsra,adsc		; aquisition température du chauffage
aqui3:		sbic adcsra,adsc	; attente de la fin de la convertion
			rjmp aqui3
			in a,adcl			;
			in b,adch 

			ldi c,15			;modification de la valeur
			clr d				; ((X*25)/47)-14
			rcall mul16
			mov a,res0
			mov b,res1
			ldi c,30
			clr d
			rcall div16

			st X,a
			rcall bin_bcd
			ldi c,0x30
			add a,c
			add b,c
			st Y+,b
			st Y,a
			ret


;********************************
; ROUTINE EEPROM
; ecriture et lecture en eemprom
;********************************

;------------------------------
; lecture eeprom
; addresse haute contenu dans a
; addresse basse contenu dans b
; retour de la valeur dans c

eeprom_read:

				sbic EECR,EEWE
				rjmp EEPROM_read

				out EEARH, a
				out EEARL, b

				sbi EECR,EERE
				
				in c,EEDR
				ret

;-------------------------------
; ecriture eeprom
; addresse haute contenu dans a
; addresse basse contenu dans b
; valeur dans c

eeprom_write:

				sbic EECR,EEWE
				rjmp EEPROM_write

				out EEARH, a
				out EEARL, b

				out EEDR,c

				sbi EECR,EEMWE

				sbi EECR,EEWE
				ret

;************************************
; ROUTINES AFFICHAGE LCD 
; routine d'affichage du lcd en 4 bit
;************************************

; initialisation du lcd dans la partie initialisation.du micro

;-----------------------
; MISE EN ROUTE CURSEUR 

curseur_on:		ldi a,0x00
				out lcd_port,a
				validation
				ldi a,0x0e
				out lcd_port,a
				validation
				ret

;--------------------
; ANNULATION CURSEUR 

curseur_off:	ldi a,0x00
				out lcd_port,a
				validation
				ldi a,0x0c
				out lcd_port,a
				validation
				ret

;----------------
; EFFACEMENT LCD
; 

clr_lcd:		ldi a,0x00
				out lcd_port,a
				validation
				ldi a,0x01
				out lcd_port,a
				validation
				ret
				

;-----------------------------------
; selection de l'adresse d'affichage
; addresse contenu dans b

add_ddram:		sbr b,0b10000000		; selection du bit addresse ddram
				push b
				swap b
				andi b,0x0F
				out lcd_port,b
				validation
				pop b
				andi b,0x0F
				out lcd_port,b
				validation
				rcall delay1ms
				ret
				

;--------------------------------------
; affichage de texte avec le pointeur Z
; texte contenu dans le code flash

affich_Ztxt:	lpm 
				tst R0 				;test si plus de valeur
				breq affich_ZtxtR	; 
				mov a,R0			; on charge le caractère et on l'affiche
				rcall affich_char
				adiw ZH:ZL,1
				rjmp affich_Ztxt	; on prend le caractère suivant
affich_ZtxtR:
				ret					;retour

;--------------------------------------
; affichage de texte avec le pointeur Y
; texte contenu dans la RAM
; b doit contenir le nombre de caractère à afficher

affich_Ytxt:
				tst b 				;test si plus de valeur
				breq affich_YtxtR	; on charge le caractère et on l'affiche
				ld a,Y+ 
				rcall affich_char
				dec b
				rjmp affich_Ytxt	; on prend le caractère suivant
affich_YtxtR:
				ret					;retour
;---------------------------------
; affichage d'un caractère 
; a contient le caractère en ASCII

				
affich_char:	push a	
				swap a
				andi a,0x0F
				sbr a,0x40			; mise à 1 du bit RS!!!!!!!!!!!!!!!!!
				out lcd_port,a
				validation
				pop a
				andi a,0x0F
				sbr a,0x40			; mise à 1 bit RS !!!!!!!!!!!!!!
				out lcd_port,a
				validation
				rcall delay1ms
				ret

;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
;Déclaration des zones de variables dans l'eeprom
;xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

.ESEG
	pression: 		.BYTE 1		; heure en ASCII
	temperature:	.BYTE 1		; valeur de pression et température en ASCII
	retro_led:		.BYTE 1		; retroéclairage de l'écran
	horloge:		.BYTE 3		; heure
	hysteresis:		.BYTE 1		; hysteresis
