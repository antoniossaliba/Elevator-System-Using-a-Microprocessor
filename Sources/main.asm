;*****************************************************************
;* This stationery serves as the framework for a                 *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                         *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                   *
;* Freescale CodeWarrior for the HC12 Program directory          *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

ROMStart    EQU  $4000  ; absolute address to place my code/constant data

; variable/data section

            ORG RAMStart
 ; Insert here your data definition.
LEVEL       DC.B  0                   ; THIS VARIABLE INDICATES THE CURRENT POSITION (0 = GF / 1 = FL1 / 2 = FL2)

;---------------TEXTUAL VARIABLES---------------
MOVEUP      DC.B    'MOVINGUP'
MOVEDOWN    DC.B    'MOVINGDOWN'
GROUND      DC.B    'GROUNDFLOOR'
FLOOR1      DC.B    'FLOORONE'
FLOOR2      DC.B    'FLOORTWO'
WEIGHT      DC.B    'WEIGHTEXCEEDED'
HOLIDAY     DC.B    'HAPPYHOLIDAY'
;---------------SIZE VARIABLES------------------
MOVEUPSIZE1   DC.B    6
MOVEUPSIZE2   DC.B    2
MOVEDOWNSIZE1 DC.B    6
MOVEDOWNSIZE2 DC.B    4
GROUNDSIZE1   DC.B    6
GROUNDSIZE2   DC.B    5
FLOOR1SIZE1   DC.B    5
FLOOR1SIZE2   DC.B    3
FLOOR2SIZE1   DC.B    5
FLOOR2SIZE2   DC.B    3
WEIGHT1       DC.B    6
WEIGHT2       DC.B    8
HOLIDAY1      DC.B    5
HOLIDAY2      DC.B    7
DBL           DC.B    2
SONGCNT       DC.B    2   
;TIMER         DC.B    2
;---------------BLINK VARIABLES;---------------
GREENLEDFLAG  DC.B    0
YELLOWLEDFLAG DC.B    0
REDLEDFLAG    DC.B    0
;---------------DELAY COUNT;---------------
COUNT         DC.B    8
INNERCOUNT    DC.B    0

POTENT      DS.B  1                   ; THIS IS A DUMMY VARIABLE TO GET THE HOLD THE VALUE OF THE POTENTIOMETER
FSM         DC.B  0                   ; THIS VARIABLE CONTROLS THE FINITE STATE MACHINE WHICH TELLS ME THE INSTRUCTIONS TO PERFORM
                                      ; $00 --> IDLE
                                      ; $FF --> WEIGHT RESTRICTIONS
                                      ; $01 --> MOVING TO FL1 UP      FORM GF       (FL01)
                                      ; $02 --> MOVING TO FL2 UP                    (FL02)
                                      ; $03 --> MOVING TO FL1 THEN 2                (FL012)
                                      ; $04 --> MOVING TO FL2 UP      FROM FL1      (FL12)
                                      ; $05 --> MOVING TO GF  DOWN                  (FL10)
                                      ; $06 --> MOVING TO FL2 THEN GF               (FL120)
                                      ; $07 --> MOVING TO FL1 DOWN    FROM FL2      (FL21)
                                      ; $08 --> MOVING TO GF  DOWN                  (FL20)
                                      ; $09 --> MOVING TO FL1 THEN GF               (FL210)
                                      

; code section
            ORG   ROMStart


Entry:
_Startup:
            LDS   #RAMEnd+1           ; initialize the stack pointer

            CLI                       ; enable interrupts
            
            MOVB  #$F0,DDRB
            MOVB  #$F0,PORTB
            
            MOVB  #$C7,MCCTL          ; CONFIGURING THE MCCNT COUNTER
            MOVW  #62500,MCCNT
            
            ;MOVB  #$03,DDRA           ; SET THE PB PINS TO INPUT AND THE FIRST TWO AS OUTPUT FOR THE H-BRIDGE
            CLR   DDRA                 
            
            MOVB  #%11111000,DDRT           ; CONFIG LEDS
            MOVB  #%00000000, PTT     ; LEDS OFF
            
            MOVB  #%11000000,ATD0CTL2 ; CONFIGURING THE POTENTIOMETER
            MOVB  #%00001000,ATD0CTL3
            MOVB  #%10000000,ATD0CTL4
            MOVB  #%10100101,ATD0CTL5
            
            MOVB #$10,MODRR           ; CONFIG LCD REGISTERS
            MOVB #$38,DDRM
            MOVB #$52,SPI0CR1
            MOVB #$10,SPI0CR2
            MOVB #$00,SPI0BR
            
            
            LDAA  #%00110011          ; LCDs INSTRUCTION + DISPLAY ON
            JSR   SENDINST
            LDAA  #%00110010
            JSR   SENDINST
            LDAA  #%00101000
            JSR   SENDINST
            LDAA  #%00001000
            JSR   SENDINST
            LDAA  #%00000001
            JSR   SENDINST
            LDAA  #%00000110
            JSR   SENDINST
          
            LDAA  #%00001110
            JSR   SENDINST
            
            MOVB  #%00010100,PWME     ; ACTIVATING CHANNEL 2, 3  AND 4
            ;MOVB  #%00010000,PWME     ; ACTIVATING CHANNEL 4 FOR BUZZER
            MOVB  #%00011100,PWMPOL   ; STARTING WITH HIGH (POLARITY 1) FOR CHANNELS 2, 3 AND 4
            MOVB  #%00000000,PWMCLK   ; WE ARE CHOSING THE CLOCK WE NEED TO USE: A AND B, NOT subA OR CONCATENATE A
            MOVB  #%11100011,PWMPRCLK ; WE ARE SPECIFIYING THE PRESCALER TO THE CLOCK: DIVISION BY 8 FOR CLOKC A AND DIVISION BY 128 FOR CLOCK B
            MOVB  #$0C,PWMCTL         ; CONFIGURING THE PWM CHANNEL
            
           ;DEFINING THE PWM PARAMETER: COUNTER, DUTY AND PERIOD
           
            CLR   PWMCNT4             ; CLEARING THE COUNTER
            ;CLR   PWMCNT3             ; CLEARING THE COUNTER
            CLR   PWMCNT2             ; CLEARING THE COUNTER
            
            MOVB  #156,PWMPER2
            
mainLoop:
            LDAA  FSM
            CMPA  #00
            LBEQ  STP
            
            
            CMPA  #$FF                
            LBEQ  ALRM            
            
            CMPA  #$01
            LBEQ  F01
            
            CMPA  #$02
            LBEQ  F02            
            
            CMPA  #$03
            LBEQ  F012            
            
            CMPA  #$04
            LBEQ  F12            
            
            CMPA  #$05
            LBEQ  F10            
            
            CMPA  #$06
            LBEQ  F120            
            
            CMPA  #$07
            LBEQ  F21            
            
            CMPA  #$08
            LBEQ  F20
            
            JMP   F210
            
STP                                   ; IDLE STATE
            MOVB  #195,PWMPER4
            MOVB  #195,PWMDTY4
            
            MOVB  #$00,PORTB
            
            
            JMP   mainLoop

                                      ; BLINK THE RED LED BY COMPLEMETING THE BIT EVEYTIME YOU GET HERE --> EACH 0.5s

ALRM                                  ; POTENTIOMETER IDICATING OVER WEIGHT
            MOVB  #$F0,PORTB
            JSR   ALARM
            MOVB  #$00,FSM
            
            
            JMP   mainLoop

F01                                   
                                      
            MOVB  #55,PWMDTY2
            MOVB  #%10010000,PORTB                          ; TO TURN THE MOTOR, ACTIVATE CHANNEL 2 OR 3, SET PWMPER2/3 TO 156 AND PWMDTY2/3 TO 32
            ;MOVB  #%00010000,PTT
            JSR   MOVEUPBSR
            ;MOVB  #%00000000,PTT
            JSR   FLOOR1BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG 
            MOVB  #1,LEVEL
            MOVB  #0,FSM                        
            JMP   mainLoop                          

F02         
            MOVB  #50,PWMDTY2
            MOVB  #%01100000, PORTB
            MOVB  #16, COUNT
            JSR   MOVEUPBSR
            JSR   FLOOR2BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #2, LEVEL
            MOVB  #0, FSM                          
            JMP   mainLoop
            

F012        
            MOVB  #55,PWMDTY2
            MOVB  #%01010000, PORTB
            JSR   MOVEUPBSR
            JSR   FLOOR1BSR
            MOVB  #195, PWMPER4
            MOVB  #195, PWMDTY4
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            JSR   MOVEUPBSR
            JSR   FLOOR2BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #2, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop
                                      

F12         
            MOVB  #55,PWMDTY2
            MOVB  #%00010000, PORTB
            JSR   MOVEUPBSR
            JSR   FLOOR2BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #2, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop                          

F10         
            MOVB  #55,PWMDTY2
            MOVB  #%01000000, PORTB
            JSR   MOVEDOWNBSR
            JSR   GFBSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #0, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop                          
              
F120        
            MOVB  #55,PWMDTY2
            JSR   MOVEUPBSR
            JSR   FLOOR2BSR
            MOVB  #195, PWMPER4
            MOVB  #195, PWMDTY4
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #16, COUNT
            MOVB  #50,PWMDTY2
            JSR   MOVEDOWNBSR
            JSR   GFBSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #0, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop
F21         
            MOVB  #55,PWMDTY2
            JSR   MOVEDOWNBSR
            JSR   FLOOR1BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #1, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop                          ; MOVE FROM 2 TO 1

F20         
            MOVB  #50,PWMDTY2
            MOVB  #16, COUNT
            JSR   MOVEDOWNBSR
            JSR   GFBSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #0, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop                          ; MOVE FROM 2 TO 0

F210                                  ; MOVE FROM 2 TO 1 TO 0 AUTOMATICALLY
            MOVB  #55,PWMDTY2
            JSR   MOVEDOWNBSR
            JSR   FLOOR1BSR
            MOVB  #195, PWMPER4
            MOVB  #195, PWMDTY4
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            JSR   MOVEDOWNBSR
            JSR   GFBSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            MOVB  #0, LEVEL
            MOVB  #0, FSM
            JMP   mainLoop                        
          
; SUBTROUTINES

;MOTOR_UP                              ; SET THE POLARITY OF THE H-BIRDGE TO MOVE THE MOTOR UP  (IF NEEDED)

;MOTOR_DN                              ; SET THE POLATITY OF THE H-BIRDGE TO MOVE THE MOTOR DN  (IF NEEDED)

COUNTER_ISR
            MOVB  #$80,MCFLG          ; RESETTING THE MCCNT FLAG
            
            ;DEC   TIMER
            ;BNE   OUTT
            ;BRA   CTT
            
OUTT        ;JMP   OUT
            
CTT         ;MOVB  #2,TIMER
            LDAA  FSM
            CMPA  #$00                ; TEST IF THE ELEVATOR IS MOVING --> LOCK THE BUTTONS
            BNE   SKIP
            
            JSR   READ_POTENT
            LDAA  POTENT
            CMPA  #128                ; TESTING IF THE POTENTIOMETER IS GREATER THAN A CERTAIN VALUE --> ALARM
            BHS   AL
            BRA   CT

AL          JMP ALARM0
            
            ; THE ELEVATOR IS IDLE, I CAN READ NEW BUTTON VALUES
            
            ;  TESTING FOR DOUBLE PRESSES
                        
            
CT          LDAA  PORTA
            COMA
            BITA  #%00001000
            BNE   CHECK2
            JMP   NEXT1
            
CHECK2      BITA  #%00010000
            LBNE  FL1_2             
            
NEXT1       LDAA  PORTA
            COMA
            BITA  #%00000100
            BNE   CHECK3
            JMP   NEXT2
            
CHECK3      BITA  #%00010000
            LBNE  FL0_2            
                                  ; TESTING FOR PB3 AND PB5            
            
NEXT2       LDAA  PORTA
            COMA
            BITA  #%00001000
            BNE   CHECK4
            BRA   NEXT3
                                  ; TESTING FOR PB3 AND PB4
CHECK4      BITA  #%00000100
            LBNE  FL0_1
            
NEXT3       LDAA  PORTA
            COMA
            ANDA  #%00100100
            TSTA                      ; TESTING FOR PB3 AND PB6, IF ANY IS PRESSED --> GO TO FLOOR 0
            LBNE  FL00
            
            
NEXT4       LDAA  PORTA
            COMA
            ANDA  #%01001000
            TSTA                      ; TESTING FOR PB4 AND PB7, IF ANY IS PRESSED --> GO TO FLOOR 1
            LBNE  FL11
            
NEXT5       LDAA  PORTA
            COMA
            ANDA  #%10010000
            TSTA                      ; TESTING FOR PB5 AND PB8, IF ANY IS PRESSED --> GO TO FLOOR 2
            LBNE  FL22            
            
            

SKIP        JMP   OUT
                                    
FL00
            LDAA  LEVEL               ; FIRST WE CEHCK WHETHER WE ARE ON THE SAME FLOOR OR NOT
            CMPA  #0                  ; THEN WE COMPARE THE CURRENT FLOOR TO KNOW THE DIRECTION OF THE 
            BEQ   OPEN0
            BRA   NN2                ; DESTINATION
            
OPEN0       JSR   GFBSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            JMP   IDLE       
            
NN2         CMPA  #1
            BEQ   FL10
            
            BRA   FL20
            
FL11
            LDAA  LEVEL
            CMPA  #1
            BEQ   OPEN1
            BRA   NN3                ; DESTINATION
            
OPEN1       JSR   FLOOR1BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            BRA   IDLE  
            
NN3         CMPA  #0
            BEQ   FL01
            
            BRA   FL21
            
FL22
            LDAA  LEVEL
            CMPA  #2
            BEQ   OPEN3
            BRA   NN4                ; DESTINATION
            
OPEN3       JSR   FLOOR2BSR
            JSR   LEDDELAY
            JSR   HOLIDAYBSR
            JSR   SONG
            BRA   IDLE  
            
NN4         CMPA  #0
            BEQ   FL02
            
            BRA   FL12
            
            ; CHOSING WHERE TO GO NEXT IF I AM NOT SELECTING THE CURRENT FLOOR
FL01
            MOVB  #$01,FSM
            BRA   OUT

FL02
            MOVB  #$02,FSM
            BRA   OUT

FL10
            MOVB  #$05,FSM
            BRA   OUT

FL12
            MOVB  #$04,FSM
            BRA   OUT

FL20
            MOVB  #$08,FSM
            BRA   OUT

FL21
            MOVB  #$07,FSM
            BRA   OUT
            
            ; CHOSING THE DOUBLE MOVEMENT
FL0_1
            LDAA  LEVEL
            CMPA  #2
            BNE   IDLE
            
            MOVB  #$09,FSM
            
            BRA   OUT
            
FL0_2
            LDAA  LEVEL
            CMPA  #1
            BNE   IDLE
            
            MOVB  #$06,FSM
            
            BRA   OUT
            
FL1_2            
            LDAA  LEVEL
            CMPA  #0
            BNE   IDLE
            
            MOVB  #$03,FSM
            BRA   OUT
            
            ;BRA   IDLE
            
ALARM0
            MOVB  #$FF,FSM
            BRA   OUT
            
IDLE
            MOVB  #0,FSM
            BRA   OUT                                              
            
OUT         MOVW  #62500,MCCNT        ; RE-INITIATING THE MMCNT
            RTI
            
;---------------MOVE UP DISPLAY;---------------
MOVEUPBSR     LDAA #%00000001
              JSR  SENDINST
              LDX  #MOVEUP
              LDAA #%10000001
              JSR  SENDINST
MOVEUPLOOP1   LDAA 1, X+
              JSR  SENDDATA
              DEC  MOVEUPSIZE1
              BNE  MOVEUPLOOP1
              LDAA #%11000011
              JSR  SENDINST
MOVEUPLOOP2   LDAA 1, X+
              JSR  SENDDATA
              DEC  MOVEUPSIZE2
              BNE  MOVEUPLOOP2
              MOVB #6, MOVEUPSIZE1
              MOVB #2, MOVEUPSIZE2
MOVEUPRES     JSR  LEDDELAY
              TST  GREENLEDFLAG
              BNE  TURNOFFMOVEUP
              MOVB #%10010000, PTT
              COM  GREENLEDFLAG
              BRA  OUTMOVEUP
TURNOFFMOVEUP MOVB #%00010000, PTT
              COM  GREENLEDFLAG
OUTMOVEUP     DEC  COUNT
              BNE  MOVEUPRES
              MOVB #%00000000, PTT
              MOVB #8, COUNT
              RTS
;---------------MOVE UP DISPLAY;---------------                        
            
;---------------MOVE DOWN DISPLAY;---------------
            
MOVEDOWNBSR   LDAA  #%00000001
              JSR   SENDINST
              LDX   #MOVEDOWN         
              LDAA  #%10000001
              JSR   SENDINST
MOVEDOWNLOOP1 LDAA  1, X+
              JSR   SENDDATA
              DEC   MOVEDOWNSIZE1
              BNE   MOVEDOWNLOOP1
              LDAA  #%11000010
              JSR   SENDINST
MOVEDOWNLOOP2 LDAA  1, X+
              JSR   SENDDATA
              DEC   MOVEDOWNSIZE2
              BNE   MOVEDOWNLOOP2
              MOVB  #6, MOVEDOWNSIZE1
              MOVB  #4, MOVEDOWNSIZE2
MOVEDOWNRES   JSR   LEDDELAY
              TST   YELLOWLEDFLAG
              BNE   TURNOFFMOVED
              MOVB  #%01001000, PTT
              COM   YELLOWLEDFLAG
              BRA   OUTMOVEDOWN
TURNOFFMOVED  MOVB  #%00001000, PTT
              COM   YELLOWLEDFLAG
OUTMOVEDOWN   DEC   COUNT
              BNE   MOVEDOWNRES
              MOVB  #%00000000, PTT
              MOVB  #8, COUNT              
              RTS             
;---------------MOVE DOWN DISPLAY;---------------

;---------------GF DISPLAY;---------------
GFBSR         MOVB  #195, PWMPER4
              MOVB  #195, PWMDTY4
              LDAA  #%00000001
              JSR   SENDINST
              LDX   #GROUND
              LDAA  #%10000001
              JSR   SENDINST
GFLOOP1       LDAA  1, X+
              JSR   SENDDATA
              DEC   GROUNDSIZE1
              BNE   GFLOOP1
              MOVB  #6, GROUNDSIZE1
              LDAA  #%11000001
              JSR   SENDINST
GFLOOP2       LDAA  1, X+
              JSR   SENDDATA
              DEC   GROUNDSIZE2
              BNE   GFLOOP2
              MOVB  #5, GROUNDSIZE2
              
STOP1         JSR   LEDDELAY
              TST   REDLEDFLAG
              BNE   TURNOFFSTOP1
              MOVB  #%00100000, PTT
              COM   REDLEDFLAG
              BRA   OUTSTOP1
TURNOFFSTOP1  MOVB  #%00000000, PTT
              COM   REDLEDFLAG
OUTSTOP1      LDAB  INNERCOUNT
              CMPB  #3
              BHI   DONG1
              MOVB  #195,PWMPER4
              MOVB  #97,PWMDTY4
              BRA   ADJUST1
DONG1         MOVB  #240,PWMPER4
              MOVB  #120,PWMDTY4   
ADJUST1       INC   INNERCOUNT
              DEC   COUNT
              BNE   STOP1
              MOVB  #8, COUNT
              MOVB  #0, INNERCOUNT
              RTS
;---------------GF DISPLAY;---------------

;---------------FLOOR 1 DISPLAY;---------------
FLOOR1BSR      MOVB  #195, PWMPER4
               MOVB  #195, PWMDTY4
               LDAA #%00000001
               JSR  SENDINST
               LDX  #FLOOR1
               LDAA #%10000010
               JSR  SENDINST
FLOOR1LOOP1    LDAA 1, X+
               JSR  SENDDATA
               DEC  FLOOR1SIZE1
               BNE  FLOOR1LOOP1
               MOVB #5, FLOOR1SIZE1
               LDAA #%11000010
               JSR  SENDINST
FLOOR1LOOP2    LDAA 1, X+
               JSR  SENDDATA
               DEC  FLOOR1SIZE2
               BNE  FLOOR1LOOP2
               MOVB #3, FLOOR1SIZE2
               
STOP2          JSR   LEDDELAY
               TST   REDLEDFLAG
               BNE   TURNOFFSTOP2
               MOVB  #%00100000, PTT
               COM   REDLEDFLAG
               BRA   OUTSTOP2
TURNOFFSTOP2   MOVB  #%00000000, PTT
               COM   REDLEDFLAG
OUTSTOP2       LDAB  INNERCOUNT
               CMPB  #3
               BHI   DONG2
               MOVB  #195,PWMPER4
               MOVB  #97,PWMDTY4
               BRA   ADJUST2
DONG2          MOVB  #240,PWMPER4
               MOVB  #120,PWMDTY4   
ADJUST2        INC   INNERCOUNT
               DEC   COUNT
               BNE   STOP2
               MOVB  #8, COUNT
               MOVB  #0, INNERCOUNT
               RTS
;---------------FLOOR 1 DISPLAY;--------------- 

FLOOR2BSR      MOVB  #195, PWMPER4
               MOVB  #195, PWMDTY4
               LDAA #%00000001
               JSR  SENDINST
               LDX  #FLOOR2
               LDAA #%10000010
               JSR  SENDINST
FLOOR2LOOP1    LDAA 1, X+
               JSR  SENDDATA
               DEC  FLOOR2SIZE1
               BNE  FLOOR2LOOP1
               MOVB #5, FLOOR2SIZE1
               LDAA #%11000010
               JSR  SENDINST
FLOOR2LOOP2    LDAA 1, X+
               JSR  SENDDATA
               DEC  FLOOR2SIZE2
               BNE  FLOOR2LOOP2
               MOVB #3, FLOOR2SIZE2
STOP3          JSR   LEDDELAY
               TST   REDLEDFLAG
               BNE   TURNOFFSTOP3
               MOVB  #%00100000, PTT
               COM   REDLEDFLAG
               BRA   OUTSTOP3
TURNOFFSTOP3   MOVB  #%00000000, PTT
               COM   REDLEDFLAG
OUTSTOP3       LDAB  INNERCOUNT
               CMPB  #3
               BHI   DONG3
               MOVB  #195,PWMPER4
               MOVB  #97,PWMDTY4
               BRA   ADJUST3
DONG3          MOVB  #240,PWMPER4
               MOVB  #120,PWMDTY4   
ADJUST3        INC   INNERCOUNT
               DEC   COUNT
               BNE   STOP3
               MOVB  #8, COUNT
               MOVB  #0, INNERCOUNT
               RTS
;---------------FLOOR 2 DISPLAY;---------------
;---------------HOLIDAY DISPLAY;---------------
HOLIDAYBSR     MOVB  #195, PWMPER4
               MOVB  #195, PWMDTY4
               LDAA  #%00000001
               JSR   SENDINST
               LDX   #HOLIDAY
               LDAA  #%10000010
               JSR   SENDINST
HOLIDAYLOOP1   LDAA  1, X+
               JSR   SENDDATA
               DEC   HOLIDAY1
               BNE   HOLIDAYLOOP1
               LDAA  #%11000001
               JSR   SENDINST
HOLIDAYLOOP2   LDAA  1, X+
               JSR   SENDDATA
               DEC   HOLIDAY2
               BNE   HOLIDAYLOOP2
               MOVB  #5, HOLIDAY1
               MOVB  #7, HOLIDAY2
               RTS
;---------------HOLIDAY DISPLAY;---------------
;---------------SONG LISTENING;----------------
SONG
REPEAT      MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY
            MOVB #%00000000, PTT          

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY          
            MOVB #%11100000, PTT

            ; G5 C5 D5 E5
            MOVB #160,PWMPER4
            MOVB #80,PWMDTY4
            JSR LEDDELAY           
            MOVB #%00000000, PTT

            MOVB #239,PWMPER4
            MOVB #120,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            MOVB #213,PWMPER4
            MOVB #106,PWMDTY4
            JSR LEDDELAY           
            MOVB #%00000000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            
            MOVB #179,PWMPER4
            MOVB #90,PWMDTY4
            JSR LEDDELAY          
            MOVB #%00000000, PTT

            MOVB #179,PWMPER4
            MOVB #90,PWMDTY4
            JSR LEDDELAY          
            MOVB #%11100000, PTT

            MOVB #179,PWMPER4
            MOVB #90,PWMDTY4
            JSR LEDDELAY          
            MOVB #%00000000, PTT

            MOVB #179,PWMPER4
            MOVB #90,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            ; E5 E5 E5 E5
            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%00000000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%00000000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            ; D5 D5 E5 D5 G5
            MOVB #213,PWMPER4
            MOVB #106,PWMDTY4
            JSR LEDDELAY           
            MOVB #%00000000, PTT

            MOVB #213,PWMPER4
            MOVB #106,PWMDTY4
            JSR LEDDELAY          
            MOVB #%11100000, PTT

            MOVB #190,PWMPER4
            MOVB #95,PWMDTY4
            JSR LEDDELAY          
            MOVB #%00000000, PTT

            MOVB #213,PWMPER4
            MOVB #106,PWMDTY4
            JSR LEDDELAY           
            MOVB #%11100000, PTT

            MOVB #160,PWMPER4
            MOVB #80,PWMDTY4
            JSR LEDDELAY          
            MOVB #%00000000, PTT
            DEC SONGCNT
            LBNE REPEAT
            MOVB  #2, SONGCNT
            MOVB  #195, PWMPER4
            MOVB  #195, PWMDTY4
            MOVB  #$E0, PTT
            JSR   LEDDELAY
            MOVB  #$00, PTT
            JSR   LEDDELAY
            MOVB  #$E0, PTT
            JSR   LEDDELAY
            MOVB  #$00, PTT
            JSR   LEDDELAY
            MOVB  #$E0, PTT
            JSR   LEDDELAY
            MOVB  #$00, PTT
            JSR   LEDDELAY
            MOVB  #$E0, PTT
            JSR   LEDDELAY
            MOVB  #$00, PTT
            JSR   LEDDELAY
            MOVB  #$E0, PTT
            JSR   LEDDELAY
            MOVB  #$00, PTT
            JSR   LEDDELAY
            MOVB  #$E0, PTT
            RTS  
;---------------SONG LISTENING;----------------

;---------------READ POTENTIOMETER;---------------
READ_POTENT                        
LOOPP         BRCLR ATD0STAT0,#$80,LOOPP
              MOVB  ATD0DR0L,POTENT
              RTS
              
ALARM         LDAA  #%00000001
              JSR   SENDINST
              LDX   #WEIGHT
BSR1          LDAA  1, X+
              JSR   SENDDATA
              DEC   WEIGHT1
              BNE   BSR1
              LDAA  #%11000000
              JSR   SENDINST
BSR2          LDAA  1, X+
              BSR   SENDDATA
              DEC   WEIGHT2
              BNE   BSR2
              MOVB  #6, WEIGHT1
              MOVB  #8, WEIGHT2

BEEP          MOVB  #195,PWMPER4
              MOVB  #97,PWMDTY4 
              MOVB  #%00100000,PTT
            
         
              JSR   LEDDELAY
            
            
              MOVB  #195,PWMPER4
              MOVB  #195,PWMDTY4
              MOVB  #%00000000,PTT
            
            
              BSR   LEDDELAY
            
              
              BSR   READ_POTENT
              LDAA  POTENT
              CMPA  #128
              BHS   BEEP
              LDAA  #%00000001
              JSR   SENDINST
              RTS
;---------------READ POTENTIOMETER;--------------- 
            
SENDINST      TAB
              RORA            
              RORA 
              RORA            
              RORA

              ORAA #%10000000 
              ANDA #%10001111
              JSR SENDSPI
              ANDA #%00001111

              JSR SENDSPI
              JSR LCDDELAY
              TBA 
              ORAA #%10000000 
              ANDA #%10001111
              JSR SENDSPI
              ANDA #%00001111
              JSR SENDSPI

              JSR LCDDELAY
              RTS

SENDDATA      TAB
              RORA            
              RORA 
              RORA            
              RORA

              ORAA #%11000000 
              ANDA #%11001111
              JSR SENDSPI
              ANDA #%01001111

              JSR SENDSPI
              JSR LCDDELAY
              TBA 
              ORAA #%11000000 
              ANDA #%11001111
              JSR SENDSPI
              ANDA #%01001111
              JSR SENDSPI

              JSR LCDDELAY
              RTS
            
            
            
SENDSPI       BRCLR SPI0SR,#$20,*
              STAA SPI0DR
            
              RTS 
              
LCDDELAY      LDY   #10527 ; 100ms DELAY (CAN BE ADJUSTED BASED ON DEMAND)
AGAIN1        PSHB
              PULB
              PSHB
              PULB
              PSHB
              PULB
              DEY
              BNE   AGAIN1
              RTS
              
LEDDELAY      LDX   #52632 ; 500ms DELAY (CAN BE ADJUSTED BASED ON DEMAND)
AGAIN2        PSHB
              PULB
              PSHB
              PULB
              PSHB
              PULB
              DEX
              BNE   AGAIN2
              RTS
              
BUZZERDELAY   LDX   #30000
AGAIN3        PSHB
              PULB
              PSHB
              PULB
              PSHB
              PULB
              DEX
              BNE   AGAIN3
              RTS                              
;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector
            
            ORG   $FFCA
            DC.W  COUNTER_ISR
