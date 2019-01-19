; ---------------------------------------------------------------------------------------
;  @file:    startup_S32K144.s
;  @purpose: Cosmic C Cross Compiler Startup File
;            S32K144
;  @version: 2.0
;  @date:    2016-4-7
;  @build:   b160421
; ---------------------------------------------------------------------------------------
;
; Copyright (c) 1997 - 2016 , Freescale Semiconductor, Inc.
; Copyright 2016-2017 NXP
; All rights reserved.
;
; THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
; IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
; IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
; STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
; IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
; THE POSSIBILITY OF SUCH DAMAGE.
;*****************************************************************************
;* Version: Cosmic C Cross Compiler                                          *
;*****************************************************************************
    xref.l _DefaultISR
    xref.l _main
    xref.l _init_data_bss
    xref.l _SystemInit
    ; Needed for data initialization
    xref.l __idesc__
    xdef Reset_Handler
    xdef __StackTop

    ;.section .isr_vector, "a"
    align 2

__isr_vector: section .text
    dc.l   __StackTop                                      ; Top of Stack
    dc.l   Reset_Handler                                   ; Reset Handler
    dc.l   NMI_Handler                                     ; NMI Handler
    dc.l   HardFault_Handler                               ; Hard Fault Handler
    dc.l   MemManage_Handler                               ; MPU Fault Handler
    dc.l   BusFault_Handler                                ; Bus Fault Handler
    dc.l   UsageFault_Handler                              ; Usage Fault Handler
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   0                                               ; Reserved
    dc.l   SVC_Handler                                     ; SVCall Handler
    dc.l   DebugMon_Handler                                ; Debug Monitor Handler
    dc.l   0                                               ; Reserved
    dc.l   PendSV_Handler                                  ; PendSV Handler
    dc.l   SysTick_Handler                                 ; SysTick Handler

                                                           ; External Interrupts
    dc.l   DMA0_IRQHandler                                 ; DMA channel 0 transfer complete
    dc.l   DMA1_IRQHandler                                 ; DMA channel 1 transfer complete
    dc.l   DMA2_IRQHandler                                 ; DMA channel 2 transfer complete
    dc.l   DMA3_IRQHandler                                 ; DMA channel 3 transfer complete
    dc.l   DMA4_IRQHandler                                 ; DMA channel 4 transfer complete
    dc.l   DMA5_IRQHandler                                 ; DMA channel 5 transfer complete
    dc.l   DMA6_IRQHandler                                 ; DMA channel 6 transfer complete
    dc.l   DMA7_IRQHandler                                 ; DMA channel 7 transfer complete
    dc.l   DMA8_IRQHandler                                 ; DMA channel 8 transfer complete
    dc.l   DMA9_IRQHandler                                 ; DMA channel 9 transfer complete
    dc.l   DMA10_IRQHandler                                ; DMA channel 10 transfer complete
    dc.l   DMA11_IRQHandler                                ; DMA channel 11 transfer complete
    dc.l   DMA12_IRQHandler                                ; DMA channel 12 transfer complete
    dc.l   DMA13_IRQHandler                                ; DMA channel 13 transfer complete
    dc.l   DMA14_IRQHandler                                ; DMA channel 14 transfer complete
    dc.l   DMA15_IRQHandler                                ; DMA channel 15 transfer complete
    dc.l   DMA_Error_IRQHandler                            ; DMA error interrupt channels 0-15
    dc.l   MCM_IRQHandler                                  ; FPU sources
    dc.l   FTFC_IRQHandler                                 ; FTFC Command complete
    dc.l   Read_Collision_IRQHandler                       ; FTFC Read collision
    dc.l   LVD_LVW_IRQHandler                              ; PMC Low voltage detect interrupt
    dc.l   FTFC_Fault_IRQHandler                           ; FTFC Double bit fault detect
    dc.l   WDOG_EWM_IRQHandler                             ; Single interrupt vector for WDOG and EWM
    dc.l   RCM_IRQHandler                                  ; RCM Asynchronous Interrupt
    dc.l   LPI2C0_Master_IRQHandler                        ; LPI2C0 Master Interrupt
    dc.l   LPI2C0_Slave_IRQHandler                         ; LPI2C0 Slave Interrupt
    dc.l   LPSPI0_IRQHandler                               ; LPSPI0 Interrupt
    dc.l   LPSPI1_IRQHandler                               ; LPSPI1 Interrupt
    dc.l   LPSPI2_IRQHandler                               ; LPSPI2 Interrupt
    dc.l   Reserved45_IRQHandler                           ; Reserved Interrupt 45
    dc.l   Reserved46_IRQHandler                           ; Reserved Interrupt 46
    dc.l   LPUART0_RxTx_IRQHandler                         ; LPUART0 Transmit / Receive Interrupt
    dc.l   Reserved48_IRQHandler                           ; Reserved Interrupt 48
    dc.l   LPUART1_RxTx_IRQHandler                         ; LPUART1 Transmit / Receive  Interrupt
    dc.l   Reserved50_IRQHandler                           ; Reserved Interrupt 50
    dc.l   LPUART2_RxTx_IRQHandler                         ; LPUART2 Transmit / Receive  Interrupt
    dc.l   Reserved52_IRQHandler                           ; Reserved Interrupt 52
    dc.l   Reserved53_IRQHandler                           ; Reserved Interrupt 53
    dc.l   Reserved54_IRQHandler                           ; Reserved Interrupt 54
    dc.l   ADC0_IRQHandler                                 ; ADC0 interrupt request.
    dc.l   ADC1_IRQHandler                                 ; ADC1 interrupt request.
    dc.l   CMP0_IRQHandler                                 ; CMP0 interrupt request
    dc.l   Reserved58_IRQHandler                           ; Reserved Interrupt 58
    dc.l   Reserved59_IRQHandler                           ; Reserved Interrupt 59
    dc.l   ERM_single_fault_IRQHandler                     ; ERM single bit error correction
    dc.l   ERM_double_fault_IRQHandler                     ; ERM double bit error non-correctable
    dc.l   RTC_IRQHandler                                  ; RTC alarm interrupt
    dc.l   RTC_Seconds_IRQHandler                          ; RTC seconds interrupt
    dc.l   LPIT0_Ch0_IRQHandler                            ; LPIT0 channel 0 overflow interrupt
    dc.l   LPIT0_Ch1_IRQHandler                            ; LPIT0 channel 1 overflow interrupt
    dc.l   LPIT0_Ch2_IRQHandler                            ; LPIT0 channel 2 overflow interrupt
    dc.l   LPIT0_Ch3_IRQHandler                            ; LPIT0 channel 3 overflow interrupt
    dc.l   PDB0_IRQHandler                                 ; PDB0 interrupt
    dc.l   Reserved69_IRQHandler                           ; Reserved Interrupt 69
    dc.l   Reserved70_IRQHandler                           ; Reserved Interrupt 70
    dc.l   Reserved71_IRQHandler                           ; Reserved Interrupt 71
    dc.l   Reserved72_IRQHandler                           ; Reserved Interrupt 72
    dc.l   SCG_IRQHandler                                  ; SCG bus interrupt request
    dc.l   LPTMR0_IRQHandler                               ; LPTIMER interrupt request
    dc.l   PORTA_IRQHandler                                ; Port A pin detect interrupt
    dc.l   PORTB_IRQHandler                                ; Port B pin detect interrupt
    dc.l   PORTC_IRQHandler                                ; Port C pin detect interrupt
    dc.l   PORTD_IRQHandler                                ; Port D pin detect interrupt
    dc.l   PORTE_IRQHandler                                ; Port E pin detect interrupt
    dc.l   SWI_IRQHandler                                  ; Software interrupt
    dc.l   Reserved81_IRQHandler                           ; Reserved Interrupt 81
    dc.l   Reserved82_IRQHandler                           ; Reserved Interrupt 82
    dc.l   Reserved83_IRQHandler                           ; Reserved Interrupt 83
    dc.l   PDB1_IRQHandler                                 ; PDB1 interrupt
    dc.l   FLEXIO_IRQHandler                               ; FlexIO Interrupt
    dc.l   Reserved86_IRQHandler                           ; Reserved Interrupt 86
    dc.l   Reserved87_IRQHandler                           ; Reserved Interrupt 87
    dc.l   Reserved88_IRQHandler                           ; Reserved Interrupt 88
    dc.l   Reserved89_IRQHandler                           ; Reserved Interrupt 89
    dc.l   Reserved90_IRQHandler                           ; Reserved Interrupt 90
    dc.l   Reserved91_IRQHandler                           ; Reserved Interrupt 91
    dc.l   Reserved92_IRQHandler                           ; Reserved Interrupt 92
    dc.l   Reserved93_IRQHandler                           ; Reserved Interrupt 93
    dc.l   CAN0_ORed_IRQHandler                            ; CAN0 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    dc.l   CAN0_Error_IRQHandler                           ; CAN0 Interrupt indicating that errors were detected on the CAN bus
    dc.l   CAN0_Wake_Up_IRQHandler                         ; CAN0 Interrupt asserted when Pretended Networking operation is enabled, and a valid message matches the selected filter criteria during Low Power mode
    dc.l   CAN0_ORed_0_15_MB_IRQHandler                    ; CAN0 OR'ed Message buffer (0-15)
    dc.l   CAN0_ORed_16_31_MB_IRQHandler                   ; CAN0 OR'ed Message buffer (16-31)
    dc.l   Reserved99_IRQHandler                           ; Reserved Interrupt 99
    dc.l   Reserved100_IRQHandler                          ; Reserved Interrupt 100
    dc.l   CAN1_ORed_IRQHandler                            ; CAN1 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    dc.l   CAN1_Error_IRQHandler                           ; CAN1 Interrupt indicating that errors were detected on the CAN bus
    dc.l   Reserved103_IRQHandler                          ; Reserved Interrupt 103
    dc.l   CAN1_ORed_0_15_MB_IRQHandler                    ; CAN1 OR'ed Interrupt for Message buffer (0-15)
    dc.l   Reserved105_IRQHandler                          ; Reserved Interrupt 105
    dc.l   Reserved106_IRQHandler                          ; Reserved Interrupt 106
    dc.l   Reserved107_IRQHandler                          ; Reserved Interrupt 107
    dc.l   CAN2_ORed_IRQHandler                            ; CAN2 OR'ed [Bus Off OR Transmit Warning OR Receive Warning]
    dc.l   CAN2_Error_IRQHandler                           ; CAN2 Interrupt indicating that errors were detected on the CAN bus
    dc.l   Reserved110_IRQHandler                          ; Reserved Interrupt 110
    dc.l   CAN2_ORed_0_15_MB_IRQHandler                    ; CAN2 OR'ed Message buffer (0-15)
    dc.l   Reserved112_IRQHandler                          ; Reserved Interrupt 112
    dc.l   Reserved113_IRQHandler                          ; Reserved Interrupt 113
    dc.l   Reserved114_IRQHandler                          ; Reserved Interrupt 114
    dc.l   FTM0_Ch0_Ch1_IRQHandler                         ; FTM0 Channel 0 and 1 interrupt
    dc.l   FTM0_Ch2_Ch3_IRQHandler                         ; FTM0 Channel 2 and 3 interrupt
    dc.l   FTM0_Ch4_Ch5_IRQHandler                         ; FTM0 Channel 4 and 5 interrupt
    dc.l   FTM0_Ch6_Ch7_IRQHandler                         ; FTM0 Channel 6 and 7 interrupt
    dc.l   FTM0_Fault_IRQHandler                           ; FTM0 Fault interrupt
    dc.l   FTM0_Ovf_Reload_IRQHandler                      ; FTM0 Counter overflow and Reload interrupt
    dc.l   FTM1_Ch0_Ch1_IRQHandler                         ; FTM1 Channel 0 and 1 interrupt
    dc.l   FTM1_Ch2_Ch3_IRQHandler                         ; FTM1 Channel 2 and 3 interrupt
    dc.l   FTM1_Ch4_Ch5_IRQHandler                         ; FTM1 Channel 4 and 5 interrupt
    dc.l   FTM1_Ch6_Ch7_IRQHandler                         ; FTM1 Channel 6 and 7 interrupt
    dc.l   FTM1_Fault_IRQHandler                           ; FTM1 Fault interrupt
    dc.l   FTM1_Ovf_Reload_IRQHandler                      ; FTM1 Counter overflow and Reload interrupt
    dc.l   FTM2_Ch0_Ch1_IRQHandler                         ; FTM2 Channel 0 and 1 interrupt
    dc.l   FTM2_Ch2_Ch3_IRQHandler                         ; FTM2 Channel 2 and 3 interrupt
    dc.l   FTM2_Ch4_Ch5_IRQHandler                         ; FTM2 Channel 4 and 5 interrupt
    dc.l   FTM2_Ch6_Ch7_IRQHandler                         ; FTM2 Channel 6 and 7 interrupt
    dc.l   FTM2_Fault_IRQHandler                           ; FTM2 Fault interrupt
    dc.l   FTM2_Ovf_Reload_IRQHandler                      ; FTM2 Counter overflow and Reload interrupt
    dc.l   FTM3_Ch0_Ch1_IRQHandler                         ; FTM3 Channel 0 and 1 interrupt
    dc.l   FTM3_Ch2_Ch3_IRQHandler                         ; FTM3 Channel 2 and 3 interrupt
    dc.l   FTM3_Ch4_Ch5_IRQHandler                         ; FTM3 Channel 4 and 5 interrupt
    dc.l   FTM3_Ch6_Ch7_IRQHandler                         ; FTM3 Channel 6 and 7 interrupt
    dc.l   FTM3_Fault_IRQHandler                           ; FTM3 Fault interrupt
    dc.l   FTM3_Ovf_Reload_IRQHandler                      ; FTM3 Counter overflow and Reload interrupt
    dc.l   _DefaultISR                                     ; 139
    dc.l   _DefaultISR                                     ; 140
    dc.l   _DefaultISR                                     ; 141
    dc.l   _DefaultISR                                     ; 142
    dc.l   _DefaultISR                                     ; 143
    dc.l   _DefaultISR                                     ; 144
    dc.l   _DefaultISR                                     ; 145
    dc.l   _DefaultISR                                     ; 146
    dc.l   _DefaultISR                                     ; 147
    dc.l   _DefaultISR                                     ; 148
    dc.l   _DefaultISR                                     ; 149
    dc.l   _DefaultISR                                     ; 150
    dc.l   _DefaultISR                                     ; 151
    dc.l   _DefaultISR                                     ; 152
    dc.l   _DefaultISR                                     ; 153
    dc.l   _DefaultISR                                     ; 154
    dc.l   _DefaultISR                                     ; 155
    dc.l   _DefaultISR                                     ; 156
    dc.l   _DefaultISR                                     ; 157
    dc.l   _DefaultISR                                     ; 158
    dc.l   _DefaultISR                                     ; 159
    dc.l   _DefaultISR                                     ; 160
    dc.l   _DefaultISR                                     ; 161
    dc.l   _DefaultISR                                     ; 162
    dc.l   _DefaultISR                                     ; 163
    dc.l   _DefaultISR                                     ; 164
    dc.l   _DefaultISR                                     ; 165
    dc.l   _DefaultISR                                     ; 166
    dc.l   _DefaultISR                                     ; 167
    dc.l   _DefaultISR                                     ; 168
    dc.l   _DefaultISR                                     ; 169
    dc.l   _DefaultISR                                     ; 170
    dc.l   _DefaultISR                                     ; 171
    dc.l   _DefaultISR                                     ; 172
    dc.l   _DefaultISR                                     ; 173
    dc.l   _DefaultISR                                     ; 174
    dc.l   _DefaultISR                                     ; 175
    dc.l   _DefaultISR                                     ; 176
    dc.l   _DefaultISR                                     ; 177
    dc.l   _DefaultISR                                     ; 178
    dc.l   _DefaultISR                                     ; 179
    dc.l   _DefaultISR                                     ; 180
    dc.l   _DefaultISR                                     ; 181
    dc.l   _DefaultISR                                     ; 182
    dc.l   _DefaultISR                                     ; 183
    dc.l   _DefaultISR                                     ; 184
    dc.l   _DefaultISR                                     ; 185
    dc.l   _DefaultISR                                     ; 186
    dc.l   _DefaultISR                                     ; 187
    dc.l   _DefaultISR                                     ; 188
    dc.l   _DefaultISR                                     ; 189
    dc.l   _DefaultISR                                     ; 190
    dc.l   _DefaultISR                                     ; 191
    dc.l   _DefaultISR                                     ; 192
    dc.l   _DefaultISR                                     ; 193
    dc.l   _DefaultISR                                     ; 194
    dc.l   _DefaultISR                                     ; 195
    dc.l   _DefaultISR                                     ; 196
    dc.l   _DefaultISR                                     ; 197
    dc.l   _DefaultISR                                     ; 198
    dc.l   _DefaultISR                                     ; 199
    dc.l   _DefaultISR                                     ; 200
    dc.l   _DefaultISR                                     ; 201
    dc.l   _DefaultISR                                     ; 202
    dc.l   _DefaultISR                                     ; 203
    dc.l   _DefaultISR                                     ; 204
    dc.l   _DefaultISR                                     ; 205
    dc.l   _DefaultISR                                     ; 206
    dc.l   _DefaultISR                                     ; 207
    dc.l   _DefaultISR                                     ; 208
    dc.l   _DefaultISR                                     ; 209
    dc.l   _DefaultISR                                     ; 210
    dc.l   _DefaultISR                                     ; 211
    dc.l   _DefaultISR                                     ; 212
    dc.l   _DefaultISR                                     ; 213
    dc.l   _DefaultISR                                     ; 214
    dc.l   _DefaultISR                                     ; 215
    dc.l   _DefaultISR                                     ; 216
    dc.l   _DefaultISR                                     ; 217
    dc.l   _DefaultISR                                     ; 218
    dc.l   _DefaultISR                                     ; 219
    dc.l   _DefaultISR                                     ; 220
    dc.l   _DefaultISR                                     ; 221
    dc.l   _DefaultISR                                     ; 222
    dc.l   _DefaultISR                                     ; 223
    dc.l   _DefaultISR                                     ; 224
    dc.l   _DefaultISR                                     ; 225
    dc.l   _DefaultISR                                     ; 226
    dc.l   _DefaultISR                                     ; 227
    dc.l   _DefaultISR                                     ; 228
    dc.l   _DefaultISR                                     ; 229
    dc.l   _DefaultISR                                     ; 230
    dc.l   _DefaultISR                                     ; 231
    dc.l   _DefaultISR                                     ; 232
    dc.l   _DefaultISR                                     ; 233
    dc.l   _DefaultISR                                     ; 234
    dc.l   _DefaultISR                                     ; 235
    dc.l   _DefaultISR                                     ; 236
    dc.l   _DefaultISR                                     ; 237
    dc.l   _DefaultISR                                     ; 238
    dc.l   _DefaultISR                                     ; 239
    dc.l   _DefaultISR                                     ; 240
    dc.l   _DefaultISR                                     ; 241
    dc.l   _DefaultISR                                     ; 242
    dc.l   _DefaultISR                                     ; 243
    dc.l   _DefaultISR                                     ; 244
    dc.l   _DefaultISR                                     ; 245
    dc.l   _DefaultISR                                     ; 246
    dc.l   _DefaultISR                                     ; 247
    dc.l   _DefaultISR                                     ; 248
    dc.l   _DefaultISR                                     ; 249
    dc.l   _DefaultISR                                     ; 250
    dc.l   _DefaultISR                                     ; 251
    dc.l   _DefaultISR                                     ; 252
    dc.l   _DefaultISR                                     ; 253
    dc.l   _DefaultISR                                     ; 254
    dc.l   0xFFFFFFFF                                      ;  Reserved for user TRIM value

; Flash Configuration
    align 4
FlashConfig: section .text
    dc.l 0xFFFFFFFF     ; 8 bytes backdoor comparison key
    dc.l 0xFFFFFFFF     ;
    dc.l 0xFFFFFFFF     ; 4 bytes program flash protection bytes
    dc.l 0xFFFF7FFE     ; FDPROT:FEPROT:FOPT:FSEC(0xFE = unsecured)

; Reset Handler

ResetH_Seg: section .text
    align 2    ; allocate an byte - so that Reset_Handler will be in thumb mode
    dc.b 1     ; FIXME: remove the above hack and specify thumb mode in makefile / lkf
Reset_Handler:
    align 2
    cpsid   i               ; Mask interrupts

    ; Init the rest of the registers
    ldr     r1,=0
    ldr     r2,=0
    ldr     r3,=0
    ldr     r4,=0
    ldr     r5,=0
    ldr     r6,=0
    ldr     r7,=0
    mov     r8,r7
    mov     r9,r7
    mov     r10,r7
    mov     r11,r7
    mov     r12,r7
    ; Initialize the stack pointer
    ldr     r0,=__StackTop
    mov     r13,r0

#ifndef __NO_SYSTEM_INIT
    ; Call the CMSIS system init routine
    bl     _SystemInit
#endif

    ; Init .data and .bss sections
    bl     _init_data_bss
    cpsie   i               ; Unmask interrupts
    bl     _main
JumpToSelf:
    b      JumpToSelf

    align 2    ; allocate an byte - so that Reset_Handler will be in thumb mode
    dc.b 1     ; FIXME: remove the above hack and specify thumb mode in makefile / lkf
_DefaultISR:
    b      _DefaultISR

;    Macro to define default handlers. Default handler
;    will be weak symbol and just dead loops. They can be
;    overwritten by other handlers
;    +1 is used to indicate thumb mode
;    -1 is an workarround
def_irq_handler: macro \handler_name
wdef _\handler_name
xref.l _\handler_name
_\handler_name : equ _DefaultISR-1
\handler_name: equ _\handler_name+1
endm

; Exception Handlers
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    DMA0_IRQHandler
    def_irq_handler    DMA1_IRQHandler
    def_irq_handler    DMA2_IRQHandler
    def_irq_handler    DMA3_IRQHandler
    def_irq_handler    DMA4_IRQHandler
    def_irq_handler    DMA5_IRQHandler
    def_irq_handler    DMA6_IRQHandler
    def_irq_handler    DMA7_IRQHandler
    def_irq_handler    DMA8_IRQHandler
    def_irq_handler    DMA9_IRQHandler
    def_irq_handler    DMA10_IRQHandler
    def_irq_handler    DMA11_IRQHandler
    def_irq_handler    DMA12_IRQHandler
    def_irq_handler    DMA13_IRQHandler
    def_irq_handler    DMA14_IRQHandler
    def_irq_handler    DMA15_IRQHandler
    def_irq_handler    DMA_Error_IRQHandler
    def_irq_handler    MCM_IRQHandler
    def_irq_handler    FTFC_IRQHandler
    def_irq_handler    Read_Collision_IRQHandler
    def_irq_handler    LVD_LVW_IRQHandler
    def_irq_handler    FTFC_Fault_IRQHandler
    def_irq_handler    WDOG_EWM_IRQHandler
    def_irq_handler    RCM_IRQHandler
    def_irq_handler    LPI2C0_Master_IRQHandler
    def_irq_handler    LPI2C0_Slave_IRQHandler
    def_irq_handler    LPSPI0_IRQHandler
    def_irq_handler    LPSPI1_IRQHandler
    def_irq_handler    LPSPI2_IRQHandler
    def_irq_handler    Reserved45_IRQHandler
    def_irq_handler    Reserved46_IRQHandler
    def_irq_handler    LPUART0_RxTx_IRQHandler
    def_irq_handler    Reserved48_IRQHandler
    def_irq_handler    LPUART1_RxTx_IRQHandler
    def_irq_handler    Reserved50_IRQHandler
    def_irq_handler    LPUART2_RxTx_IRQHandler
    def_irq_handler    Reserved52_IRQHandler
    def_irq_handler    Reserved53_IRQHandler
    def_irq_handler    Reserved54_IRQHandler
    def_irq_handler    ADC0_IRQHandler
    def_irq_handler    ADC1_IRQHandler
    def_irq_handler    CMP0_IRQHandler
    def_irq_handler    Reserved58_IRQHandler
    def_irq_handler    Reserved59_IRQHandler
    def_irq_handler    ERM_single_fault_IRQHandler
    def_irq_handler    ERM_double_fault_IRQHandler
    def_irq_handler    RTC_IRQHandler
    def_irq_handler    RTC_Seconds_IRQHandler
    def_irq_handler    LPIT0_Ch0_IRQHandler
    def_irq_handler    LPIT0_Ch1_IRQHandler
    def_irq_handler    LPIT0_Ch2_IRQHandler
    def_irq_handler    LPIT0_Ch3_IRQHandler
    def_irq_handler    PDB0_IRQHandler
    def_irq_handler    Reserved69_IRQHandler
    def_irq_handler    Reserved70_IRQHandler
    def_irq_handler    Reserved71_IRQHandler
    def_irq_handler    Reserved72_IRQHandler
    def_irq_handler    SCG_IRQHandler
    def_irq_handler    LPTMR0_IRQHandler
    def_irq_handler    PORTA_IRQHandler
    def_irq_handler    PORTB_IRQHandler
    def_irq_handler    PORTC_IRQHandler
    def_irq_handler    PORTD_IRQHandler
    def_irq_handler    PORTE_IRQHandler
    def_irq_handler    SWI_IRQHandler
    def_irq_handler    Reserved81_IRQHandler
    def_irq_handler    Reserved82_IRQHandler
    def_irq_handler    Reserved83_IRQHandler
    def_irq_handler    PDB1_IRQHandler
    def_irq_handler    FLEXIO_IRQHandler
    def_irq_handler    Reserved86_IRQHandler
    def_irq_handler    Reserved87_IRQHandler
    def_irq_handler    Reserved88_IRQHandler
    def_irq_handler    Reserved89_IRQHandler
    def_irq_handler    Reserved90_IRQHandler
    def_irq_handler    Reserved91_IRQHandler
    def_irq_handler    Reserved92_IRQHandler
    def_irq_handler    Reserved93_IRQHandler
    def_irq_handler    CAN0_ORed_IRQHandler
    def_irq_handler    CAN0_Error_IRQHandler
    def_irq_handler    CAN0_Wake_Up_IRQHandler
    def_irq_handler    CAN0_ORed_0_15_MB_IRQHandler
    def_irq_handler    CAN0_ORed_16_31_MB_IRQHandler
    def_irq_handler    Reserved99_IRQHandler
    def_irq_handler    Reserved100_IRQHandler
    def_irq_handler    CAN1_ORed_IRQHandler
    def_irq_handler    CAN1_Error_IRQHandler
    def_irq_handler    Reserved103_IRQHandler
    def_irq_handler    CAN1_ORed_0_15_MB_IRQHandler
    def_irq_handler    Reserved105_IRQHandler
    def_irq_handler    Reserved106_IRQHandler
    def_irq_handler    Reserved107_IRQHandler
    def_irq_handler    CAN2_ORed_IRQHandler
    def_irq_handler    CAN2_Error_IRQHandler
    def_irq_handler    Reserved110_IRQHandler
    def_irq_handler    CAN2_ORed_0_15_MB_IRQHandler
    def_irq_handler    Reserved112_IRQHandler
    def_irq_handler    Reserved113_IRQHandler
    def_irq_handler    Reserved114_IRQHandler
    def_irq_handler    FTM0_Ch0_Ch1_IRQHandler
    def_irq_handler    FTM0_Ch2_Ch3_IRQHandler
    def_irq_handler    FTM0_Ch4_Ch5_IRQHandler
    def_irq_handler    FTM0_Ch6_Ch7_IRQHandler
    def_irq_handler    FTM0_Fault_IRQHandler
    def_irq_handler    FTM0_Ovf_Reload_IRQHandler
    def_irq_handler    FTM1_Ch0_Ch1_IRQHandler
    def_irq_handler    FTM1_Ch2_Ch3_IRQHandler
    def_irq_handler    FTM1_Ch4_Ch5_IRQHandler
    def_irq_handler    FTM1_Ch6_Ch7_IRQHandler
    def_irq_handler    FTM1_Fault_IRQHandler
    def_irq_handler    FTM1_Ovf_Reload_IRQHandler
    def_irq_handler    FTM2_Ch0_Ch1_IRQHandler
    def_irq_handler    FTM2_Ch2_Ch3_IRQHandler
    def_irq_handler    FTM2_Ch4_Ch5_IRQHandler
    def_irq_handler    FTM2_Ch6_Ch7_IRQHandler
    def_irq_handler    FTM2_Fault_IRQHandler
    def_irq_handler    FTM2_Ovf_Reload_IRQHandler
    def_irq_handler    FTM3_Ch0_Ch1_IRQHandler
    def_irq_handler    FTM3_Ch2_Ch3_IRQHandler
    def_irq_handler    FTM3_Ch4_Ch5_IRQHandler
    def_irq_handler    FTM3_Ch6_Ch7_IRQHandler
    def_irq_handler    FTM3_Fault_IRQHandler
    def_irq_handler    FTM3_Ovf_Reload_IRQHandler

;EOF
