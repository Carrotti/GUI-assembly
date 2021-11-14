IDEAL
P386
MODEL FLAT, C
ASSUME cs:_TEXT,ds:FLAT,es:FLAT,fs:FLAT,gs:FLAT

; compile-time constants (with macros)
VMEMADR EQU 0A0000h	; video memory address
SCRWIDTH EQU 320	; screen witdth
SCRHEIGHT EQU 200	; screen height
BACKGROUNDCOL EQU 0 ; background color	 
WHITE EQU 63 		; white

; -------------------------------------------------------------------
CODESEG

; Set the video mode
PROC setVideoMode
	ARG 	@@VM:byte
	USES 	eax

	movzx ax,[@@VM]
	int 10h

	ret
ENDP setVideoMode

; Update the colour palette.
; 	* Ncolours: number of colours that have to be updated [word]
PROC updateColourPalette
	ARG	 	@@Ncolours: word
	USES 	eax, ecx, edx, esi

	mov esi, offset palette	; pointer to source palette
	movzx ecx, [@@Ncolours] ; amount of colors to read (movzx = zero extend)
	
	; multiply ecx by 3 (three color components per color)
	lea ecx, [2*ecx + ecx]	; do it efficiently 3*ecx = 2*ecx + ecx
		; ; You can also do:
		; mov eax, ecx
		; sal eax, 1
		; add ecx, eax

	mov dx, 03C8h 	; DAC write port
	xor al, al		; index of first color to change (0)
	out dx, al		; write to IO

	inc dx
	rep outsb		; update all colors

	ret

ENDP updateColourPalette



PROC fillBackground
	ARG 	@@fillcolor:byte
	USES 	eax, ecx, edi

	; Initialize video memory address.
	mov	edi, VMEMADR
	
	; copy color value across all bytes of eax
	mov al, [@@fillcolor]	; ???B
	mov ah, al				; ??BB
	mov cx, ax			
	shl eax, 16				; BB00
	mov ax, cx				; BBBB

	; Scan the whole video memory and assign the background colour.
	mov	ecx, SCRWIDTH*SCRHEIGHT/4
	rep	stosd

	ret
ENDP fillBackground

PROC displayString
    ARG @@row:DWORD, @@column:DWORD, @@offset:DWORD
    USES EAX, EBX, EDX
    MOV EDX, [@@row]    ; row in EDX
    MOV EBX, [@@column] ; column in EBX
    MOV AH, 02H
    SHL EDX, 08H
    MOV DL, BL
    MOV BH, 0
    INT 10H
    MOV AH, 09H
    MOV EDX, [@@offset]
    INT 21H
    RET
ENDP displayString

PROC setPixel
	ARG @@screenX:byte, @@screenY:byte
	USES eax, ecx, edx, edi

	;calc index of pixel
	movzx eax, [@@screenY]
	mov edx, SCRWIDTH
	mul edx
	add al, [@@screenX]

	;calc address of index
	mov edi, VMEMADR
	add edi, eax
	mov eax, 63
	stosb
	ret
ENDP setPixel

PROC drawRectangle
	ARG 	@@x0:word, @@y0:word, @@w:word, @@h:word, @@col: byte
	USES 	eax, ecx, edx, edi ; note: MUL uses edx!

	; Compute the index of the rectangle's top left corner
	movzx eax, [@@y0]
	mov edx, SCRWIDTH
	mul edx
	add	ax, [@@x0]

	; Compute top left corner address
	mov edi, VMEMADR
	add edi, eax
	
	; Plot the top horizontal edge.
	movzx edx, [@@w]	; store width in edx for later reuse
	mov	ecx, edx
	mov	al,[@@col]
	rep stosb
	sub edi, edx		; reset edi to left-top corner
	
	; plot both vertical edges
	movzx ecx,[@@h]
	@@vertLoop:
		mov	[edi],al		; left edge
		mov	[edi+edx-1],al	; right edge
		add	edi, SCRWIDTH
		loop @@vertLoop
	; edi should point at the bottom-left corner now
	sub edi, SCRWIDTH

	; Plot the bottom horizontal edge.
	mov	ecx, edx
	rep stosb
	ret
ENDP drawRectangle

;dir = [0; 360[
;speed = pixels / frame (later per second maybe)



; Wait for a specific keystroke.
PROC waitForSpecificKeystroke
	ARG 	@@key:byte
	USES 	eax

	@@waitForKeystroke:
		mov	ah,00h
		int	16h
		cmp	al,[@@key]
	jne	@@waitForKeystroke

	ret
ENDP waitForSpecificKeystroke

; Terminate the program.
PROC terminateProcess
	USES eax
	call setVideoMode, 03h
	mov	ax,04C00h
	int 21h
	ret
ENDP terminateProcess

PROC main
	sti
	cld
	
	push ds
	pop	es
	
	call	setVideoMode,13h
	call	fillBackground, 0
	
	;call 	makePalette
	call	updateColourPalette, 256
	call 	displayString, 50, 50, offset teststr
	call 	setPixel, 50, 50
	;call	drawGradient

	;mov [squareX], 60
	;mov [squareY], 60
	;mov ax, [squareX]
	;mov bx, [squareY]
	;mov cx, 1

	;call	drawRectangle,[word ptr squareY],[word ptr squareX],60,60,cx
	;call 	waitForSpecificKeystroke, 0020h
	;call	moveRectangle,60,60,60,60, 0, 10, 10
	;call	drawRectangle,80,80,100,100, 63
	;call	drawRectangle,1,1,SCRWIDTH-2,SCRHEIGHT-2, 48
	
	call	waitForSpecificKeystroke, 001Bh
	call	terminateProcess
ENDP main

; -------------------------------------------------------------------
DATASEG
	palette db 768 dup (?)
	squareX	dw 60
	squareY	dw 60
	teststr dw "H", 13, 10, '$'
; -------------------------------------------------------------------
; STACK
; -------------------------------------------------------------------
STACK 100h

END main

/*PROC drawRectangle
	USES 	eax, ebx, ecx, edx, edi ; note: MUL uses edx!

	; Compute the index of the rectangle's top left corner
	mov ebx, []
	mov eax, [ebx.y]
	mov edx, SCRWIDTH
	mul edx
	add	ax, [@@rectangle.x]

	; Compute top left corner address
	mov edi, offset screenBuffer
	add edi, eax
	
	; Plot the top horizontal edge.
	movzx edx, [@@rectangle.w]	; store width in edx for later reuse
	mov	ecx, edx
	mov	al, [@@col]
	rep stosb
	sub edi, edx		; reset edi to left-top corner
	
	; plot both vertical edges
	movzx ecx, [@@rectangle.h]
	@@vertLoop:
		mov	[edi], al		; left edge
		mov	[edi+edx-1], al	; right edge
		add	edi, SCRWIDTH
		loop @@vertLoop
	; edi should point at the bottom-left corner now
	sub edi, SCRWIDTH

	; Plot the bottom horizontal edge.
	mov	ecx, edx
	rep stosb

	ret
ENDP drawRectangle

PROC moveRectangle
	ARG		@@prevX:word:1, @@prevY:word:1, @@w:word:1, @@h:word:1, @@dir:word:1, @@speed:word:1, @@col:byte 
	USES 	eax, ecx, edx, edi

	mov eax, offset my_square
	call	drawRectangle, eax, BACKGROUNDCOL
	mov ax, [my_square.x]
	mov bx, [my_square.y]
	mov cx, [@@dir]
	mov dx, [@@speed]
	call 	drawRectangle, eax, WHITE

	ret
ENDP moveRectangle*/