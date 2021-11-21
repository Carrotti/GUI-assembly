IDEAL
P386
MODEL FLAT, C
ASSUME cs:_TEXT,ds:FLAT,es:FLAT,fs:FLAT,gs:FLAT

WHITE EQU 0Fh 			; white
VMEMADR EQU 0A0000h		; video memory address
SCRWIDTH EQU 320		; screen witdth
SCRHEIGHT EQU 200		; screen height
BACKGROUNDCOL EQU 0		; background color	 

CODESEG

;--------------------------------------------------------
;STRUC
;--------------------------------------------------------

STRUC vec
	x dd ?
	y dd ?
ENDS vec

;min: top left corner, max: bottom right corner

STRUC AABB
	min	vec <>
	max vec <>
ENDS AABB

STRUC rect
	boundingBox AABB <>
	velocity vec <>
	mass dd ?
ENDS rect

STRUC manifold
	rect1 rect <>
	rect2 rect <>
	penetration dd ?
	normal vec <>
ENDS manifold

;set begin values for AABB struct

PROC initRectangle
	ARG 	@@rectPtr:dword, @@xmin:dword, @@ymin:dword, @@xmax:dword, @@ymax:dword, @@velx:dword, @@vely:dword, @@mass:dword
	USES 	eax, ebx, ecx, edx, edi

	mov eax, [@@rectPtr]
initAABB:
	mov ebx, [@@xmin]
	mov ecx, [@@ymin]
	mov edx, [@@xmax]
	mov edi, [@@ymax]
	mov [(rect eax).boundingBox.min.x], ebx
	mov [(rect eax).boundingBox.min.y], ecx
	mov [(rect eax).boundingBox.max.x], edx
	mov [(rect eax).boundingBox.max.y], edi
initVelocity:
	mov ebx, [@@velx]
	mov ecx, [@@vely]
	mov [(rect eax).velocity.x], ebx
	mov [(rect eax).velocity.y], ecx
initMass:
	mov ebx, [@@mass]
	mov [(rect eax).mass], ebx

	ret
ENDP initRectangle

;--------------------------------------------------------
;GAMELOOP PROCEDURES
;--------------------------------------------------------

;set video mode (text: 03h or VGA: 13h)

PROC setVideoMode
	ARG     @@VM:byte
	USES 	eax

	movzx ax, [@@VM]
	int 10h

	ret
ENDP setVideoMode

;setup function

PROC startGameStatus
	USES 	eax, ebx, ecx

	square rect <>
	balk rect <>
	paal rect <>
	mov [rectLst], offset square
	call initRectangle, [rectLst], 0, 10, 10, 20, 2, 1, 10
	mov ecx, 1
	mov [rectLst + 4*ecx], offset balk
	call initRectangle, [rectLst + 4*ecx], SCRWIDTH - 20, 10, SCRWIDTH, 15, -1, 2, 10
	inc ecx
	mov [rectLst + 4*ecx], offset paal
	call initRectangle, [rectLst + 4*ecx], SCRWIDTH/2, SCRHEIGHT/2, SCRWIDTH/2 + 5, SCRHEIGHT/2 + 20, 1, 2
	mov [rectNum], 3

	ret
ENDP startGameStatus

;wait until vertical blank interval is completed

PROC waitForVBI
	USES 	edx, eax

	mov dx, 03DAh
@@waitForEnd:
	in al, dx
	and al, 8
	jnz @@waitForEnd
@@waitForBegin:
	in al, dx
	and al, 8
	jz @@waitForBegin
	ret
ENDP waitForVBI

;copy buffer to actual video memory

PROC updateVideoBuffer
	USES 	esi, edi, ecx

	cld
	mov esi, offset screenBuffer
	mov edi, 0A0000h
	mov ecx, 64000/4
	rep movsd

	ret
ENDP

;if esc is pressed, terminate process

PROC handleInput
	USES 	eax

	mov ah, 01h
	int 16h
	jz @@noKeyPressed

@@keyPressed:
	mov ah, 00h
	int 16h
	cmp ah, 1h
	jz @@escPressed
	ret

@@escPressed:
	call terminateProcess
	ret

@@noKeyPressed:
	ret

ENDP handleInput

;update loop

PROC updateGameStatus

	call moveAllRects
	call checkAllIfHit
	call checkAllCollisions

	ret
ENDP updateGameStatus

;check if rect hit border

PROC checkIfHit
	ARG 	@@rect1Ptr:dword
	USES 	eax, ebx, ecx, edx

	mov edx, [@@rect1Ptr]
	;check if hit borders
checkHorizontal:
	mov eax, [(rect edx).boundingBox.min.x]
	add eax, [(rect edx).velocity.x]
	cmp eax, 0
	jle hitLeftBorder
	mov eax, [(rect edx).boundingBox.max.x]
	add eax, [(rect edx).velocity.x]
	cmp eax, SCRWIDTH
	jge hitRightBorder
	jmp checkVertical
hitLeftBorder:
hitRightBorder:
	mov eax, [(rect edx).velocity.x]
	imul eax, -1
	mov [(rect edx).velocity.x], eax
checkVertical:
	mov eax, [(rect edx).boundingBox.min.y]
	add eax, [(rect edx).velocity.y]
	cmp eax, 0
	jle hitTopBorder
	mov eax, [(rect edx).boundingBox.max.y]
	add eax, [(rect edx).velocity.y]
	cmp eax, SCRHEIGHT
	jge hitBottomBorder
	jmp noBorder
hitTopBorder:
hitBottomBorder:
	mov eax, [(rect edx).velocity.y]
	imul eax, -1
	mov [(rect edx).velocity.y], eax
noBorder:
	ret
ENDP checkIfHit

PROC checkAllIfHit
	USES	eax, ecx

	mov ecx, [rectNum]
	mov eax, 0
keepChecking:
	call checkIfHit, [rectLst + 4*eax]
	inc eax
	loop keepChecking
doneChecking:
	ret
ENDP checkAllIfHit

;collision detection

PROC checkCollision
	ARG 	@@rect1Ptr:dword, @@rect2Ptr:dword
	USES eax, ebx, edx, ecx

	mov eax, [@@rect1Ptr]
	mov ebx, [@@rect2Ptr]

	mov ecx, [(rect ebx).boundingBox.min.x]
	add ecx, [(rect ebx).velocity.x]
	mov edx, [(rect eax).boundingBox.max.x]
	add edx, [(rect eax).velocity.x]
	cmp edx, ecx
	jle noIntersection
	mov ecx, [(rect ebx).boundingBox.max.x]
	add ecx, [(rect ebx).velocity.x]
	mov edx, [(rect eax).boundingBox.min.x]
	add edx, [(rect eax).velocity.x]
	cmp edx, ecx
	jge noIntersection
	mov ecx, [(rect ebx).boundingBox.min.y]
	add ecx, [(rect ebx).velocity.y]
	mov edx, [(rect eax).boundingBox.max.y]
	add edx, [(rect eax).velocity.y]
	cmp edx, ecx
	jle noIntersection
	mov ecx, [(rect ebx).boundingBox.max.y]
	add ecx, [(rect ebx).velocity.y]
	mov edx, [(rect eax).boundingBox.min.y]
	add edx, [(rect eax).velocity.y]
	cmp edx, ecx
	jge noIntersection
	jmp intersection
noIntersection:
	ret
intersection:
	mov [(rect eax).velocity.x], 0
	mov [(rect eax).velocity.y], 0
	mov [(rect ebx).velocity.x], 0
	mov [(rect ebx).velocity.y], 0
	ret
ENDP checkCollision

PROC checkAllCollisions
	USES 	eax, ebx, ecx, edx

	mov eax, 0
	mov ebx, 1
	mov ecx, [rectNum]
	sub ecx, 1
collisionLoop:
	push ecx
	mov ecx, [rectNum]
	sub ecx, ebx
checkLoop:
	call checkCollision, [rectLst + 4*eax], [rectLst + 4*ebx]
	inc ebx
	loop checkLoop
	pop ecx
	inc eax
	mov ebx, eax
	inc ebx
	loop collisionLoop
stopCheckingLoop:
	ret
ENDP checkAllCollisions

PROC impulseResolution
	ARG 	@@rect1Ptr:dword, @@rect2Ptr:dword
	USES 	eax, ebx, ecx, edx



	ret
ENDP impulseResolution

;wait until user enters a key

PROC waitForSpecificKeystroke
	ARG 	@@key:byte
	USES 	eax

waitForKeystroke:
	mov	ah, 00h	
	int	16h
	cmp	al, [@@key]
	jne	waitForKeystroke
	ret
ENDP waitForSpecificKeystroke

;set video mode to text mode

PROC terminateProcess
	USES    eax

	call setVideoMode, 03h
	mov	ax, 04C00h
	int 21h

	ret
ENDP terminateProcess

;--------------------------------------------------------
;DRAWING PROCEDURES
;--------------------------------------------------------

;Change color of a single pixel, given a coördinate (x: [0; 320], y: [0; 200]) and a color [00h; FFh]

PROC setPixel
	ARG 	@@x:dword, @@y:dword, @@col:byte
	USES 	edi, eax, ecx, edx

	mov eax, [@@y]
	mov edx, SCRWIDTH
	mul edx				;calculate row offset
	add	eax, [@@x]		;calculate column offset
	mov edi, offset screenBuffer
	add edi, eax		;calculate coordinate idx
    mov al, [@@col]
    stosb				;store color in screenBuffer at idx edi

	ret
ENDP setPixel

;draw AABB struc, given a color

PROC drawRectangle
	ARG 	@@rectPtr:dword, @@col:byte
	USES 	eax, ebx, ecx, edx, edi ; note: MUL uses edx!

	; Compute the index of the rectangle's top left corner
	mov ebx, [@@rectPtr]
	mov eax, [(rect ebx).boundingBox.min.y]
	mov edx, SCRWIDTH
	mul edx
	add	eax, [(rect ebx).boundingBox.min.x]

	; Compute top left corner address
	mov edi, offset screenBuffer
	add edi, eax
	
	; Plot the top horizontal edge.
	mov edx, [(rect ebx).boundingBox.max.x]
	sub edx, [(rect ebx).boundingBox.min.x] ; store width in edx for later reuse
	mov	ecx, edx
	mov	al, [@@col]
	rep stosb
	sub edi, edx		; reset edi to left-top corner
	
	; plot both vertical edges
	mov ecx, [(rect ebx).boundingBox.max.y]
	sub ecx, [(rect ebx).boundingBox.min.y]
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

;--------------------------------------------------------
;PHYSICS
;--------------------------------------------------------

;move a rectangle (draw same rectangle in black, move and draw again)

PROC moveRectangle
	ARG		@@rectPtr:dword
	USES 	eax, ebx, ecx, edx, edi

	mov eax, [@@rectPtr]
	call drawRectangle, eax, BACKGROUNDCOL
	mov ecx, [(rect eax).velocity.x]
	mov edx, [(rect eax).velocity.y]
	add [(rect eax).boundingBox.min.x], ecx
	add [(rect eax).boundingBox.min.y], edx
	add [(rect eax).boundingBox.max.x], ecx
	add [(rect eax).boundingBox.max.y], edx
	call drawRectangle, eax, WHITE
	ret
ENDP moveRectangle

PROC moveAllRects
	USES	eax, ecx

	mov ecx, [rectNum]
	mov eax, 0
keepMoving:
	call moveRectangle, [rectLst + 4*eax]
	inc eax
	loop keepMoving
doneMoving:
	ret
ENDP moveAllRects

;--------------------------------------------------------
;MAIN
;--------------------------------------------------------

PROC main

    sti
    cld

    push ds
    pop es

    call setVideoMode, 13h
	call startGameStatus

gameLoop:
	call waitForVBI
	call updateVideoBuffer
	call handleInput
	call updateGameStatus
	jnz gameLoop
dead:

ENDP main

;--------------------------------------------------------
;DATA
;--------------------------------------------------------

DATASEG

	screenBuffer 	db 64000 dup(0), '$'
	rectLst 		dd 64 dup(?), '$'
	rectNum 		dd 2

STACK 100h

END main