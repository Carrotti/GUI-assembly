IDEAL
P386
MODEL FLAT, C
ASSUME cs:_TEXT,ds:FLAT,es:FLAT,fs:FLAT,gs:FLAT

WHITE EQU 0Fh 			;white (row: 0, col: F) using default colour palette
VMEMADR EQU 0A0000h		;video memory address
SCRWIDTH EQU 320		;screen width
SCRHEIGHT EQU 200		;screen height
BACKGROUNDCOL EQU 00h	;background color (row: 0, col: 0)

CODESEG

;--------------------------------------------------------
;STRUC
;--------------------------------------------------------

;2D vector (coördinate)

STRUC vec
	x dd ?
	y dd ?
ENDS vec

;axis aligned bounding box (no rotation)
;min: top left corner, max: bottom right corner

STRUC AABB
	min	vec <>
	max vec <>
ENDS AABB


STRUC circle
	radius dd ?
	position vec <>
	velocity vec <>
	mass dd ?
	restitution dd ?
ENDS circle

;rectangle
STRUC rect
	box AABB <>
	velocity vec <>
	mass dd ?
	restitution dd ?
ENDS rect

;manifold
;collection of points (intersection of 2 rectangles)
;used to compute penetration and a normal vector to later compute impulse resolution with

STRUC manifold
	rect1 rect <>
	rect2 rect <>
	circ1 circle <>
	circ2 circle <>
	penetration dd ?
	normal vec <>
ENDS manifold

;--------------------------------------------------------
;SETUP PROCEDURES
;--------------------------------------------------------
PROC initCircle
	ARG 	@@circPtr:dword, @@x:dword, @@y:dword, @@radius:dword, @@velx:dword, @@vely:dword, @@mass:dword, @@restitution:dword
	USES	eax, ebx

	;put circle into circLst and increment circNum
	mov ebx, [circNum]
	mov eax, [@@circPtr]
	mov [circLst + 4*ebx], eax
	inc ebx
	mov [circNum], ebx
	;enter values into circle
	mov ebx, [@@x]
	mov [(circle eax).position.x], ebx
	mov ebx, [@@y]
	mov [(circle eax).position.y], ebx
	mov ebx, [@@radius]
	mov [(circle eax).radius], ebx
	mov ebx, [@@velx]
	mov [(circle eax).velocity.x], ebx
	mov ebx, [@@vely]
	mov [(circle eax).velocity.y], ebx
	mov ebx, [@@mass]
	mov [(circle eax).mass], ebx
	mov ebx, [@@restitution]
	mov [(circle eax).restitution], ebx
	ret
ENDP initCircle



;set begin values for AABB struct in a rectangle

PROC initRectangle
	ARG 	@@rectPtr:dword, @@xmin:dword, @@ymin:dword, @@xmax:dword, @@ymax:dword, @@velx:dword, @@vely:dword, @@mass:dword, @@restitution:dword
	USES 	eax, ebx, ecx, edx, edi


	mov ebx, [rectNum]
	mov eax, [@@rectPtr]
	mov [rectLst + 4*ebx], eax
	inc ebx
	mov [rectNum], ebx
initAABB:
	mov ebx, [@@xmin]
	mov ecx, [@@ymin]
	mov edx, [@@xmax]
	mov edi, [@@ymax]
	mov [(rect eax).box.min.x], ebx
	mov [(rect eax).box.min.y], ecx
	mov [(rect eax).box.max.x], edx
	mov [(rect eax).box.max.y], edi
initVelocity:
	mov ebx, [@@velx]
	mov ecx, [@@vely]
	mov [(rect eax).velocity.x], ebx
	mov [(rect eax).velocity.y], ecx
initMass:
	mov ebx, [@@mass]
	mov [(rect eax).mass], ebx
initRestitution:
	mov ebx, [@@restitution]
	mov [(rect eax).restitution], ebx

	ret
ENDP initRectangle

;--------------------------------------------------------
;HELP PROCEDURES
;--------------------------------------------------------

;calculate square root and store in eax
;using binary search algorithm
PROC sqrt
	ARG 	@@y:dword
	USES	ebx, ecx, edx, edi

	mov ebx, 0 
	mov ecx, [@@y] 
	inc ecx     
binarySearch:
	dec ecx     
	cmp ebx, ecx
	je exitLoop 
	inc ecx		
	mov eax, ebx
	add eax, ecx
	mov edx, 0  
	mov edi, 2  
	div edi	
	mov edi, eax
	mov eax, [@@y]
	mov edx, 0
	div edi
	cmp edi, eax
	mov eax, edi
	jle lessOrEqual
	mov ecx, eax
	jmp binarySearch
lessOrEqual:
	mov ebx, eax
	jmp binarySearch
exitLoop:
	mov eax, ebx
	ret
ENDP sqrt

;calculates distance between two points and stores it in eax
PROC distance
	ARG		@@vec1Ptr:dword, @@vec2Ptr:dword
	USES	ebx, ecx, edx

	mov ebx, [@@vec1Ptr]
	mov ecx, [@@vec2Ptr]
	mov edx, [(vec ebx).x]
	mov eax, [(vec ecx).x]
	sub eax, edx
	mul eax
	push eax
	mov edx, [(vec ebx).y]
	mov eax, [(vec ecx).y]
	sub eax, edx
	mul eax
	pop edx
	add eax, edx
	call sqrt, eax
	ret
ENDP distance

;--------------------------------------------------------
;PHYSICS
;--------------------------------------------------------

PROC moveCircle
	ARG 	@@circPtr:dword
	USES	eax, ebx

	mov eax, [@@circPtr]
	call drawCircle, eax, BACKGROUNDCOL
	mov ebx, [(circle eax).velocity.x]
	add [(circle eax).position.x], ebx
	mov ebx, [(circle eax).velocity.y]
	add [(circle eax).position.y], ebx
	call drawCircle, [@@circPtr], WHITE
	ret

ENDP moveCircle
;move a rectangle (draw same rectangle in black, move and draw again)

PROC moveRectangle
	ARG		@@rectPtr:dword
	USES 	eax, ebx

	mov eax, [@@rectPtr]
	call drawRectangle, eax, BACKGROUNDCOL
	mov ebx, [(rect eax).velocity.x]
	add [(rect eax).box.min.x], ebx
	add [(rect eax).box.max.x], ebx
	mov ebx, [(rect eax).velocity.y]
	add [(rect eax).box.min.y], ebx
	add [(rect eax).box.max.y], ebx
	call drawRectangle, eax, WHITE
	ret
ENDP moveRectangle

;map moveCircle on circLst, then map moveRectangle on rectLst
PROC moveAllObjects
	USES 	eax, ecx
moveCircles:
	mov ecx, [circNum]
	cmp ecx, 0
	jle moveRects
	mov eax, 0
moveNextCircle:
	call moveCircle, [circLst + 4*eax]
	inc eax
	loop moveNextCircle
moveRects:
	mov ecx, [rectNum]
	cmp ecx, 0
	jle doneMoving
	mov eax, 0
moveNextRect:
	call moveRectangle, [rectLst + 4*eax]
	inc eax
	loop moveNextRect
doneMoving:
	ret
ENDP moveAllObjects

;given a circle, check if it has hit a border and change direction if necessary
PROC circHitBorder
	ARG 	@@circPtr:dword
	USES	eax, ebx

	mov eax, [@@circPtr]
leftBorder:
	mov ebx, [(circle eax).position.x]
	sub ebx, [(circle eax).radius]
	cmp ebx, 0
	jl leftBorderHit
rightBorder:
	mov ebx, [(circle eax).position.x]
	add ebx, [(circle eax).radius]
	cmp ebx, SCRWIDTH
	jg rightBorderHit
topBorder:
	mov ebx, [(circle eax).position.y]
	sub ebx, [(circle eax).radius]
	cmp ebx, 0
	jl topBorderHit
bottomBorder:
	mov ebx, [(circle eax).position.y]
	add ebx, [(circle eax).radius]
	cmp ebx, SCRHEIGHT
	jg bottomBorderHit
	jmp exit
leftBorderHit:
	mov ebx, [(circle eax).velocity.x]
	cmp ebx, 0
	jg topBorder
	imul ebx, -1
	mov [(circle eax).velocity.x], ebx
	jmp topBorder
rightBorderHit:
	mov ebx, [(circle eax).velocity.x]
	cmp ebx, 0
	jl topBorder
	imul ebx, -1
	mov [(circle eax).velocity.x], ebx
	jmp topBorder
topBorderHit:
	mov ebx, [(circle eax).velocity.y]
	cmp ebx, 0
	jg exit
	imul ebx, -1
	mov [(circle eax).velocity.y], ebx
	jmp exit
bottomBorderHit:
	mov ebx, [(circle eax).velocity.y]
	cmp ebx, 0
	jl exit
	imul ebx, -1
	mov [(circle eax).velocity.y], ebx
exit:
	ret
ENDP circHitBorder
;check if a rectangle hit a border
;if so, bounce off

PROC rectHitBorder
	ARG 	@@rect1Ptr:dword
	USES 	eax, ebx, ecx, edx

	mov edx, [@@rect1Ptr]
	;check if hit borders
checkHorizontal:
	mov eax, [(rect edx).box.min.x]
	add eax, [(rect edx).velocity.x]
	cmp eax, 0
	jle hitLeftBorder
	mov eax, [(rect edx).box.max.x]
	add eax, [(rect edx).velocity.x]
	cmp eax, SCRWIDTH
	jge hitRightBorder
	jmp checkVertical
hitLeftBorder:
hitRightBorder:
	mov eax, [(rect edx).velocity.x]
	imul eax, -1
	cmp eax, 0
	jl @@neg1
	sub eax, 1 ;if positive velocity, subtract 1
	jmp @@pos1
@@neg1:
	add eax, 1 ;if negative velocity, add 1
@@pos1:
	mov [(rect edx).velocity.x], eax
checkVertical:
	mov eax, [(rect edx).box.min.y]
	add eax, [(rect edx).velocity.y]
	cmp eax, 0
	jle hitTopBorder
	mov eax, [(rect edx).box.max.y]
	add eax, [(rect edx).velocity.y]
	cmp eax, SCRHEIGHT
	jge hitBottomBorder
	jmp noBorder
hitTopBorder:
hitBottomBorder:
	mov eax, [(rect edx).velocity.y]
	imul eax, -1
	cmp eax, 0
	jl @@neg2
	sub eax, 1
	jmp @@pos2
@@neg2:
	add eax, 1
@@pos2:
	mov [(rect edx).velocity.y], eax
noBorder:
	ret
ENDP rectHitBorder

;map circHitBorder on circLst
;map rectHitBorder on rectLst 

PROC allHitBorder
	USES	eax, ecx
checkCircles:
	mov ecx, [circNum]
	cmp ecx, 0
	jle checkRects
	mov eax, 0
checkNextCircle:
	call circHitBorder, [circLst + 4*eax]
	inc eax
	loop checkNextCircle
checkRects:
	mov ecx, [rectNum]
	cmp ecx, 0
	jle doneChecking
	mov eax, 0
checkNextRect:
	call rectHitBorder, [rectLst + 4*eax]
	inc eax
	loop checkNextRect
doneChecking:
	ret
ENDP allHitBorder

;collision detection
;using the separating axis theorem:
;if you are able to draw a line to separate two convex polygons, then they do not collide

PROC checkRectCollision
	ARG 	@@rect1Ptr:dword, @@rect2Ptr:dword
	USES eax, ebx, edx, ecx

	mov eax, [@@rect1Ptr]
	mov ebx, [@@rect2Ptr]

	mov ecx, [(rect ebx).box.min.x]
	mov edx, [(rect eax).box.max.x]
	cmp edx, ecx
	jle noIntersection
	mov ecx, [(rect ebx).box.max.x]
	mov edx, [(rect eax).box.min.x]
	cmp edx, ecx
	jge noIntersection
	mov ecx, [(rect ebx).box.min.y]
	mov edx, [(rect eax).box.max.y]
	cmp edx, ecx
	jle noIntersection
	mov ecx, [(rect ebx).box.max.y]
	mov edx, [(rect eax).box.min.y]
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
ENDP checkRectCollision

;map checkCollision on rectLst

PROC checkAllCollisions
	USES 	eax, ebx, ecx, edx

	mov eax, 0
	mov ebx, 1
	mov ecx, [rectNum]
	sub ecx, 1
@@outerLoop:
	push ecx
	mov ecx, [rectNum]
	sub ecx, ebx
@@innerLoop:
	call checkRectCollision, [rectLst + 4*eax], [rectLst + 4*ebx]
	inc ebx
	loop @@innerLoop
	pop ecx
	inc eax
	mov ebx, eax
	inc ebx
	loop @@outerLoop
stopCheckingLoop:
	ret
ENDP checkAllCollisions

;impulse resolution



;calculate manifold for 2 circles

PROC CirclevsCircle
	ARG 	@@manifoldPtr:dword
	USES	eax, ebx, ecx, edx

	mov ecx, [@@manifoldPtr]
	mov ebx, [(manifold ecx).circ1.position.x]
	mov eax, [(manifold ecx).circ2.position.x]
	sub eax, ebx ;eax = delta x
	mov [vecAB.x], eax
	mov ebx, [(manifold ecx).circ1.position.y]
	mov eax, [(manifold ecx).circ2.position.y]
	sub eax, ebx ;eax = delta y
	mov [vecAB.y], eax
	mul eax
	push eax
	mov eax, [vecAB.x]
	mul eax		;eax = square(delta x)
	pop ebx		;ebx = square(delta y)
	add ebx, eax;ebx = square(distance(circ1, circ2))
	mov eax, [(manifold ecx).circ1.radius]
	add eax, [(manifold ecx).circ2.radius]
	mov edx, eax;edx = rad1 + rad2
	mul eax 	;eax = square(rad1 + rad2)
	cmp eax, ebx 
	jl notTouching 	;return if circles not touching
	call sqrt, ebx ;eax = distance(circ1, circ2)
	cmp eax, 0
	jne notSamePosition
samePosition:
	mov eax, [(manifold ecx).circ1.radius]
	mov [(manifold ecx).penetration], eax;penetration is radius of either circle
	mov [(manifold ecx).normal.x], 0
	mov [(manifold ecx).normal.y], 1 ;choose any unit vector as normal
	ret
notSamePosition:
	sub edx, eax
	mov [(manifold ecx).penetration], edx;penetration = rad1+rad2-distance
	mov ebx, eax
	mov eax, [vecAB.x]
	div ebx
	mov [(manifold ecx).normal.x], eax
	mov eax, [vecAB.y]
	div ebx
	mov [(manifold ecx).normal.y], eax;normal is a unit vector in the direction of vecAB
	ret
notTouching:
	ret
ENDP CirclevsCircle

;calculate manifold for 2 rectangles
PROC AABBvsAABB
	ARG		@@manifoldPtr:dword
	USES 	eax, ebx, ecx, edx

	mov ecx, [@@manifoldPtr]
	mov eax, [(manifold ecx).rect1.box.min.x]
	mov ebx, [(manifold ecx).rect2.box.min.x]
	sub ebx, eax
	push ebx
	mov eax, [(manifold ecx).rect1.box.min.y]
	mov ebx, [(manifold ecx).rect2.box.min.y]
	sub ebx, eax
	pop eax
	mov [vecAB.x], eax
	mov [vecAB.y], ebx

	mov ebx, [(manifold ecx).rect1.box.max.x]
	sub ebx, eax
	mov eax, ebx
	mov ebx, 2
	xor edx, edx
	div ebx
	push eax ;result of half extent along x axis rect1 in eax

	mov ebx, [(manifold ecx).rect2.box.max.x]
	mov eax, [(manifold ecx).rect2.box.min.x]
	sub ebx, eax
	mov eax, ebx
	mov ebx, 2
	xor edx, edx
	div ebx ;result of half extent along x axis rect2 in eax

	pop ebx
	add eax, ebx ;added half extents
	mov ebx, [vecAB.x]
	cmp ebx, 0
	jg @@alreadyPositive
	imul ebx, -1
@@alreadyPositive:
	sub eax, ebx ;x overlap
	cmp eax, 0
	jle @@return
	push eax

	mov eax, [(manifold ecx).rect1.box.min.y]
	mov ebx, [(manifold ecx).rect1.box.max.y]
	sub ebx, eax
	mov eax, ebx
	mov ebx, 2
	xor edx, edx
	div ebx
	push eax ;result of half extent along y axis rect1 in eax

	mov eax, [(manifold ecx).rect2.box.min.y]
	mov ebx, [(manifold ecx).rect2.box.max.y]
	sub ebx, eax
	mov eax, ebx
	mov ebx, 2
	xor edx, edx
	div ebx ;result of half extent along x axis rect2 in eax

	pop ebx 
	add eax, ebx ;added half extents
	mov ebx, [vecAB.y]
	cmp ebx, 0
	jg @@alreadyPositive1
	imul ebx, -1
@@alreadyPositive1:
	sub eax, ebx ;y overlap
	cmp eax, 0
	jle short @@return

	pop ebx ;x overlap
	cmp eax, ebx
	jg @@alongY
@@alongX:
	cmp [vecAB.x], 0
	jge @@rightSide
@@leftSide:
	mov [(manifold ecx).normal.x], -1
	mov [(manifold ecx).normal.y], 0
	jmp @@setPenetrationToXOverlap
@@rightSide:
	mov [(manifold ecx).normal.x], 1
	mov [(manifold ecx).normal.y], 0
@@setPenetrationToXOverlap:
	mov [(manifold ecx).penetration], ebx
	jmp @@return

@@alongY:
	cmp [vecAB.y], 0
	jge @@bottomSide
@@topSide:
	mov [(manifold ecx).normal.x], 0
	mov [(manifold ecx).normal.y], 1
	jmp @@setPenetrationToYOverlap
@@bottomSide:
	mov [(manifold ecx).normal.x], 0
	mov [(manifold ecx).normal.y], -1
@@setPenetrationToYOverlap:
	mov [(manifold ecx).penetration], eax

@@return:
	ret
ENDP AABBvsAABB

PROC applyGravity
	ARG @@rectPtr:dword
	USES eax, ebx, ecx, edx

	mov eax, [timeStamp]
	mov ebx, 10 
	xor edx, edx
	div ebx
	cmp edx, 0
	jne @@noSecondElapsed
	mov ebx, [@@rectPtr]
	mov ecx, [(rect ebx).velocity.y]
	add ecx, 2
	mov [(rect ebx).velocity.y], ecx
@@noSecondElapsed:
	ret
ENDP applyGravity

PROC applyAllGravity
	USES eax, ebx, ecx

	mov ecx, [rectNum]
	mov eax, 0
@@keepChecking:
	call applyGravity, [rectLst + 4*eax]
	inc eax
	loop @@keepChecking
@@doneChecking:
	ret
ENDP applyAllGravity

;--------------------------------------------------------
;DRAWING PROCEDURES
;--------------------------------------------------------

;Change color of a single pixel, given a coördinate (x: [0; 320], y: [0; 200]) and a color [00h; FFh]

PROC setPixel
	ARG 	@@x:dword, @@y:dword, @@col:byte
	USES 	edi, eax, ecx, edx

	;check if the pixel is in the bounds first
	mov eax, [@@x]
	cmp eax, SCRWIDTH
	ja dontSetPixel
	mov eax, [@@y]
	cmp eax, SCRHEIGHT
	ja dontSetPixel
	mov edx, SCRWIDTH
	mul edx				;calculate row offset
	add	eax, [@@x]		;calculate column offset
	mov edi, offset screenBuffer
	add edi, eax		;calculate coordinate idx
    mov al, [@@col]
    stosb				;store color in screenBuffer at idx edi
dontSetPixel:
	ret
ENDP setPixel

;draw circle struct using mid-point circle algorithm 
PROC drawCircle
	ARG		@@circPtr:dword, @@col:byte
	USES 	eax, ebx, ecx, edx, esi, edi

	mov eax, [@@circPtr]
	mov ebx, [(circle eax).radius]	   ;r
	mov ecx, [(circle eax).position.x] ;x_center
	mov edx, [(circle eax).position.y] ;y_center
	mov al, [@@col]
	add ecx, ebx
	call setPixel, ecx, edx, eax ;(x_center + r, y_center)
	cmp ebx, 0
	jle return;if radius = 0 only one pixel is drawn
	sub ecx, ebx
	add edx, ebx
	call setPixel, ecx, edx, eax ;(x_center, y_center + r)
	sub ecx, ebx
	sub edx, ebx
	call setPixel, ecx, edx, eax ;(x_center - r, y_center)
	add ecx, ebx
	sub edx, ebx
	call setPixel, ecx, edx, eax ;(x_center, y_center - r)
	add edx, ebx
	mov esi, ebx;x = r
	mov edi, 0	;y = 0
	mov ebx, 1
	sub ebx, esi;p = 1 - r
computePosition:
	cmp esi, edi
	jle return	;stop if x <= y
	inc edi 	;y++
	push edi
	add edi, edi
	cmp ebx, 0
	jle branch1	;if (p>0) => x--; p = p + 2y - 2x + 1 
	dec esi
	push esi
	add esi, esi
	add ebx, edi
	sub ebx, esi
	inc ebx
	pop esi
	jmp drawPixels
branch1:	;else => p = p + 2y + 1
	add ebx, edi
	inc ebx 
drawPixels:
	pop edi
	cmp esi, edi
	jl return;stop if x < y
	mov al, [@@col]
	add ecx, esi
	add edx, edi
	call setPixel, ecx, edx, eax ;(x_center + x, y_center + y)
	sub ecx, esi
	sub ecx, esi
	call setPixel, ecx, edx, eax ;(x_center - x, y_center + y)
	sub edx, edi
	sub edx, edi
	call setPixel, ecx, edx, eax ;(x_center - x, y_center - y)
	add ecx, esi
	add ecx, esi
	call setPixel, ecx, edx, eax ;(x_center + x, y_center - y)
	sub ecx, esi
	add ecx, edi
	add edx, edi
	add edx, esi
	call setPixel, ecx, edx, eax ;(x_center + y, y_center + x)
	sub ecx, edi
	sub ecx, edi
	call setPixel, ecx, edx, eax ;(x_center - y, y_center + x)
	sub edx, esi
	sub edx, esi
	call setPixel, ecx, edx, eax ;(x_center - y, y_center - x)
	add ecx, edi
	add ecx, edi
	call setPixel, ecx, edx, eax ;(x_center + y, y_center - x)
	add edx, esi
	sub ecx, edi
	jmp computePosition
return:
	ret
ENDP drawCircle

;draw AABB struc, given a color
PROC drawRectangle
	ARG 	@@rectPtr:dword, @@col:byte
	USES 	eax, ebx, ecx, edx, edi ; note: MUL uses edx!

	; Compute the index of the rectangle's top left corner
	mov ebx, [@@rectPtr]
	mov eax, [(rect ebx).box.min.y]
	mov edx, SCRWIDTH
	mul edx
	add	eax, [(rect ebx).box.min.x]

	; Compute top left corner address
	mov edi, offset screenBuffer
	add edi, eax
	
	; Plot the top horizontal edge.
	mov edx, [(rect ebx).box.max.x]
	sub edx, [(rect ebx).box.min.x] ; store width in edx for later reuse
	mov	ecx, edx
	mov	al, [@@col]
	rep stosb
	sub edi, edx		; reset edi to left-top corner
	
	; plot both vertical edges
	mov ecx, [(rect ebx).box.max.y]
	sub ecx, [(rect ebx).box.min.y]
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

;start game status
;create structs and initialize shapes

PROC startGameStatus
	USES 	eax, ebx, ecx

	square rect <>
	balk rect <>
	paal rect <>
	squareBalkMan manifold <>
	call initRectangle, offset square, 0, 10, 10, 20, 2, 1, 10, 1
	call initRectangle, offset balk, SCRWIDTH - 20, 10, SCRWIDTH, 15, -1, 2, 10, 2
	call initRectangle, offset paal, SCRWIDTH/2, SCRHEIGHT/2, SCRWIDTH/2 + 5, SCRHEIGHT/2 + 20, 1, 2, 3
	
	circ1 circle 	<>
	call initCircle, offset circ1, 50, 50, 10, -1,-1,0,0 
	circ2 circle	<>
	call initCircle, offset circ2, 100, 100, 20, 1,1,0,0
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

	call moveAllObjects
	call allHitBorder
	call applyAllGravity
	inc [timeStamp]
	call checkAllCollisions

	ret
ENDP updateGameStatus

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
	
	timeStamp		dd 0
	vecAB vec 		<>
	screenBuffer 	db 64000 dup(0), '$'
	circLst			dd 64 dup(?), '$'
	circNum			dd 0
	rectLst 		dd 64 dup(?), '$'
	rectNum 		dd 0

STACK 100h

END main