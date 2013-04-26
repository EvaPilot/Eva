;------------------------------------------------------------------------------
;-      ATMEL Microcontroller Software Support  -   ROUSSET -
;------------------------------------------------------------------------------
;- File source          : remap.s
;- Librarian            : Not applicable
;- Translator           : 
;-
;- Treatment            : Execute the remap and branch to 0
;-
;- 03/10/01   HI 		: Creation
;------------------------------------------------------------------------------

                AREA    reset, CODE, READONLY

                EXPORT  Jump
Jump
		mov pc, r0				

           	END
