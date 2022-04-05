/*
 * MIT License
 *
 * Copyright (c) 2022 Teledatics, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

 /**
 * @file teledatics_gui_util.c
 * @author James Ewing
 * @date 31 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"

#define SCB_AIRCR_ADDRESS        ( 0xE000ED0C )
#define SCB_AIRCR                ( ( volatile unsigned long* ) SCB_AIRCR_ADDRESS )
#define SCB_AIRCR_VECTKEY        ( 0x5FA << 16 )
#define SCB_AIRCR_SYSRESETREQ    ( 0x1 << 2 )

/**
 * @brief restart system
 * 
 * Force system restart
 * 
 * @param none
 * @returns none
 */
void restart_system(void)
{
        *SCB_AIRCR = SCB_AIRCR_SYSRESETREQ | SCB_AIRCR_VECTKEY;
}

/**
 * @brief insert new string in place of old string within destination string
 * 
 * String substitution utility
 * 
 * @param destination string
 * @param string to replace
 * @param string to insert
 * @returns pointer to destination string on success, NULL on failure
 */
char *subst_string(char* dest, char *old, const char *new) 
{
        char* p;
        size_t dl,ol,nl,tl;

        if(!dest || !old || !new)
                return NULL;

        p = strstr(dest, old);

        if(!p)
                return NULL;

        dl = strlen(dest) + 1;
        ol = strlen(old);
        nl = strlen(new);
        tl = dl - (p - dest) - ol;

        if(dl == 1 || dl < ol)
                return NULL;

        memmove(p + nl, p + ol, tl);
        memcpy (p, new,  nl);

        return dest;
}
