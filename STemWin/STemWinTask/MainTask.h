/*
*********************************************************************************************************
*	                                  

*********************************************************************************************************
*/

#ifndef __MainTask_H
#define __MainTask_H

#include "GUI.h"
#include "DIALOG.h"
#include "WM.h"
#include "BUTTON.h"
#include "CHECKBOX.h"
#include "DROPDOWN.h"
#include "EDIT.h"
#include "FRAMEWIN.h"
#include "LISTBOX.h"
#include "MULTIEDIT.h"
#include "RADIO.h"
#include "SLIDER.h"
#include "TEXT.h"
#include "PROGBAR.h"
#include "SCROLLBAR.h"
#include "LISTVIEW.h"
#include "GRAPH.h"
#include "MENU.h"
#include "MULTIPAGE.h"
#include "ICONVIEW.h"
#include "TREEVIEW.h"

/*
************************************************************************
*
************************************************************************
*/

#define WM_USER_UPDATEFILENAME (WM_USER + 0)

typedef struct
{
	uint8_t second;
	uint8_t minute;
	uint8_t current_second;
	uint8_t current_minute;
	uint32_t current_progress_insecond;
	uint32_t total_second;
}AudioTime;
void YaHeiFont_Init(void);
#endif

/*****************************(END OF FILE) *********************************/
