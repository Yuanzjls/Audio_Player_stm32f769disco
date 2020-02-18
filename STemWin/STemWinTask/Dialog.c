/*
 * Dialog.c
 *
 *  Created on: Feb 9, 2020
 *      Author: Yuan
 */

#include "DIALOG.h"
/*
*********************************************************************************************************
* 宏定义
*********************************************************************************************************
*/
#define ID_FRAMEWIN_0 (GUI_ID_USER + 0x00)
#define ID_BUTTON_0 (GUI_ID_USER + 0x01)
#define ID_SCROLLBAR_0 (GUI_ID_USER + 0x02)
#define ID_SLIDER_0 (GUI_ID_USER + 0x03)

/*
*********************************************************************************************************
* GUI_WIDGET_CREATE_INFO 类型数组
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0, 480, 272, 0, 0x64, 0 },
{ BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 130, 28, 147, 35, 0, 0x0, 0 },
{ SCROLLBAR_CreateIndirect, "Scrollbar", ID_SCROLLBAR_0, 129, 74, 147, 28, 0, 0x0, 0 },
{ SLIDER_CreateIndirect, "Slider", ID_SLIDER_0, 133, 118, 137, 25, 0, 0x0, 0 },
};
/*
*********************************************************************************************************
* 函 数 名: _cbDialog
* 功能说明: 对话框回调函数
* 形 参: pMsg 回调参数
* 返 回 值: 无
*********************************************************************************************************
*/
static void _cbDialog(WM_MESSAGE * pMsg)
{
WM_HWIN hItem;
int NCode;
int Id;
switch (pMsg->MsgId) //--------------（1）
{
case WM_INIT_DIALOG: //--------------（2）
//
// 初始化 'Framewin'
//
hItem = pMsg->hWin;
FRAMEWIN_SetFont(hItem, GUI_FONT_32B_ASCII);
FRAMEWIN_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
FRAMEWIN_SetText(hItem, "armfly");
//
// 初始化 'Button'
//
hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
BUTTON_SetFont(hItem, GUI_FONT_24B_ASCII);
BUTTON_SetText(hItem, "armfly");
break;
case WM_PAINT: //--------------（3）
GUI_SetBkColor(GUI_BLUE);
GUI_Clear();
break;
case WM_KEY: //--------------（4）
switch (((WM_KEY_INFO*)(pMsg->Data.p))->Key)
{
case GUI_KEY_ESCAPE:
	GUI_EndDialog(pMsg->hWin, 1);
	break;
	case GUI_KEY_ENTER:
	GUI_EndDialog(pMsg->hWin, 0);
	break;
	}
	break;
	case WM_NOTIFY_PARENT: //--------------（ 5）
	Id = WM_GetId(pMsg->hWinSrc);
	NCode = pMsg->Data.v;
	switch(Id)
	{
	case ID_BUTTON_0:
	switch(NCode)
	{
	case WM_NOTIFICATION_CLICKED:
	break;
	case WM_NOTIFICATION_RELEASED:
	break;
	}
	break;
	case ID_SCROLLBAR_0:
	switch(NCode)
	{
	case WM_NOTIFICATION_CLICKED:
	break;
	case WM_NOTIFICATION_RELEASED:
	break;
	case WM_NOTIFICATION_VALUE_CHANGED:
	break;
	}
	break;
	case ID_SLIDER_0:
	switch(NCode)
	{
	case WM_NOTIFICATION_CLICKED:
	break;
	case WM_NOTIFICATION_RELEASED:
	break;
	case WM_NOTIFICATION_VALUE_CHANGED:
	break;
	}
	break;
	}
	break;
	default:
	WM_DefaultProc(pMsg);
	break;
}
}
/*
*********************************************************************************************************
* 函 数 名: CreateFramewin
* 功能说明: 创建对话框
* 形 参: 无
* 返 回 值: 返回对话框句柄
*********************************************************************************************************
*/
WM_HWIN CreateFramewin(void)
{
WM_HWIN hWin;
hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
return hWin;
}
/*
*********************************************************************************************************
* 函 数 名: MainTask
* 功能说明: GUI 主函数
* 形 参: 无
* 返 回 值: 无
*********************************************************************************************************
*/
void MainTask(void)
{
	WM_HWIN hWin;
/* 初始 emWin */
	GUI_Init();

	/* 创建对话框 */
	hWin = CreateFramewin();
	WM_PaintWindowAndDescs(hWin);
	while(1)
	{
		GUI_Delay(10);
	}
}
