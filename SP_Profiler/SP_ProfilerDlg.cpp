
// SP_ProfilerDlg.cpp : файл реализации
//

#include "stdafx.h"
#include "INC\hidsdi++.h"
#include "INC\hid.h"
#include "INC\setupapi.h"
#include "INC\usb100.h"

#include <dbt.h>
#include <stdint.h>

#include "SP_Profiler.h"
#include "SP_ProfilerDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

GUID Guid;
HANDLE HidDeviceObject;
HIDP_CAPS Capabilities;
OVERLAPPED	HIDOverlapped;
DWORD NumberOfBytesRead;
DWORD NumberOfBytesWriten;

int Use_Setxxx = 0;

HANDLE ReadThread;
DWORD ReadThreadId;
HANDLE WriteThread;
DWORD WriteThreadId;

extern BYTE InputReport[256];

extern unsigned short xLowTh;
extern unsigned short xHighTh;
extern unsigned short yLowTh;
extern unsigned short yHighTh;

BYTE OutputReport[256];
BOOL WRITE_ROPRT = FALSE, READ_ROPRT = FALSE;

CSP_ProfilerDlg *_this;
CFont gearFont;

// диалоговое окно CSP_ProfilerDlg

CSP_ProfilerDlg::CSP_ProfilerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SP_PROFILER_DIALOG, pParent)
{
	m_HidTarget = _T("");
	m_VIDValue = _T("0xFFFF");
	m_PIDValue = _T("0xFFFF");
	m_VersionNumber = _T("0xFFFF");
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	_this = this;
}

void CSP_ProfilerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_HID_TARGETS, m_HidTargetsCtrl);
	DDX_Control(pDX, IDC_TARGETS_CTRL, m_TargetsCtrl);
	DDX_CBString(pDX, IDC_HID_TARGETS, m_HidTarget);
	DDX_Text(pDX, IDC_VID, m_VIDValue);
	DDX_Text(pDX, IDC_PID, m_PIDValue);
	DDX_Text(pDX, IDC_VERSION, m_VersionNumber);
	DDX_Control(pDX, IDC_X, m_XPos);
	DDX_Control(pDX, IDC_Y, m_YPos);
	DDX_Control(pDX, IDC_X_LOW_TH, m_XLowTh);
	DDX_Control(pDX, IDC_X_HIGH_TH, m_XHighTh);
	DDX_Control(pDX, IDC_Y_LOW_TH, m_YLowTh);
	DDX_Control(pDX, IDC_Y_HIGH_TH, m_YHighTh);
	DDX_Control(pDX, IDC_GEAR, m_Gear);
}

BEGIN_MESSAGE_MAP(CSP_ProfilerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_CBN_SELCHANGE(IDC_HID_TARGETS, &CSP_ProfilerDlg::OnCbnSelchangeHidTargets)
	ON_BN_CLICKED(ID_UPDATE, &CSP_ProfilerDlg::OnBnClickedUpdate)
	ON_BN_CLICKED(IDC_READ, &CSP_ProfilerDlg::OnBnClickedRead)
	ON_EN_CHANGE(IDC_X_LOW_TH, &CSP_ProfilerDlg::OnEnChangeXLowTh)
	ON_EN_CHANGE(IDC_X_HIGH_TH, &CSP_ProfilerDlg::OnEnChangeXHighTh)
	ON_EN_CHANGE(IDC_Y_HIGH_TH, &CSP_ProfilerDlg::OnEnChangeYHighTh)
	ON_EN_CHANGE(IDC_Y_LOW_TH, &CSP_ProfilerDlg::OnEnChangeYLowTh)
	ON_WM_DEVICECHANGE()
END_MESSAGE_MAP()


// обработчики сообщений CSP_ProfilerDlg

BOOL CSP_ProfilerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Задает значок для этого диалогового окна.  Среда делает это автоматически,
	//  если главное окно приложения не является диалоговым
	SetIcon(m_hIcon, TRUE);			// Крупный значок
	SetIcon(m_hIcon, FALSE);		// Мелкий значок

	// TODO: добавьте дополнительную инициализацию
	m_ReportView_frm.Create(IDD_REPORT_VIEW, this);
	ASSERT(IsWindow(m_ReportView_frm.m_hWnd));

	CRect rect;
	CWnd *pWnd = GetDlgItem(IDC_DRAW_FRAME);
	ASSERT(pWnd != NULL);
	ASSERT(IsWindow(pWnd->m_hWnd));
	pWnd->GetWindowRect(&rect);
	ScreenToClient(&rect);

	m_ReportView_frm.SetWindowPos(NULL, rect.left, rect.top, 0, 0, SWP_NOZORDER | SWP_NOSIZE | SWP_NOACTIVATE);
	m_ReportView_frm.EnableWindow(TRUE);
	m_ReportView_frm.ShowWindow(TRUE);

	LOGFONT lf;

	//gets the old font and gets the information and stores it in lf struct
	CFont* old = m_Gear.GetFont();
	old->GetLogFont(&lf);

	//creates the new font using the information derived from the
	//old font and increases the new font height by 30
	//regardless of what number I add to lf.lfheight the font reminds
	//unchanged

	gearFont.CreateFont(lf.lfHeight + 120, 0, lf.lfEscapement, lf.lfOrientation, lf.lfWeight, lf.lfItalic,
		lf.lfUnderline, lf.lfStrikeOut, lf.lfCharSet, lf.lfOutPrecision, lf.lfClipPrecision, lf.lfQuality,
		lf.lfPitchAndFamily, lf.lfFaceName);

	//sets the font to theCEdit control
	m_Gear.SetFont(&gearFont);

	DEV_BROADCAST_DEVICEINTERFACE filter = { 0 };
	filter.dbcc_size = sizeof(filter);
	filter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;

	HidD_GetHidGuid(&Guid);
	filter.dbcc_classguid = Guid;
	RegisterDeviceNotification(m_hWnd, (PVOID)&filter, DEVICE_NOTIFY_WINDOW_HANDLE);

	RefreshDevices();

	SetTimer(1, 1, NULL);

	if (ReadThread == NULL)
	{
		ReadThread = CreateThread(NULL,
			0,
			ReadReport,
			NULL,
			0,
			&ReadThreadId);
	}


	// We check if the thread est ok 
	if (ReadThread == NULL)
	{
		AfxMessageBox(_T("Error occured when creating Read thread (%d)\n"), GetLastError());
	}


	if (WriteThread == NULL)
	{
		WriteThread = CreateThread(NULL,
			0,
			WriteReport,
			NULL,
			0,
			&WriteThreadId);
	}

	/// We check if the thread est ok 
	if (WriteThread == NULL)
	{
		AfxMessageBox(_T("Error occured when creating Write thread (%d)\n"), GetLastError());
	}

	return TRUE;  // возврат значения TRUE, если фокус не передан элементу управления
}

// При добавлении кнопки свертывания в диалоговое окно нужно воспользоваться приведенным ниже кодом,
//  чтобы нарисовать значок.  Для приложений MFC, использующих модель документов или представлений,
//  это автоматически выполняется рабочей областью.

void CSP_ProfilerDlg::OnPaint()
{
	CPaintDC dc(this); // контекст устройства для рисования

	if (IsIconic())
	{
		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Выравнивание значка по центру клиентского прямоугольника
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Нарисуйте значок
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// Система вызывает эту функцию для получения отображения курсора при перемещении
//  свернутого окна.
HCURSOR CSP_ProfilerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CSP_ProfilerDlg::OnBnClickedOk()
{
	// TODO: добавьте свой код обработчика уведомлений
	CDialogEx::OnOK();
}

void CSP_ProfilerDlg::GetDeviceCapabilities()
{
	//Get the Capabilities structure for the device.

	PHIDP_PREPARSED_DATA	PreparsedData;

	/*
	API function: HidD_GetPreparsedData
	Returns: a pointer to a buffer containing the information about the device's capabilities.
	Requires: A handle returned by CreateFile.
	There's no need to access the buffer directly,
	but HidP_GetCaps and other API functions require a pointer to the buffer.
	*/

	HidD_GetPreparsedData(HidDeviceObject, &PreparsedData);
	//DisplayLastError("HidD_GetPreparsedData: ");

	/*
	API function: HidP_GetCaps
	Learn the device's capabilities.
	For standard devices such as joysticks, you can find out the specific
	capabilities of the device.
	For a custom device, the software will probably know what the device is capable of,
	and the call only verifies the information.
	Requires: the pointer to the buffer returned by HidD_GetPreparsedData.
	Returns: a Capabilities structure containing the information.
	*/

	HidP_GetCaps(PreparsedData, &Capabilities);
	HidD_FreePreparsedData(PreparsedData);
}

void CSP_ProfilerDlg::OnCbnSelchangeHidTargets()
{
	// TODO: добавьте свой код обработчика уведомлений
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);

	LPTSTR DevicePath = (LPTSTR)malloc(256);
	m_TargetsCtrl.GetText(m_HidTargetsCtrl.GetCurSel(), DevicePath);

	HidDeviceObject = CreateFile(
		(LPCTSTR)DevicePath,
		GENERIC_READ | GENERIC_WRITE,
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		NULL, //&SecurityAttributes,  //no SECURITY_ATTRIBUTES structure
		OPEN_EXISTING,  //No special create flags
		FILE_FLAG_OVERLAPPED, // No special attributes
		NULL); // No template file
			   
	if (HidDeviceObject != INVALID_HANDLE_VALUE)
	{
		PHIDD_ATTRIBUTES Attributes = (PHIDD_ATTRIBUTES)malloc(sizeof(HIDD_ATTRIBUTES));
		if (HidD_GetAttributes(HidDeviceObject, Attributes))
		{
			m_PIDValue.Format(_T("0x%4X"), Attributes->ProductID);
			m_VIDValue.Format(_T("0x%4X"), Attributes->VendorID);
			m_VersionNumber.Format(_T("0x%4X"), Attributes->VersionNumber);

			if (Attributes->VersionNumber > 0x207)
				Use_Setxxx = 1;

			m_PIDValue.Replace(_T(" "), _T("0"));
			m_VIDValue.Replace(_T(" "), _T("0"));
			m_VersionNumber.Replace(_T(" "), _T("0"));
			
			GetDeviceCapabilities();
			/*
			m_ReportView_frm.UpdateData(TRUE);

			m_ReportView_frm.m_OutPutHxEditLength.Format("%4x", Capabilities.OutputReportByteLength);
			m_ReportView_frm.m_OutPutHxEditLength.Replace((LPCTSTR)" ", (LPCTSTR)"0");

			m_ReportView_frm.m_InPutHxEditLength.Format("%4x", Capabilities.InputReportByteLength);
			m_ReportView_frm.m_InPutHxEditLength.Replace((LPCTSTR)" ", (LPCTSTR)"0");

			WORD  OutSize = _tcstoul(m_ReportView_frm.m_OutPutHxEditLength, 0, 16);
			m_ReportView_frm.m_OutputHexEditorCtrl.SetDataSize(OutSize);

			WORD  InSize = _tcstoul(m_ReportView_frm.m_InPutHxEditLength, 0, 16);
			m_ReportView_frm.m_InputHexEditorCtrl.SetDataSize(InSize);

			m_ReportView_frm.UpdateData(FALSE);

			m_DesrciptorView_frm.UpdateData(FALSE);
			*/
		}
	}

	UpdateData(FALSE);
}

BOOL CSP_ProfilerDlg::OnDeviceChange(UINT nEventType, DWORD dwData)
{
	_DEV_BROADCAST_HEADER *hdr;

	if ((nEventType == DBT_DEVICEREMOVECOMPLETE))// || (nEventType == 0x0007))
	{
		hdr = (_DEV_BROADCAST_HEADER*)dwData;
		if (hdr->dbcd_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
		{
			RefreshDevices();
		}
	}
	if ((nEventType == DBT_DEVICEARRIVAL))
	{
		hdr = (_DEV_BROADCAST_HEADER*)dwData;
		if (hdr->dbcd_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
			RefreshDevices();
	}
	return TRUE;
}


void CSP_ProfilerDlg::RefreshDevices()
{
	char	Product[253];
	CString	Prod, String;

	int Sel = m_HidTargetsCtrl.GetCurSel();
	CString LinkCurSel;

	if (Sel != -1)
		m_TargetsCtrl.GetText(Sel, LinkCurSel);

	m_HidTargetsCtrl.ResetContent();
	m_TargetsCtrl.ResetContent();

	HDEVINFO info;
	info = SetupDiGetClassDevs(&Guid, NULL, NULL, DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);
	if (info != INVALID_HANDLE_VALUE)
	{
		DWORD devIndex;
		SP_INTERFACE_DEVICE_DATA ifData;
		ifData.cbSize = sizeof(ifData);

		for (devIndex = 0; SetupDiEnumDeviceInterfaces(info, NULL, &Guid, devIndex, &ifData); ++devIndex)
		{
			DWORD needed;

			SetupDiGetDeviceInterfaceDetail(info, &ifData, NULL, 0, &needed, NULL);

			PSP_INTERFACE_DEVICE_DETAIL_DATA detail = (PSP_INTERFACE_DEVICE_DETAIL_DATA)new BYTE[needed];
			detail->cbSize = sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);
			SP_DEVINFO_DATA did = { sizeof(SP_DEVINFO_DATA) };

			if (SetupDiGetDeviceInterfaceDetail(info, &ifData, detail, needed, NULL, &did))
			{
				// Add the link to the list of all HID devices
				if (_tcsstr(detail->DevicePath, _T("vid_1209")) != NULL || _tcsstr(detail->DevicePath, _T("vid_0483")) != NULL)
				{
					CString Tmp;
					Tmp = detail->DevicePath;
					Tmp.MakeUpper();
					m_TargetsCtrl.AddString(Tmp);
				}
			}
			else
				m_TargetsCtrl.AddString(_T(""));

			if (_tcsstr(detail->DevicePath, _T("vid_1209")) != NULL || _tcsstr(detail->DevicePath, _T("vid_0483")) != NULL)
			{
				if (SetupDiGetDeviceRegistryProperty(info, &did, SPDRP_DEVICEDESC, NULL, (PBYTE)Product, 253, NULL))
					Prod = (PTCHAR)Product;
				else
					Prod = _T("(Unnamed HID device)");
				// Add the name of the device
				m_HidTargetsCtrl.AddString(Prod);
			}

			delete[](PBYTE)detail;
		}
		SetupDiDestroyDeviceInfoList(info);
	}

	Sel = 0;
	if (LinkCurSel != _T(""))
	{
		Sel = m_TargetsCtrl.FindString(0, LinkCurSel);
		if (Sel == -1)
			Sel = 0;
	}

	m_HidTargetsCtrl.SetCurSel(0);
	OnCbnSelchangeHidTargets();
}

DWORD WINAPI CSP_ProfilerDlg::ReadReport(void*)
{
	int Result;
	CWnd *pWnd;

	DWORD ticks = GetTickCount(), c_ticks;

	while (TRUE)
	{
		if (!WRITE_ROPRT)
		{
			if (HidDeviceObject != INVALID_HANDLE_VALUE)
			{
				CancelIo(HidDeviceObject);
				//InputReport[0]=0;

				unsigned short *pxLowTh = (unsigned short *)&InputReport[3];
				unsigned short *pxHighTh = (unsigned short *)&InputReport[5];
				unsigned short *pyLowTh = (unsigned short *)&InputReport[7];
				unsigned short *pyHighTh = (unsigned short *)&InputReport[9];

				CString str;

				if (READ_ROPRT)
				{
					memset(InputReport, 0, sizeof(InputReport));
					InputReport[0] = 0x02; // Feature report 0x02
					if (HidD_GetFeature(HidDeviceObject, InputReport, sizeof(InputReport)))
					{
						str.Format(_T("%d"), *pxLowTh);
						_this->m_XLowTh.SetWindowText(str);

						str.Format(_T("%d"), *pxHighTh);
						_this->m_XHighTh.SetWindowText(str);

						str.Format(_T("%d"), *pyLowTh);
						_this->m_YLowTh.SetWindowText(str);

						str.Format(_T("%d"), *pyHighTh);
						_this->m_YHighTh.SetWindowText(str);
					}

					READ_ROPRT = FALSE;
				}

				Result = ReadFile
				(HidDeviceObject,
					&InputReport,
					Capabilities.InputReportByteLength,
					&NumberOfBytesRead,
					(LPOVERLAPPED)&HIDOverlapped);

				if (Result) {

					switch (InputReport[0]) {
					case 0x01:
						
						_this->m_ReportView_frm.addPoint(*(unsigned short *)&InputReport[4], *(unsigned short *)&InputReport[6]);

						c_ticks = GetTickCount();
						if (ticks + 40 < c_ticks) // limit form update to ~25 fps
						{
							ticks = c_ticks;

							pWnd = &_this->m_ReportView_frm;

							pWnd->Invalidate();
							pWnd->UpdateWindow();

							if (InputReport[2] & 1) { 
								_this->m_Gear.SetWindowText(_T("1")); 
							}
							else if (InputReport[2] & 2) {
								_this->m_Gear.SetWindowText(_T("2"));
							}
							else if (InputReport[2] & 4) {
								_this->m_Gear.SetWindowText(_T("3"));
							}
							else if (InputReport[2] & 8) {
								_this->m_Gear.SetWindowText(_T("4"));
							}
							else if (InputReport[2] & 16) {
								_this->m_Gear.SetWindowText(_T("5"));
							}
							else if (InputReport[2] & 32) {
								_this->m_Gear.SetWindowText(_T("6"));
							}
							else if (InputReport[2] & 64) {
								_this->m_Gear.SetWindowText(_T("R"));
							}
							else {
								_this->m_Gear.SetWindowText(_T("0"));
							}

							str.Format(_T("%4u"), *(unsigned short *)&InputReport[4]);
							str.Replace(_T(" "), _T("0"));
							_this->m_XPos.SetWindowText(str);

							str.Format(_T("%4u"), *(unsigned short *)&InputReport[6]);
							str.Replace(_T(" "), _T("0"));
							_this->m_YPos.SetWindowText(str);
						}
						break;

					case 0x03:
						str.Format(_T("%d"), *pxLowTh);
						_this->m_XLowTh.SetWindowText(str);

						str.Format(_T("%d"), *pxHighTh);
						_this->m_XHighTh.SetWindowText(str);

						str.Format(_T("%d"), *pyLowTh);
						_this->m_YLowTh.SetWindowText(str);

						str.Format(_T("%d"), *pyHighTh);
						_this->m_YHighTh.SetWindowText(str);
						break;

					default:
						break;
					}
				}
			}
		}
		Sleep(0);
	}
}

DWORD WINAPI CSP_ProfilerDlg::WriteReport(void*)
{
	BOOL Result = TRUE;
	while (TRUE)
	{
		if (WRITE_ROPRT)
		{
			if (HidDeviceObject != INVALID_HANDLE_VALUE)
			{
				CancelIo(HidDeviceObject);

				if (Use_Setxxx == 1)
				{
					Result = HidD_SetFeature
					(HidDeviceObject,
						OutputReport,
						Capabilities.FeatureReportByteLength);
				}
				else
				{
					Result = WriteFile
					(HidDeviceObject,
						&OutputReport,
						Capabilities.OutputReportByteLength,
						&NumberOfBytesWriten,
						(LPOVERLAPPED)&HIDOverlapped);
				}


				WRITE_ROPRT = FALSE;
			}
		}
		Sleep(25);
	}
}


void CSP_ProfilerDlg::OnEnChangeXLowTh()
{
	CString text;
	m_XLowTh.GetWindowText(text);
	xLowTh = _ttoi(text);
	if (xLowTh > 4095) xLowTh = 4095;
}


void CSP_ProfilerDlg::OnEnChangeXHighTh()
{
	CString text;
	m_XHighTh.GetWindowText(text);
	xHighTh = _ttoi(text);
	if (xHighTh > 4095) xHighTh = 4095;
}


void CSP_ProfilerDlg::OnEnChangeYHighTh()
{
	CString text;
	m_YHighTh.GetWindowText(text);
	yHighTh = _ttoi(text);
	if (yHighTh > 4095) yHighTh = 4095;
}


void CSP_ProfilerDlg::OnEnChangeYLowTh()
{
	CString text;
	m_YLowTh.GetWindowText(text);
	yLowTh = _ttoi(text);
	if (yLowTh > 4095) yLowTh = 4095;
}


void CSP_ProfilerDlg::OnBnClickedRead()
{
	// TODO: добавьте свой код обработчика уведомлений

	if (!Use_Setxxx)
	{
		UpdateData(TRUE);

		BYTE pData[] = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		memcpy(&OutputReport, pData, sizeof(pData));

		UpdateData(FALSE);

		WRITE_ROPRT = TRUE;
	}
	else
		READ_ROPRT = TRUE;
}


void CSP_ProfilerDlg::OnBnClickedUpdate()
{
	// TODO: добавьте свой код обработчика уведомлений
	UpdateData(TRUE);

	BYTE pData[] = { 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	if (Use_Setxxx)
	{
		// overwrite second byte
	}

	unsigned short *pxLowTh = (unsigned short *)&pData[3];
	unsigned short *pxHighTh = (unsigned short *)&pData[5];
	unsigned short *pyLowTh = (unsigned short *)&pData[7];
	unsigned short *pyHighTh = (unsigned short *)&pData[9];

	*pxLowTh  = xLowTh;
	*pxHighTh = xHighTh;
	*pyLowTh  = yLowTh;
	*pyHighTh = yHighTh;

	memcpy(&OutputReport, pData, sizeof(pData));

	UpdateData(FALSE);

	WRITE_ROPRT = TRUE;
}

