
// SP_ProfilerDlg.h : файл заголовка
//

#include "SP_ReportDlg.h"
#pragma once


// диалоговое окно CSP_ProfilerDlg
class CSP_ProfilerDlg : public CDialogEx
{
// Создание
public:
	CSP_ProfilerDlg(CWnd* pParent = NULL);	// стандартный конструктор

// Данные диалогового окна
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SP_PROFILER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// поддержка DDX/DDV

	void RefreshDevices();
	void CSP_ProfilerDlg::GetDeviceCapabilities();

	static DWORD WINAPI ReadReport(void*);
	static DWORD WINAPI WriteReport(void*);

// Реализация
protected:
	HICON		m_hIcon;
	BOOL		OnDeviceChange(UINT nEventType, DWORD dwData);

	CComboBox	m_HidTargetsCtrl;
	CListBox    m_TargetsCtrl;
	CString		m_HidTarget;

	CString		m_VIDValue;
	CString		m_PIDValue;
	CString		m_VersionNumber;

	CEdit		m_XPos;
	CEdit		m_YPos;

	CEdit		m_XLowTh;
	CEdit		m_XHighTh;
	CEdit		m_YLowTh;
	CEdit		m_YHighTh;
	CEdit		m_Gear;

	CString		m_ShifterType;
	
	CSP_ReportDlg m_ReportView_frm;

	// Созданные функции схемы сообщений
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnCbnSelchangeHidTargets();	
	afx_msg void OnBnClickedUpdate();
	afx_msg void OnEnChangeXLowTh();
	afx_msg void OnEnChangeXHighTh();
	afx_msg void OnEnChangeYHighTh();
	afx_msg void OnEnChangeYLowTh();
	afx_msg void OnBnClickedRead();

	afx_msg void OnEnSetFocusXLowTh();
//	afx_msg void OnEnKillFocusXLowTh();
	afx_msg void OnEnSetFocusXHighTh();
//	afx_msg void OnEnKillFocusXHighTh();
	afx_msg void OnEnSetFocusYHighTh();
//	afx_msg void OnEnKillFocusYHighTh();
	afx_msg void OnEnSetFocusYLowTh();
//	afx_msg void OnEnKillFocusYLowTh();
};
