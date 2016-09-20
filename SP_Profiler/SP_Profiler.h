
// SP_Profiler.h : главный файл заголовка для приложения PROJECT_NAME
//

#pragma once

#ifndef __AFXWIN_H__
	#error "включить stdafx.h до включения этого файла в PCH"
#endif

#include "resource.h"		// основные символы


// CSP_ProfilerApp:
// О реализации данного класса см. SP_Profiler.cpp
//

class CSP_ProfilerApp : public CWinApp
{
public:
	CSP_ProfilerApp();

// Переопределение
public:
	virtual BOOL InitInstance();

// Реализация

	DECLARE_MESSAGE_MAP()
};

extern CSP_ProfilerApp theApp;