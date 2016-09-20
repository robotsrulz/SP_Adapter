// SP_ReportDlg.cpp: файл реализации
//

#include <stdint.h>
#include <utility>

#include "stdafx.h"
#include "SP_Profiler.h"
#include "SP_ReportDlg.h"
#include "afxdialogex.h"

#include "iterable_queue.h"

BYTE InputReport[256];

unsigned short xLowTh = 0;
unsigned short xHighTh = 0;
unsigned short yLowTh = 0;
unsigned short yHighTh = 0;

#define TRAIL_MAX_SIZE	200
static iterable_queue<std::pair<int, int>> trail;

static CPen penBlue(PS_SOLID, 1, RGB(0, 0, 255));
static CPen penRed(PS_SOLID, 1, RGB(200, 0, 0));

// диалоговое окно CSP_ReportDlg

IMPLEMENT_DYNAMIC(CSP_ReportDlg, CDialogEx)

CSP_ReportDlg::CSP_ReportDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_REPORT_VIEW, pParent)
{

}

CSP_ReportDlg::~CSP_ReportDlg()
{
}

void CSP_ReportDlg::addPoint(int x, int y)
{
	std::pair<int, int> p(x, y);
	trail.push(p);

	if (trail.size() > TRAIL_MAX_SIZE)
		trail.pop();
}

void CSP_ReportDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CSP_ReportDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
END_MESSAGE_MAP()


// обработчики сообщений CSP_ReportDlg

void CSP_ReportDlg::OnPaint() {

	CPaintDC dc(this);
	CRect rect;

	GetWindowRect(&rect);
	ScreenToClient(&rect);

	int x0 = rect.TopLeft().x, y0 = rect.TopLeft().y + 10, x1 = rect.BottomRight().x, y1 = rect.BottomRight().y;

	if (InputReport[0] == 0x01)
	{
		unsigned short x = *(unsigned short *)&InputReport[4];
		unsigned short y = *(unsigned short *)&InputReport[6];

		int x2 = x0 + (x1 - x0) * x / 4096;
		int y2 = y0 + 10 + (y1 - y0) * y / 4096;

		int c = 0;
		int nsave = dc.SaveDC();

		for (auto it = trail.begin(); it != trail.end(); ++it)
		{
			c += 1;
			std::pair<int, int> p = *it;

			SetPixel(dc, x0 + (x1 - x0) * p.first / 4096, y0 + 10 + (y1 - y0) * p.second / 4096, RGB(c, c, c));
		}

		dc.SelectObject(&penRed);

		dc.MoveTo(x2, y2 - 4);
		dc.LineTo(x2, y2 + 5);

		dc.MoveTo(x2 - 4, y2);
		dc.LineTo(x2 + 5, y2);

		dc.SelectObject(&penBlue);

		if (yLowTh) {
			int y_low = y0 + 10 + (y1 - y0) * yLowTh / 4096;

			dc.MoveTo(x0 + 5, y_low);
			dc.LineTo(x1 - 5, y_low);
		}

		if (yHighTh) {
			int y_high = y0 + 10 + (y1 - y0) * yHighTh / 4096;

			dc.MoveTo(x0 + 5, y_high);
			dc.LineTo(x1 - 5, y_high);
		}

		if (xLowTh) {
			int x_low = x0 + (x1 - x0) * xLowTh / 4096;

			dc.MoveTo(x_low, y0);
			dc.LineTo(x_low, y1 - 10);
		}

		if (xHighTh) {
			int x_high = x0 + (x1 - x0) * xHighTh / 4096;

			dc.MoveTo(x_high, y0);
			dc.LineTo(x_high, y1 - 10);
		}

		dc.RestoreDC(nsave);
	}
}