// comm_demo.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// Ccomm_demoApp:
// �йش����ʵ�֣������ comm_demo.cpp
//

class Ccomm_demoApp : public CWinApp
{
public:
	Ccomm_demoApp();

// ��д
	public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern Ccomm_demoApp theApp;