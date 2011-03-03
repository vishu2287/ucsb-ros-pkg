///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __remote_frame_base__
#define __remote_frame_base__

#include <wx/string.h>
#include <wx/button.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class CRemoteFrameBase
///////////////////////////////////////////////////////////////////////////////
class CRemoteFrameBase : public wxFrame 
{
	private:
	
	protected:
		wxButton* mpStop;
		wxButton* mpAutonomous;
		wxButton* mpManual;
		wxButton* mpTurnAround;
		
		wxStaticText* XLabel;
		wxStaticText* YLabel;
		wxStaticText* ZLabel;
		wxStaticText* Count;
		wxTextCtrl* mpLeftCentX;
		wxTextCtrl* mpLeftCentY;
		wxTextCtrl* mpLeftCentZ;
		wxTextCtrl* mpLeftCount;
		wxTextCtrl* mpRightCentX;
		wxTextCtrl* mpRightCentY;
		wxTextCtrl* mpRightCentZ;
		wxTextCtrl* mpRightCount;
		
		// Virtual event handlers, overide them in your derived class
		virtual void PressStop( wxCommandEvent& event ) { event.Skip(); }
		virtual void PressAutonomous( wxCommandEvent& event ) { event.Skip(); }
		virtual void PressManual( wxCommandEvent& event ) { event.Skip(); }
		virtual void PressTurnAround( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		CRemoteFrameBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Remote Monitor"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 587,225 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~CRemoteFrameBase();
	
};

#endif //__remote_frame_base__
