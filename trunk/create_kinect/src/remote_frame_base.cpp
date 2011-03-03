///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "remote_frame_base.h"

///////////////////////////////////////////////////////////////////////////

CRemoteFrameBase::CRemoteFrameBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* mainSizer;
	mainSizer = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* buttonSizer;
	buttonSizer = new wxBoxSizer( wxHORIZONTAL );
	
	mpStop = new wxButton( this, wxID_ANY, wxT("Stop"), wxDefaultPosition, wxDefaultSize, 0 );
	buttonSizer->Add( mpStop, 0, wxALL, 5 );
	
	mpAutonomous = new wxButton( this, wxID_ANY, wxT("Autonomous"), wxDefaultPosition, wxDefaultSize, 0 );
	buttonSizer->Add( mpAutonomous, 0, wxALL, 5 );
	
	mpManual = new wxButton( this, wxID_ANY, wxT("Manual"), wxDefaultPosition, wxDefaultSize, 0 );
	buttonSizer->Add( mpManual, 0, wxALL, 5 );
	
	mpTurnAround = new wxButton( this, wxID_ANY, wxT("Turn Around!"), wxDefaultPosition, wxDefaultSize, 0 );
	buttonSizer->Add( mpTurnAround, 0, wxALL, 5 );
	
	mainSizer->Add( buttonSizer, 0, wxEXPAND, 5 );
	
	wxGridSizer* centroidSizer;
	centroidSizer = new wxGridSizer( 2, 5, 0, 0 );
	
	
	centroidSizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
	XLabel = new wxStaticText( this, wxID_ANY, wxT("X Centroid"), wxDefaultPosition, wxDefaultSize, 0 );
	XLabel->Wrap( -1 );
	XLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( XLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	YLabel = new wxStaticText( this, wxID_ANY, wxT("Y Centroid"), wxDefaultPosition, wxDefaultSize, 0 );
	YLabel->Wrap( -1 );
	YLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( YLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	ZLabel = new wxStaticText( this, wxID_ANY, wxT("Z Centroid"), wxDefaultPosition, wxDefaultSize, 0 );
	ZLabel->Wrap( -1 );
	ZLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( ZLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	Count = new wxStaticText( this, wxID_ANY, wxT("Count"), wxDefaultPosition, wxDefaultSize, 0 );
	Count->Wrap( -1 );
	Count->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( Count, 0, wxALIGN_CENTER|wxALL, 5 );
	
	wxStaticText* LeftCentLabel;
	LeftCentLabel = new wxStaticText( this, wxID_ANY, wxT("Left"), wxDefaultPosition, wxDefaultSize, 0 );
	LeftCentLabel->Wrap( -1 );
	LeftCentLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( LeftCentLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpLeftCentX = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpLeftCentX->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCentX, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpLeftCentY = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpLeftCentY->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCentY, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpLeftCentZ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpLeftCentZ->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCentZ, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpLeftCount = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpLeftCount->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCount, 0, wxALIGN_CENTER|wxALL, 5 );
	
	wxStaticText* RightCentLabel;
	RightCentLabel = new wxStaticText( this, wxID_ANY, wxT("Right"), wxDefaultPosition, wxDefaultSize, 0 );
	RightCentLabel->Wrap( -1 );
	RightCentLabel->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( RightCentLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpRightCentX = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpRightCentX->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCentX, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpRightCentY = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpRightCentY->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCentY, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpRightCentZ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpRightCentZ->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCentZ, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mpRightCount = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 100,-1 ), wxTE_READONLY );
	mpRightCount->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCount, 0, wxALIGN_CENTER|wxALL, 5 );
	
	mainSizer->Add( centroidSizer, 0, 0, 5 );
	
	this->SetSizer( mainSizer );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	mpStop->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressStop ), NULL, this );
	mpAutonomous->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressAutonomous ), NULL, this );
	mpManual->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressManual ), NULL, this );
	mpTurnAround->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressTurnAround ), NULL, this );
}

CRemoteFrameBase::~CRemoteFrameBase()
{
	// Disconnect Events
	mpStop->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressStop ), NULL, this );
	mpAutonomous->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressAutonomous ), NULL, this );
	mpManual->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressManual ), NULL, this );
	mpTurnAround->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressTurnAround ), NULL, this );
	
}
