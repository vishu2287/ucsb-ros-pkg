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
	
	mainSizer->Add( buttonSizer, 0, wxEXPAND, 5 );
	
	wxGridSizer* centroidSizer;
	centroidSizer = new wxGridSizer( 4, 2, 0, 0 );
	
	wxStaticText* leftCentX;
	leftCentX = new wxStaticText( this, wxID_ANY, wxT("Left Centroid X"), wxDefaultPosition, wxDefaultSize, 0 );
	leftCentX->Wrap( -1 );
	leftCentX->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( leftCentX, 0, wxALL, 5 );
	
	mpLeftCentX = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_READONLY );
	mpLeftCentX->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCentX, 0, wxALL, 5 );
	
	wxStaticText* leftCentZ;
	leftCentZ = new wxStaticText( this, wxID_ANY, wxT("Left Centroid Z"), wxDefaultPosition, wxDefaultSize, 0 );
	leftCentZ->Wrap( -1 );
	leftCentZ->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( leftCentZ, 0, wxALL, 5 );
	
	mpLeftCentZ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_READONLY );
	mpLeftCentZ->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpLeftCentZ, 0, wxALL, 5 );
	
	wxStaticText* rightCentX;
	rightCentX = new wxStaticText( this, wxID_ANY, wxT("Right Centroid X"), wxDefaultPosition, wxDefaultSize, 0 );
	rightCentX->Wrap( -1 );
	rightCentX->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( rightCentX, 0, wxALL, 5 );
	
	mpRightCentX = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_READONLY );
	mpRightCentX->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCentX, 0, wxALL, 5 );
	
	wxStaticText* rightCentZ;
	rightCentZ = new wxStaticText( this, wxID_ANY, wxT("Right Centroid Z"), wxDefaultPosition, wxDefaultSize, 0 );
	rightCentZ->Wrap( -1 );
	rightCentZ->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( rightCentZ, 0, wxALL, 5 );
	
	mpRightCentZ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 200,-1 ), wxTE_READONLY );
	mpRightCentZ->SetFont( wxFont( 20, 70, 90, 90, false, wxEmptyString ) );
	
	centroidSizer->Add( mpRightCentZ, 0, wxALL, 5 );
	
	mainSizer->Add( centroidSizer, 1, wxEXPAND, 5 );
	
	this->SetSizer( mainSizer );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	mpStop->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressStop ), NULL, this );
	mpAutonomous->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressAutonomous ), NULL, this );
	mpManual->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressManual ), NULL, this );
}

CRemoteFrameBase::~CRemoteFrameBase()
{
	// Disconnect Events
	mpStop->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressStop ), NULL, this );
	mpAutonomous->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressAutonomous ), NULL, this );
	mpManual->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CRemoteFrameBase::PressManual ), NULL, this );
	
}
