//=================================================================================================
// Copyright (c) 2011, Paul Filitchkin
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the UCSB Robotics Lab nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "remote_frame.h"
#include "remote_frame_base.h"
#include "remote_modes.h"
#include <wx/button.h>

//=================================================================================================
//=================================================================================================
CRemoteFrame::CRemoteFrame(
    ros::NodeHandlePtr nh,
    wxWindow* parent,
    wxWindowID id,
    const wxString& title,
    const wxPoint& pos,
    const wxSize& size,
    long style ) : CRemoteFrameBase( parent, id, title, pos, size, style )
{

  srand(time(NULL));

  mpUpdateTimer = new wxTimer(this);
  mpUpdateTimer->Start(16);

  Connect(mpUpdateTimer->GetId(), wxEVT_TIMER,
      wxTimerEventHandler(CRemoteFrame::OnUpdate), NULL, this);

  mNh = nh;
  mModePub = nh->advertise<create_kinect::Mode>("remote_monitor/mode", 1);
  mCentSub = nh->subscribe<create_kinect::PointStatus>(
      "/remote_monitor/pointStatus", 1, &CRemoteFrame::CentCallback, this);
}

//=================================================================================================
//=================================================================================================
CRemoteFrame::~CRemoteFrame()
{
  delete mpUpdateTimer;
}
//=================================================================================================
//=================================================================================================
void CRemoteFrame::CentCallback(create_kinect::PointStatus p)
{

  mpLeftCentX->SetValue(wxString::Format(_T("%0.3f"), p.centlx));
  mpLeftCentY->SetValue(wxString::Format(_T("%0.3f"), p.cently));
  mpLeftCentZ->SetValue(wxString::Format(_T("%0.3f"), p.centlz));

  mpRightCentX->SetValue(wxString::Format(_T("%0.3f"), p.centrx));
  mpRightCentY->SetValue(wxString::Format(_T("%0.3f"), p.centry));
  mpRightCentZ->SetValue(wxString::Format(_T("%0.3f"), p.centrz));

  mpRightCount->SetValue(wxString::Format(_T("%d"), p.pointCntr));
  mpLeftCount->SetValue(wxString::Format(_T("%d"), p.pointCntl));

}

//=================================================================================================
//=================================================================================================
void CRemoteFrame::PressStop(wxCommandEvent& event)
{
  create_kinect::Mode m;
  m.mode = create_kinect::stop;
  mModePub.publish(m);
}

//=================================================================================================
//=================================================================================================
void CRemoteFrame::PressAutonomous(wxCommandEvent& event)
{
  create_kinect::Mode m;
  m.mode = create_kinect::autonomous;
  mModePub.publish(m);
}

//=================================================================================================
//=================================================================================================
void CRemoteFrame::PressManual(wxCommandEvent& event)
{
  create_kinect::Mode m;
  m.mode = create_kinect::manual;
  mModePub.publish(m);
}

//=================================================================================================
//=================================================================================================
void CRemoteFrame::PressTurnAround(wxCommandEvent& event)
{
  create_kinect::Mode m;
  m.mode = create_kinect::turnAround;
  mModePub.publish(m);
}
//=================================================================================================
//=================================================================================================
void CRemoteFrame::OnUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();

  if (!ros::ok())
  {
    Close();
  }
}
