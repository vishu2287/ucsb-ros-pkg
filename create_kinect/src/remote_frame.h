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

#ifndef REMOTE_FRAME_H_
#define REMOTE_FRAME_H_

#include "remote_frame_base.h"
#include "create_kinect/Mode.h"
#include "create_kinect/PointStatus.h"

#include <wx/string.h>
#include <wx/button.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/sizer.h>
#include <wx/frame.h>

#include <wx/event.h>
#include <wx/timer.h>

#include <ros/ros.h>

class CRemoteFrame : public CRemoteFrameBase
{
  public:
    // Constructor
    CRemoteFrame(
        ros::NodeHandlePtr nh,
        wxWindow* parent,
        wxWindowID id,
        const wxString& title,
        const wxPoint& pos,
        const wxSize& size,
        long style);

    ~CRemoteFrame();

    void CentCallback(create_kinect::PointStatus ps);

  protected:
    virtual void PressStop(wxCommandEvent& event);
    virtual void PressAutonomous(wxCommandEvent& event);
    virtual void PressManual(wxCommandEvent& event);

  private:

    void OnUpdate(wxTimerEvent& evt);

    ros::NodeHandlePtr mNh;
    ros::Publisher mModePub;
    ros::Subscriber mCentSub;

    wxTimer* mpUpdateTimer;

};

#endif /* REMOTE_FRAME_H_ */
