
#include <wx/app.h>
#include <wx/timer.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "remote_frame.h"

class RemoteMonitorApp : public wxApp
{
  public:
    char** local_argv_;
    ros::NodeHandlePtr nh_;

    RemoteMonitorApp() {}

    bool OnInit()
    {

      // create our own copy of argv, with regular char*s.
      local_argv_ = new char*[argc];
      for (int i = 0; i < argc; ++i)
      {
        local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
      }

      ros::init(argc, local_argv_, "remote_monitor");
      nh_.reset(new ros::NodeHandle);

      //wxInitAllImageHandlers();

      CRemoteFrame* frame = new CRemoteFrame(
        nh_,
        NULL,
        wxID_ANY,
        wxT("Remote Frame"),
        wxDefaultPosition,
        wxSize(700, 350),
        wxDEFAULT_FRAME_STYLE & ~wxRESIZE_BORDER);

      SetTopWindow(frame);
      frame->Show();

      return true;
    }

    int OnExit()
    {
      for (int i = 0; i < argc; ++i)
      {
        free( local_argv_[ i ] );
      }
      delete [] local_argv_;

      return 0;
    }

};

DECLARE_APP(RemoteMonitorApp);
IMPLEMENT_APP(RemoteMonitorApp);
