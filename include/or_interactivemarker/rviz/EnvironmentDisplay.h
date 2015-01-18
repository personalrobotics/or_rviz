/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>
         Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#ifndef ENVIRONMENTDISPLAY_H_
#define ENVIRONMENTDISPLAY_H_
#include <QObject>
#include <boost/unordered_map.hpp>
#include <boost/signals2.hpp>
#include <rviz/display.h>
#include <rviz/properties/tf_frame_property.h>
#include <openrave/openrave.h>

namespace or_interactivemarker {
namespace rviz {

class EnvironmentDisplay : public ::rviz::Display
{
    Q_OBJECT

public:
    typedef void FrameChangeCallback(std::string const &frame_id);

    EnvironmentDisplay();
    virtual ~EnvironmentDisplay();

    void set_environment(OpenRAVE::EnvironmentBasePtr const &env);

    boost::signals2::connection RegisterFrameChangeCallback(
        boost::function<FrameChangeCallback> const &callback);

public Q_SLOTS:
    void FrameChangeSlot();

protected:
    virtual void onInitialize();

private:
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::UserDataPtr env_callback_handle_;
    ::rviz::TfFrameProperty *property_frame_;
    ::rviz::Property *property_bodies_;
    boost::unordered_map<OpenRAVE::KinBody *, ::rviz::Property *> body_properties_;
    boost::signals2::signal<FrameChangeCallback> frame_callbacks_;


    void CreateProperties(::rviz::Property *parent);
    void BodyCallback(OpenRAVE::KinBodyPtr body, int flag);
};

}
}

#endif
