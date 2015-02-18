/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Matthew Klingensmith <mklingen@cs.cmu.edu>

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
#ifndef KINBODYDISPLAY_H_
#define KINBODYDISPLAY_H_

#include <openrave-core.h>

#include <rviz/display.h>
#include "KinBodyVisual.h"

namespace or_rviz
{


    class KinBodyDisplay : public rviz::Display
    {
        public:
            KinBodyDisplay();
            KinBodyDisplay(OpenRAVE::KinBodyPtr kinBody, Ogre::SceneManager* sceneManager);
            virtual ~KinBodyDisplay();

            void CreateProperties(rviz::Property *parent);
            void CreateVisual(OpenRAVE::KinBodyPtr kinBody, Ogre::SceneManager* sceneManager);
            inline void UpdateTransforms() { m_visual->UpdateTransforms(); }

            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();

        protected:
            OpenRAVE::KinBodyPtr m_kinbody;
            rviz::BoolProperty* m_visibleProperty;
            KinBodyVisual* m_visual;

    };

} /* namespace superviewer */
#endif /* KINBODYDISPLAY_H_ */
