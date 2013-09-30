/*
 * KinBodyDisplay.h
 *
 *  Created on: Sep 24, 2013
 *      Author: mklingen
 */

#ifndef KINBODYDISPLAY_H_
#define KINBODYDISPLAY_H_

#include <rviz/display.h>

namespace superviewer
{

    class KinBodyDisplay : rviz::Display
    {
        public:
            KinBodyDisplay();
            virtual ~KinBodyDisplay();

            virtual void onInitialize();
            virtual void fixedFrameChanged();
            virtual void reset();
            virtual void createProperties();

    };

} /* namespace superviewer */
#endif /* KINBODYDISPLAY_H_ */
