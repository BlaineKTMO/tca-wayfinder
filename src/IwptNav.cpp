/**
 * File: IwptNav.cpp
 * Author: Blaine Oania
 * Date: 2/11/2023
 * Description:
 *  Abstract interface class for sending gwpt and
 *  receiving iwpt.
*/

#include <tca_turtlebot/IwptService.h>

class IwptNav {
    private:
    virtual void iwpt_callback(
        tca_turtlebot::IwptServiceRequest req, 
        tca_turtlebot::IwptServiceResponse res) = 0;
};