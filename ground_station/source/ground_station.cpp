/*
File: ground_station.cpp
Project: DART
Description: Ground station GUI
*/

#include "frame.hpp"
 
class ground_app : public wxApp
{
public:
    bool OnInit() override;
};

wxIMPLEMENT_APP(ground_app);

bool ground_app::OnInit()
{
    frame_t *frame = new frame_t();
    frame->Show(true);
    return true;
}