/*
File: frame.hpp
Project: DART
Description: Header file for the wxWidgets frame class implementation
*/

#pragma once

#include <wx/wx.h>
#include <wx/event.h>
#include <wx/listbase.h>
#include <wx/listctrl.h>

#include "IDs.hpp"

class frame_t : public wxFrame
{
public:
    frame_t();

private:
    void OnHello(wxCommandEvent &event);
    void OnExit(wxCommandEvent &event);
    void OnAbout(wxCommandEvent &event);
};