/*
File: frame.cpp
Project: DART
Description: Source file for the wxWidgets frame class implementation
*/

#include "frame.hpp"

frame_t::frame_t() : wxFrame(nullptr, wxID_ANY, "DART Ground Station")
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    SetMenuBar(menuBar);

    CreateStatusBar();
    SetStatusText("DART Ground Station Software v1");

    Bind(wxEVT_MENU, &frame_t::OnHello, this, ID_Hello);
    Bind(wxEVT_MENU, &frame_t::OnAbout, this, wxID_ABOUT);
    Bind(wxEVT_MENU, &frame_t::OnExit, this, wxID_EXIT);

    wxBoxSizer *main_sizer = new wxBoxSizer(wxHORIZONTAL); // horizontal box sizer for the primary frame

    wxStaticBox *vehicle_information_static_box = new wxStaticBox(this, wxID_ANY, "Vehicle Information", wxDefaultPosition, wxDefaultSize); // static box to surround vehicle information
    wxStaticBoxSizer *vehicle_information_sizer = new wxStaticBoxSizer(vehicle_information_static_box, wxVERTICAL); // vertical box sizer for the vehicle information list view

    wxListView *vehicle_information_listview = new wxListView(vehicle_information_static_box, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT | wxLC_HRULES); // wxListView to organize vehicle information

    // Add state and value columns to vehicle information wxListView
    for (int i = 0; i < 6; i++){
        if (i % 2 == 0){
            vehicle_information_listview->AppendColumn("State", wxLIST_FORMAT_LEFT);
        } else {
            vehicle_information_listview->AppendColumn("Value", wxLIST_FORMAT_LEFT);
        }
    }

    // 2D vector of states being tracked
    std::vector<std::vector<std::string>> states = {{"State:", "Voltage [V]:", "Temperature [F]:"},
                                                    {"Pyro 1:", "Pyro 2:", "Pyro 3:"},
                                                    {"x-pos [m]:", "x-vel [m]:", "x-acc [m]:"},
                                                    {"y-pos [m/s]:", "y-vel [m/s]:", "y-acc [m/s]:"},
                                                    {"z-pos [m/s^2]:", "z-vel [m/s^2]:", "z-pos [m/s^2]:"},
                                                    {"Roll [deg]:", "Pitch [deg]:", "Yaw [deg]:"}};

    // Populate wxListView with states being tracked
    for (int i = 0; i < vehicle_information_listview->GetColumnCount(); i++){
        if (i % 2 == 0){
            for (int j = 0; j < states.size(); j++){
                wxListItem state;
                state.SetId(j);
                state.SetColumn(i);
                state.SetText(states[j][i/2]);
                vehicle_information_listview->InsertItem(state);
                vehicle_information_listview->SetItem(state);

                // Auto adjust column width to fit widest entry
                vehicle_information_listview->SetColumnWidth(i, wxLIST_AUTOSIZE);
            }
        }
    }

    vehicle_information_sizer->Add(vehicle_information_listview, 1, wxALL | wxALIGN_LEFT); // add vehicle information listview to the vehicle information sizer

    wxBoxSizer *vehicle_communcation_sizer = new wxBoxSizer(wxVERTICAL); // vertical sizer for the communication objects (i.e., CMD/TLM textctrl & Abort button)

    wxStaticBox *cmd_tlm_static_box = new wxStaticBox(this, wxID_ANY, "CMD/TLM Viewer", wxDefaultPosition, wxDefaultSize); // static box to surround the CMD/TLM textctrl
    wxStaticBoxSizer *cmd_tlm_viewer_sizer = new wxStaticBoxSizer(cmd_tlm_static_box, wxVERTICAL); // vertical wxWidgets box sizer for CMD/TLM viewer

    wxTextCtrl *cmd_tlm_text_ctrl = new wxTextCtrl(this, wxID_ANY, "", wxDefaultPosition, FromDIP(wxSize(400, 250)), wxTE_READONLY); // wxTextCtrl to display CMDs/TLMs
    cmd_tlm_viewer_sizer->Add(cmd_tlm_text_ctrl, 1, wxALL | wxEXPAND); // add CMD/TLM textctrl to the CMD/TLM sizer

    wxStaticBox *abort_button_static_box = new wxStaticBox(this, wxID_ANY, "Oh Sh*t!", wxDefaultPosition, wxDefaultSize); // static box to surround the abort button
    wxStaticBoxSizer *abort_button_sizer = new wxStaticBoxSizer(abort_button_static_box, wxVERTICAL); // vertical wxWidgets box sizer for abort button

    wxButton *abort_button = new wxButton(this, wxID_ANY, "ABORT", wxDefaultPosition, wxDefaultSize); // wxWidgets button for the abort button
    abort_button->SetBackgroundColour(*wxRED); // set abort button color to red
    abort_button_sizer->Add(abort_button, 1, wxALL | wxEXPAND); // add abort button to the abort button sizer

    vehicle_communcation_sizer->Add(cmd_tlm_viewer_sizer, 1, wxALL | wxEXPAND); // add the CMD/TLM sizer to the vehicle communication sizer
    vehicle_communcation_sizer->Add(abort_button_sizer, 1, wxALL | wxEXPAND); // add the abort button sizer to the vehicle communication sizer

    main_sizer->Add(vehicle_information_sizer, 0, wxALL | wxEXPAND, 5); // add the vehicle information sizer to the main (top level/primary frame) sizer
    main_sizer->Add(vehicle_communcation_sizer, 1, wxALL | wxEXPAND, 5); // add the vehicle communication sizer to the main (top level/primary frame) sizer

    this->SetSizerAndFit(main_sizer); // set the main sizer as the sizer for the frame and set its default size as the minimum acceptable size
}

void frame_t::OnExit(wxCommandEvent &event)
{
    Close(true);
}

void frame_t::OnAbout(wxCommandEvent &event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}

void frame_t::OnHello(wxCommandEvent &event)
{
    wxLogMessage("Hello world from wxWidgets!");
}