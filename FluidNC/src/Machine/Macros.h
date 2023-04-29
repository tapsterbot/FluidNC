// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "src/Configuration/Configurable.h"
#include "src/WebUI/InputBuffer.h"  // WebUI::inputBuffer
#include "src/UartChannel.h"
#include "src/Event.h"
#include <algorithm>

class MacroEvent : public Event {
    int _num;

public:
    MacroEvent(int num) : _num(num) {}
    void run(void*) override;
};

extern MacroEvent macro0Event;
extern MacroEvent macro1Event;
extern MacroEvent macro2Event;
extern MacroEvent macro3Event;

namespace Machine {
    class Macros : public Configuration::Configurable {
    public:
        static const int n_startup_lines = 2;
        static const int n_macros        = 4;

    private:
        std::string _startup_macro;
        std::string _post_home_macro;
        std::string _reset_macro;
        std::string _m30_macro;
        std::string _macro[n_macros];
        std::string _startup_line[n_startup_lines];

    public:
        Macros() = default;

        bool run_macro(size_t index);
        bool run_startup_macro();

        bool execute_macro(std::string macro);

        std::string startup_line(size_t index) {
            if (index >= n_startup_lines) {
                return "";
            }
            auto s = _startup_line[index];
            if (s == "") {
                return s;
            }
            // & is a proxy for newlines in startup lines, because you cannot
            // enter a newline directly in a config file string value.
            std::replace(s.begin(), s.end(), '&', '\n');
            return s + "\n";
        }

        // Configuration helpers:

        // TODO: We could validate the startup lines

        void group(Configuration::HandlerBase& handler) override {
            handler.item("startup_macro", _startup_macro);
            handler.item("post_home", _post_home_macro);
            handler.item("reset", _reset_macro);
            handler.item("m30", _m30_macro);
            handler.item("macro0", _macro[0]);
            handler.item("macro1", _macro[1]);
            handler.item("macro2", _macro[2]);
            handler.item("macro3", _macro[3]);
        }

        ~Macros() {}
    };
}
