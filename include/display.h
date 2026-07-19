//
// Created by Jasper Cusiel on 20/07/2026.
//

#ifndef ROBOCUP_DISPLAY_H
#define ROBOCUP_DISPLAY_H

void display_init();
void draw();
void display_log(const char* msg);
void display_log_status(const char* name, bool ok);

#endif // ROBOCUP_DISPLAY_H
