#include "DigiKeyboard.h"

void setup() {
  // this is generally not necessary but with some older systems it seems to
  // prevent missing the first character after a delay:
  DigiKeyboard.sendKeyStroke(0);
  
  // Ctrl-C everything
  for(int i=0; i<5; i++){
    DigiKeyboard.sendKeyStroke(KEY_C, MOD_CONTROL_LEFT);
    DigiKeyboard.delay(500);
  }

  // Prompt the user and wait
  DigiKeyboard.println("");
  DigiKeyboard.print("Launching in 5sec, if abort plz disconnect NOW...");
  for(int i=0; i<5; i++){
    DigiKeyboard.print(".");
    DigiKeyboard.delay(1000);
  }

  // Ctrl-C again to dismiss the prompt
  DigiKeyboard.sendKeyStroke(KEY_C, MOD_CONTROL_LEFT);
  DigiKeyboard.delay(100);
  DigiKeyboard.sendKeyStroke(KEY_C, MOD_CONTROL_LEFT);
  DigiKeyboard.delay(100);

  // Make sure we are on tty1
  DigiKeyboard.sendKeyStroke(KEY_F3, MOD_CONTROL_LEFT | MOD_ALT_LEFT);
  DigiKeyboard.delay(100);

  // Launch our tmux session
  DigiKeyboard.sendKeyStroke(KEY_C, MOD_CONTROL_LEFT);
  DigiKeyboard.delay(100);
  DigiKeyboard.println("tmux new -s tr-2020");
  DigiKeyboard.delay(100);

  // Now switch to tty2
  DigiKeyboard.sendKeyStroke(KEY_F4, MOD_CONTROL_LEFT | MOD_ALT_LEFT);
  DigiKeyboard.delay(100);

  // Launch our tui
  DigiKeyboard.sendKeyStroke(KEY_C, MOD_CONTROL_LEFT);
  DigiKeyboard.delay(100);
  DigiKeyboard.println("python ~/2020_ws/src/m2_tr2020/src/tmux_tui.py");
  DigiKeyboard.delay(100);

}


void loop() {
  // Do nothing in loops
  DigiKeyboard.delay(1000);
}
