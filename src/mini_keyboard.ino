#include <DebounceEvent.h>
#include <Keyboard.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//---  START prototypes   ---
void normal_layout(uint8_t pin);
void tmux_layout(uint8_t pin);
void command_layout(uint8_t pin);
void displayfunc();

void arrow_up();
void arrow_down();
void arrow_left();
void arrow_right();
void spacebar();
void enter();
void ctrl_b();
void ctrl_c();
void ctrl_d();
void m2_command();
void tmux_command();
void ctrl_alt_f1();
void ctrl_alt_f2();
void ctrl_alt_f3();
void python_command();
void page_up();
void page_down();
void launch_ui();
void tmux_z();
void tab();
void esc();
//it is not used, handled in callback(state transition only?)
void nothing(); 
//---   END prototypes   ---


//---   START store   ---
typedef void (*function_ptr)();

struct keyType {
  char symbol[2];
  function_ptr function;   //15,0,4,8,15, 2,1,3,7,5
};

#define key_count 23
//!!!!!! all keys display characters and functions stored here, value assignment in setup()
keyType *key = new keyType[key_count]; 
keyType *mykeys = new keyType[10]; // all the 10 keys of the current mode
//store display layout in this array (2 characters for each key, 10 keys)
char keySymbol[key_count][2] = {
  {24, ' ' }, {25, ' '}, {27, ' '}, {26, ' '}, {95, ' '},
  {'E', 'n'}, {94, 'B'}, {94, 'C'}, {94, 'D'}, {'m','2'},
  {'T', 'm'}, {'F','1'}, {'F','2'}, {'F','3'}, {'P','y'},
  {240, ' '}, {219,' '}, {'P','U'}, {'P','D'}, {'U','I'},
  {'Z', 'm'}, {27 , 26}, {19 ,' '}};
//store key functions in this array
void (*keyFunction[key_count])() = {
  arrow_up, arrow_down, arrow_left, arrow_right, spacebar,
  enter, ctrl_b, ctrl_c, ctrl_d, m2_command,
  tmux_command, ctrl_alt_f1, ctrl_alt_f2, ctrl_alt_f3, python_command,
  nothing, nothing, page_up, page_down, launch_ui,
  tmux_z, tab};
//---   END store   ---


// ------ START buttons library ------
#define NUM_BUTTONS 10

#define BUTTON_R1C1 9
#define BUTTON_R1C2 A2
#define BUTTON_R1C3 A1
#define BUTTON_R1C4 A0
#define BUTTON_R1C5 10

#define BUTTON_R2C1 4
#define BUTTON_R2C2 5
#define BUTTON_R2C3 6
#define BUTTON_R2C4 7
#define BUTTON_R2C5 8

byte buttons[NUM_BUTTONS] = {BUTTON_R1C1, BUTTON_R1C2, BUTTON_R1C3, BUTTON_R1C4, BUTTON_R1C5,
                             BUTTON_R2C1, BUTTON_R2C2, BUTTON_R2C3, BUTTON_R2C4, BUTTON_R2C5
                            };
byte button_convertion[32] = {0};

DebounceEvent* button_events[NUM_BUTTONS];
#define CUSTOM_REPEAT_DELAY     0 // waits for 2nd click, set 0 to disable double clicks
#define CUSTOM_DEBOUNCE_DELAY   50
// ------ END buttons library ------


// ------ START OLED display library ------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ------ END OLED display library ------


// ------ START Keyboard library ------
#include "Keyboard.h"

enum layout {
  NORMAL = 0,
  TMUX,
  COMMAND
};
layout kbl = 0; //kbl is the mode
// ------ END Keyboard library ------

void callback(uint8_t pin, uint8_t event, uint8_t count, uint16_t length) {
  //  Serial.print("Pin : "); Serial.print(button_convertion[pin]);
  //  Serial.print(" Event : "); Serial.print(event);
  //  Serial.print(" Count : "); Serial.print(count);
  //  Serial.print(" Length: "); Serial.print(length);
  //  Serial.println();
  if (button_convertion[pin] == 1) {
    if (event == 2) {
      kbl = (kbl + 1) % 3;
      display.invertDisplay(true); delay(5);
      display.invertDisplay(false);
    } else if (event == 3 && count == 1 && length > 250) {
      kbl = NORMAL; // go back to normal mode on long press
    }
    displayfunc();
  } else if(event == 2){
    // perform key function 
    byte ordered_pin = button_convertion[pin];
    if (ordered_pin>0 && ordered_pin<=10){
      mykeys[ordered_pin-1].function();
    }
  }
}

void setup() {
  // START copy keys symbol and functions into the single array key
  for (int i; i < key_count; ++i) {
    key[i].symbol[0] = keySymbol[i][0];
    key[i].symbol[1] = keySymbol[i][1];
    key[i].function = keyFunction[i];
  }

  // create the hash table for the buttons
  button_convertion[BUTTON_R1C1] = 1;
  button_convertion[BUTTON_R1C2] = 2;
  button_convertion[BUTTON_R1C3] = 3;
  button_convertion[BUTTON_R1C4] = 4;
  button_convertion[BUTTON_R1C5] = 5;
  button_convertion[BUTTON_R2C1] = 6;
  button_convertion[BUTTON_R2C2] = 7;
  button_convertion[BUTTON_R2C3] = 8;
  button_convertion[BUTTON_R2C4] = 9;
  button_convertion[BUTTON_R2C5] = 10;

  // init all pins thru the debounce libaray
  for (int i = 0; i < NUM_BUTTONS; i++) {
    byte temp = buttons[i];
    button_events[i] = new DebounceEvent(temp, callback, BUTTON_PUSHBUTTON, CUSTOM_DEBOUNCE_DELAY, CUSTOM_REPEAT_DELAY);
  }

  // Start emulate itself as a keyboard
  Keyboard.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    //  Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Show Adafruit splash screen.
  display.display(); delay(10);
  display.clearDisplay();

  kbl = 0;
  // Init the menu
  displayfunc();//

  // Ready: flash the screen
  for (byte i = 0; i < 3; i++) {
    display.invertDisplay(true); delay(50);
    display.invertDisplay(false); delay(50);
  }

  //  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < NUM_BUTTONS; i++) {
    button_events[i]->loop();
  }
}


void displayfunc( ) {
  switch (kbl) {
    case NORMAL:
      normal_layout(); break;
    case TMUX:
      tmux_layout(); break;
    case COMMAND:
      command_layout(); break;
    default:
      break;
  }
}

void normal_layout() {
  // Init
  draw_init();
  // Draw top row
  display.setCursor(37, 0);
  display.print(F("M2 UI Mode"));
  //choose the 10 keys:
  mykeys[0] = key[15];
  mykeys[1] = key[0];
  mykeys[2] = key[4];
  mykeys[3] = key[6];
  mykeys[4] = key[22];
  mykeys[5] = key[2];
  mykeys[6] = key[1];
  mykeys[7] = key[3];
  mykeys[8] = key[21];
  mykeys[9] = key[5];
  // Draw bottom row
  draw_bottom();
  display.display();
}

void tmux_layout() {
  draw_init();
  display.setCursor(40, 0);
  display.print(F("TMUX Mode"));
  //choose the 10 keys:
  mykeys[0] = key[15];
  mykeys[1] = key[0];
  mykeys[2] = key[20];
  mykeys[3] = key[17];
  mykeys[4] = key[6];
  mykeys[5] = key[2];
  mykeys[6] = key[1];
  mykeys[7] = key[3];
  mykeys[8] = key[18];
  mykeys[9] = key[7];
  // Draw bottom row
  draw_bottom();
  display.display();
}

void command_layout() {
  draw_init();
  display.setCursor(40, 0);
  display.print(F("COMMAND Mode"));
  display.setTextSize(1);
  display.setCursor(0,23);
  display.print(F("tty>"));
  //choose the 10 keys:
  mykeys[0] = key[15];
  mykeys[1] = key[7]; 
  mykeys[2] = key[8];
  mykeys[3] = key[10];
  mykeys[4] = key[19];
  mykeys[5] = key[11]; 
  mykeys[6] = key[12];
  mykeys[7] = key[13];
  mykeys[8] = key[9];
  mykeys[9] = key[5];
  // Draw bottom row
  draw_bottom();
  display.display();
}

void draw_init() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
}

void draw_bottom( ) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(35, 13);
  display.write(mykeys[0].symbol[0]); display.write(mykeys[0].symbol[1]); 
  if(mykeys[0].symbol[1]!=' '||mykeys[5].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[1].symbol[0]); display.write(mykeys[1].symbol[1]);
  if(mykeys[1].symbol[1]!=' '||mykeys[6].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[2].symbol[0]); display.write(mykeys[2].symbol[1]);
  if(mykeys[2].symbol[1]!=' '||mykeys[7].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[3].symbol[0]); display.write(mykeys[3].symbol[1]);
  if(mykeys[3].symbol[1]!=' '||mykeys[8].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[4].symbol[0]); display.write(mykeys[4].symbol[1]);

  display.setCursor(35, 23);
  display.write(mykeys[5].symbol[0]); display.write(mykeys[5].symbol[1]);
  if(mykeys[0].symbol[1]!=' '||mykeys[5].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[6].symbol[0]); display.write(mykeys[6].symbol[1]);
  if(mykeys[1].symbol[1]!=' '||mykeys[6].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[7].symbol[0]); display.write(mykeys[7].symbol[1]);
  if(mykeys[2].symbol[1]!=' '||mykeys[7].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[8].symbol[0]); display.write(mykeys[8].symbol[1]);
  if(mykeys[3].symbol[1]!=' '||mykeys[8].symbol[1]!=' '){display.write(' ');}
  display.write(mykeys[9].symbol[0]); display.write(mykeys[9].symbol[1]);
  display.display();
}

//---   START key functions   ---
void arrow_up() {
  // do ↑ on press
  Keyboard.press(KEY_UP_ARROW);
  display.invertDisplay(true); delay(10);
  Keyboard.releaseAll(); display.invertDisplay(false);
}
void arrow_down() {
  // do ↓ on press
  Keyboard.press(KEY_DOWN_ARROW); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void arrow_left() {
  // do ← on press
  Keyboard.press(KEY_LEFT_ARROW); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void arrow_right() {
  // do → on press
  Keyboard.press(KEY_RIGHT_ARROW); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void spacebar() {
  // do spacebar on press
  Keyboard.press(32); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void enter() {
  // do enter on press
  Keyboard.press(KEY_RETURN); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void ctrl_b() {
  // do ctrl-B on press
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(98); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void ctrl_c() {
  // do ctrl-C on press
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(99); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void ctrl_d() {
  // do ctrl-D on press
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(100); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}
void m2_command() {
  // type "m2" on press
  display.invertDisplay(true);
  Keyboard.print("m2");
  display.invertDisplay(false);
}
void tmux_command() {
  // type "tmux a" on press (attatch tmux session)
  display.invertDisplay(true);
  Keyboard.println("tmux a");
  display.invertDisplay(false);
}
void ctrl_alt_f1() {
  // do ctrl alt f1 on press
  display.invertDisplay(true);
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_F1); delay(10); Keyboard.releaseAll();
  display.invertDisplay(false);
}
void ctrl_alt_f2() {
  // do ctrl alt f2 on press
  display.invertDisplay(true);
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_F2); delay(10); Keyboard.releaseAll();
  display.invertDisplay(false);
}
void ctrl_alt_f3() {
  // do ctrl alt f3 on press
  display.invertDisplay(true);
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_F3); delay(10); Keyboard.releaseAll();
  display.invertDisplay(false);
}
void python_command() { // repurposed to killall python
  // type "python" on press
  display.invertDisplay(true);
  Keyboard.println("killall python");
  delay(10);
  display.invertDisplay(false);
}
void nothing() {
  display.invertDisplay(true);
  Serial.print("nothing");
  delay(10);Keyboard.releaseAll();
  display.invertDisplay(false);
};
void page_up() {
  // page up on press
  display.invertDisplay(true);
  Keyboard.press(KEY_PAGE_UP);
  delay(10);Keyboard.releaseAll();
  display.invertDisplay(false);
}
void page_down() {
  // page down on press
  display.invertDisplay(true);
  Keyboard.press(KEY_PAGE_DOWN);
  delay(10);Keyboard.releaseAll();
  display.invertDisplay(false);
}
void launch_ui() {
  // switch tty and launch tui

  // goto tty1
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_F1);delay(10);
  Keyboard.releaseAll();delay(100);

  // kill everything, including tmux
  Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(99);
  delay(10); Keyboard.releaseAll(); // ctrl-c
  delay(50);Keyboard.println("tmux a -t robocon_2020");delay(100);
  Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(98);
  delay(10); Keyboard.releaseAll(); // ctrl-b + :kill-session
  Keyboard.println(":kill-session"); delay(100);
  for(int i=0;i<2;i++){
    Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(99); // ctrl-c
    delay(10); Keyboard.releaseAll();
    Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(100); // ctrl-d
    delay(10); Keyboard.releaseAll();
  }

  // launch tmux tr_2020 session
  delay(100);Keyboard.println("tmux new -s robocon_2020");delay(100);

  // goto tty3
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_F3);delay(10);
  Keyboard.releaseAll();delay(100);

  // kill everything and relaunch
  for(int i=0;i<2;i++){
    Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(99); // ctrl-c
    delay(10); Keyboard.releaseAll();
    Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(100); // ctrl-d
    delay(10); Keyboard.releaseAll();
  }
  delay(100);Keyboard.println("killall python");
  delay(100);Keyboard.println("killall roscore");

  // actually launch the stuff
  delay(100);Keyboard.println("python ~/tmux_tui.py");
  kbl=NORMAL;displayfunc();
}

void tmux_z(){
  // ctrl-b + z (toggle zoom) on press
  Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(98);delay(10);
  Keyboard.releaseAll();display.invertDisplay(true);
  Keyboard.press(122);delay(10);
  Keyboard.releaseAll(); display.invertDisplay(false);
}

void tab(){
  // do tab on press
  Keyboard.press(KEY_TAB); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}

void esc(){
  Keyboard.press(KEY_ESC); display.invertDisplay(true);
  delay(10); Keyboard.releaseAll(); display.invertDisplay(false);
}

//---   END key functions   ---
