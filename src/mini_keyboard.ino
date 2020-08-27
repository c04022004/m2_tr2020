/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#include "Keyboard.h"

enum layout {
  NORMAL=0,
  TMUX,
  TMUX_PANE,
  TERM,
  FUNCT,
  PANIC
};
layout kbl = 0;
bool tmux_func = false;

#include <DebounceEvent.h>
#define BUTTON_V0 16
#define BUTTON_V1 14
#define BUTTON_V2 9
#define BUTTON_H0 8
#define BUTTON_H1 7
#define BUTTON_H2 6
#define BUTTON_H3 5
#define BUTTON_H4 4


void key_cb(uint8_t pin, uint8_t event, uint8_t count, uint16_t length) {
  if(pin==BUTTON_V0){ // do mode switch on press
    if(event==2){
      kbl=kbl+1;kbl=kbl>PANIC?NORMAL:kbl;
    }else if(event==3 && count == 1 && length>250){
      kbl=NORMAL;
    }
    tmux_func=false;displayfucn();
  }

  // norm_left
  if(kbl==NORMAL){
    switch(pin){
      case BUTTON_V1:
        if(event==2){ // do 'space' on press
          Keyboard.press(' ');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_V2:
        if(event==2){
          // do TAB on press
          // Keyboard.press(KEY_TAB);display.invertDisplay(true);
          // delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          // do ^C on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('c');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }
  // tumx_left
  if(kbl==TMUX||kbl==TERM||kbl==TMUX_PANE){
    switch(pin){
      case BUTTON_V1:
        if(event==2){ // do ^D on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('d');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_V2:
        if(event==2){ // do ^C on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('c');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }
  // funct left
  if(kbl==FUNCT){
    switch(pin){
      case BUTTON_V1:
        if(event==2){ // do 'space' on press
          Keyboard.press(' ');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_V2:
        if(event==2){ // do ^C on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('c');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }
  // panic_left
  if(kbl==PANIC){
    switch(pin){
      case BUTTON_V1:
        if(event==2){ // do 'esc' on press
          Keyboard.press(KEY_ESC);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_V2:
        if(event==2){ // do ^C^D*10 on press
          for(byte i;i<10;i++){
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('c');display.invertDisplay(true);
            delay(20);Keyboard.releaseAll();display.invertDisplay(false);
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('d');display.invertDisplay(true);
            delay(20);Keyboard.releaseAll();display.invertDisplay(false);
          }
        }
        break;
    }
  }

  // Bottom
  if(kbl==NORMAL){
    switch(pin){
      case BUTTON_H0:
        if(event==2){ // do ↑ on press
          Keyboard.press(KEY_UP_ARROW);;display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;     
      case BUTTON_H1:
        if(event==2){ // do ↓ on press
          Keyboard.press(KEY_DOWN_ARROW);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H2:
        if(event==2){ // do → on press
          Keyboard.press(KEY_LEFT_ARROW);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H3:
        if(event==2){ // do ← on press
          Keyboard.press(KEY_RIGHT_ARROW);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H4:
        if(event==2){ // do 'return' on press
          Keyboard.press(KEY_RETURN);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }else if(kbl==TMUX && !tmux_func){
    switch(pin){
      case BUTTON_H0:
        if(event==2){ // do ^B on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;     
      case BUTTON_H1:
        if(event==2){ // do another layout
          tmux_func=true;displayfucn();
        }
        break;
      case BUTTON_H2:
        if(event==2){ // do esc on press
          Keyboard.press(KEY_ESC);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H3:
        if(event==2){ // do PageUp on press
          Keyboard.press(KEY_PAGE_UP);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H4:
        if(event==2){ // do PageDown on press
          Keyboard.press(KEY_PAGE_DOWN);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      }
    }else if(kbl==TMUX && tmux_func){
      switch(pin){
        case BUTTON_H0:
          if(event==2){ // do ^B+z on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();display.invertDisplay(true);
            Keyboard.press('z');delay(10);Keyboard.releaseAll();display.invertDisplay(false);
            tmux_func=false;displayfucn();
          }
          break;     
        case BUTTON_H1:
          if(event==2){ // do ^B+o on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();display.invertDisplay(true);
            Keyboard.press('o');delay(10);Keyboard.releaseAll();display.invertDisplay(false);
            tmux_func=false;displayfucn();
          }
          break;
        case BUTTON_H2:
          if(event==2){ // do ^B+q on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();display.invertDisplay(true);
            Keyboard.press('q');delay(10);Keyboard.releaseAll();display.invertDisplay(false);
            tmux_func=false;displayfucn();
          }
          break;
        case BUTTON_H3:
          if(event==2){ // do ^B+n on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();display.invertDisplay(true);
            Keyboard.press('n');delay(10);Keyboard.releaseAll();display.invertDisplay(false);
            tmux_func=false;displayfucn();
          }
          break;
        case BUTTON_H4:
          if(event==2){ // do ^B+t on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();display.invertDisplay(true);
            Keyboard.press('t');delay(10);Keyboard.releaseAll();display.invertDisplay(false);
            tmux_func=false;displayfucn();
          }
          break;
      }
  }else if(kbl==TMUX_PANE){
      switch(pin){
        case BUTTON_H0:
          if(event==2){ // do ^B+q + 0 on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();
            Keyboard.press('q');delay(10);Keyboard.releaseAll();
            Keyboard.press('0');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
          break;     
        case BUTTON_H1:
          if(event==2){ // do ^B+q + 1 on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();
            Keyboard.press('q');delay(10);Keyboard.releaseAll();
            Keyboard.press('1');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
          break;
        case BUTTON_H2:
          if(event==2){ // do ^B+q + 2 on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();
            Keyboard.press('q');delay(10);Keyboard.releaseAll();
            Keyboard.press('2');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
          break;
        case BUTTON_H3:
          if(event==2){ // do ^B+q + 3 on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();
            Keyboard.press('q');delay(10);Keyboard.releaseAll();
            Keyboard.press('4');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
          break;
        case BUTTON_H4:
          if(event==2){ // do ^B+q + 4 on press
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('b');
            delay(10);Keyboard.releaseAll();
            Keyboard.press('q');delay(10);Keyboard.releaseAll();
            Keyboard.press('4');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
          break;
      }
  }else if(kbl==TERM){
    switch(pin){
      case BUTTON_H0:
        if(event==2){ // do Ctrl+Alt+F1 on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(KEY_LEFT_ALT);
          Keyboard.press(KEY_F1);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;     
      case BUTTON_H1:
        if(event==2){ // do Ctrl+Alt+F2 on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(KEY_LEFT_ALT);
          Keyboard.press(KEY_F2);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H2:
        if(event==2){ // do Ctrl+Alt+F7 on press
          Keyboard.press(KEY_LEFT_CTRL);Keyboard.press(KEY_LEFT_ALT);
          Keyboard.press(KEY_F7);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H3:
        if(event==2){ // type "m2" on press
          Keyboard.print("m2");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H4:
        if(event==2){ // do 'return' on press
          Keyboard.press(KEY_RETURN);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }else if(kbl==FUNCT){
    switch(pin){
      case BUTTON_H0:
        if(event==2){ // type '4' on press
          Keyboard.print("4");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;     
      case BUTTON_H1:
        if(event==2){ // type '5' on press
          Keyboard.print("5");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H2:
        if(event==2){ // type '6' on press
          Keyboard.print("6");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H3:
        if(event==2){ // type "m2" on press
          Keyboard.press(KEY_BACKSPACE);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H4:
        if(event==2){ // do 'return' on press
          Keyboard.press(KEY_RETURN);display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
    }
  }else if(kbl==PANIC){
    switch(pin){
      case BUTTON_H0:
        if(event==2){ // type 'roscore\n' on press
          Keyboard.println("roscore");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;     
      case BUTTON_H1:
        if(event==2){ // type 'python tr_launch.py\n' on press
          Keyboard.println("python tr_launch.py");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H2:
        if(event==2){ // type 'sudo -i' on press
          Keyboard.println("sudo -i");display.invertDisplay(true);delay(50);
          Keyboard.println("m2m2m2m2");display.invertDisplay(false);
          for(byte i;i<2;i++){
            Keyboard.press(KEY_LEFT_CTRL);Keyboard.press('c');display.invertDisplay(true);
            delay(10);Keyboard.releaseAll();display.invertDisplay(false);
          }
        }
        break;
      case BUTTON_H3:
        if(event==2){ // type 'ip a\n' on press
          Keyboard.println("ip a");display.invertDisplay(true);
          delay(10);Keyboard.releaseAll();display.invertDisplay(false);
        }
        break;
      case BUTTON_H4:
        break;
    }
  }
  if (event==3){
    Keyboard.releaseAll();
  }
}

DebounceEvent bv0 = DebounceEvent(BUTTON_V0, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bv1 = DebounceEvent(BUTTON_V1, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bv2 = DebounceEvent(BUTTON_V2, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);

DebounceEvent bh0 = DebounceEvent(BUTTON_H0, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bh1 = DebounceEvent(BUTTON_H1, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bh2 = DebounceEvent(BUTTON_H2, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bh3 = DebounceEvent(BUTTON_H3, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent bh4 = DebounceEvent(BUTTON_H4, key_cb, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);


void setup() {

  Keyboard.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show Adafruit splash screen.
  display.display();delay(10);
  display.clearDisplay();

  // Init the menu
  displayfucn();
  
  // Ready: flash the screen
  for(byte i=0; i<3; i++){
    display.invertDisplay(true);delay(50);
    display.invertDisplay(false);delay(50);
  }
  
}

void loop() {
  bv0.loop();bv1.loop();bv2.loop();
  bh0.loop();bh1.loop();bh2.loop();bh3.loop();bh4.loop();
}

void displayfucn(void){
  switch(kbl){
    case NORMAL:
      normal_layout();break;
    case TMUX:
      tmux_layout();break;
    case TMUX_PANE:
      tmux_playout();break;
    case TERM:
      term_layout();break;
    case FUNCT:
      func_layout();break;
    case PANIC:
      panic_layout();break;
  }
}

void normal_layout(void){
  // Init
  draw_init();

  // Draw left column
  draw_func_left();

  // Draw top row
  display.setCursor(32, 0);
  display.print(F("Normal:"));

  // Draw bottom row
  draw_bottom();
 
  display.display();
}

void tmux_layout(void){
  draw_init();
  draw_tmux_left();
  display.setCursor(32, 0);
  display.print(F("TMUX:"));
  if(!tmux_func)draw_tmux_bottom();
  else draw_tmux_fbottom();
  display.display();
}

void tmux_playout(void){
  draw_init();
  draw_tmux_left();
  display.setCursor(32, 0);
  display.print(F("TMUX goto pane:"));
  draw_tmux_pbottom();
  display.display();
}

void term_layout(void){
  draw_init();
  draw_tmux_left();
  display.setCursor(32, 0);
  display.print(F("Goto terminal:"));
  draw_term_bottom();
  display.display();
}

void func_layout(void){
  draw_init();
  draw_func_left();
  display.setCursor(32, 0);
  display.print(F("Type sth:"));
  draw_func_bottom();
  display.display();
}

void panic_layout(void){
  draw_init();
  draw_panic_left();
  display.setCursor(32, 0);
  display.print(F("Panic mode!!"));
  draw_panic_bottom();
  display.display();
}

void draw_init(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.cp437(true);
}

// Left Col

void draw_norm_left(void){
  display.setCursor(7, 0);
  display.write(240); // ≡
  display.setCursor(0, 12);
  display.write(192);display.write(196);display.write(217); // spacebar
  display.setCursor(4, 24);
  display.write(27);  // ←
  display.write(26);  // →
  display.drawLine(20, 0, 20, display.height()-1, SSD1306_WHITE);
}

void draw_tmux_left(void){
  display.setCursor(7, 0);
  display.write(240); // ≡
  display.setCursor(3, 12);
  display.write(94);  // ^
  display.write(68);  // d
  display.setCursor(3, 24);
  display.write(94);  // ^
  display.write(67);  // c
  display.drawLine(20, 0, 20, display.height()-1, SSD1306_WHITE);
}

void draw_func_left(void){
  display.setCursor(7, 0);
  display.write(240); // ≡
  display.setCursor(0, 12);
  display.write(192);display.write(196);display.write(217); // spacebar
  display.setCursor(3, 24);
  display.write(94);  // ^
  display.write(67);  // c
  display.drawLine(20, 0, 20, display.height()-1, SSD1306_WHITE);
}

void draw_panic_left(void){
  display.setCursor(7, 0);
  display.write(240); // ≡
  display.setCursor(0, 12);
  display.write(101);display.write(115);display.write(99); // esc
  display.setCursor(7, 24);
  display.write(236);  // ∞
  display.drawLine(20, 0, 20, display.height()-1, SSD1306_WHITE);
}

// Bottom Bar

void draw_bottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(24);  // ↑
  display.write(25);  // ↓
  display.write(27);  // ←
  display.write(26);  // →
  display.write((byte)0);
  display.write(20);  // ¶
}

void draw_tmux_bottom(void){
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(32, 20);
  display.write(94);  // ^
  display.write(66);  // b
  display.write((byte)0);
  display.write(102); // f
  display.write(99);  // c
  display.write(110); // n
  display.write((byte)0);
  display.write(101);
  display.write(115);
  display.write(99); // esc
  display.write((byte)0);
  display.write(80);  // P
  display.write(85);  // U
  display.write((byte)0);
  display.write(80);  // P
  display.write(68);  // D
}

void draw_tmux_fbottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(122);  // z
  display.write(111);  // o
  display.write((byte)0);
  display.write(113);  // q
  display.write(110);  // n
  display.write(116);  // t
}

void draw_tmux_pbottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(48);  // 0
  display.write(49);  // 1
  display.write(50);  // 2
  display.write(51);  // 3
  display.write(52);  // 4
}


void draw_term_bottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(49);  // 1
  display.write(50);  // 2
  display.write(55);  // 7
  display.write((byte)0);
  display.write(42);  // *
  display.write(20);  // ¶
}

void draw_func_bottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(52);  // 4
  display.write(53);  // 5
  display.write(54);  // 6
  display.write((byte)0);
  display.write(17);  // ◄
  display.write(20);  // ¶
}

void draw_panic_bottom(void){
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 16);
  display.write(224);  // α
  display.write(225);  // ß
  display.write(226);  // Γ
  display.write(229);  // σ
  display.write(233);  // Θ
}
