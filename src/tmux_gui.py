import npyscreen, time

class TmuxApp(npyscreen.NPSApp):
    def main(self):
        # These lines create the form and populate it with widgets.
        # msg  = F.add(npyscreen.TitleText, name = "Message:",)
        # tdy_date    = F.add(npyscreen.TitleDateCombo, name = "Date:")
        F  = npyscreen.Form(name = "Robot/Field Configuration Program v0.9",)

        auto_vel = F.add(npyscreen.TitleSlider, name="Auto MAX speed:",
            value=4.5, out_of=5.5, lowest=2.0, step=0.1, 
            max_width=70, rely=2)
        manual_vel = F.add(npyscreen.TitleSlider, name="Manual MAX speed:",
            value=1.2, out_of=5.5, lowest=1.2, step=0.1,
            max_width=70, rely=5)
        ds4_name = F.add(npyscreen.TitleSelectOne, rely=9,
            scroll_exit=True, max_height=9, name='Joystick name', 
            values = ['ds4red', 'ds4blue', 'ds4black', 'ds4white',
                    'ds4orange', 'ds4navy', 'ds4berry', 'default(js0)'])
        color = F.add(npyscreen.TitleSelectOne, 
            scroll_exit=True, max_height=3, name='Field Color', 
            values = ['red', 'blue'])
        team = F.add(npyscreen.TitleSelectOne, 
            scroll_exit=True, max_height=3, name='Team Destinator', 
            values = ['rx(TR1)', 'rtx(TR2)'])
        
        # F.add(npyscreen.OptionListDisplay,
        # F.add(npyscreen.TitlePager,
        # F.add(npyscreen.AnnotateTextboxBase,
        # F.add(npyscreen.ButtonPress,
        npyscreen.notify('Program initiating!\nPlease wait...', title='Welcome')
        time.sleep(1)
        F.edit()

        print(ds4_name.get_selected_objects())

if __name__ == '__main__':
   TmuxApp().run()