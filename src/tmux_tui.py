import npyscreen, time
import threading
import bluetoothctl, pexpect
import libtmux

class connectButton(npyscreen.ButtonPress):
    def __init__(self, *args, **keywords):
        super(connectButton, self).__init__(*args, **keywords)
        self.bl = bluetoothctl.Bluetoothctl()

    def whenPressed(self):
        # Example device ds4berry
        mac_addr = "8C:41:F2:8C:DD:A2"
        npyscreen.notify('Target device: %s\nRemoving it from inventory and acquire it again. '%mac_addr, title='Connection in progress...')
        
        # Removing the target and reconnect
        try:
            self.bl.remove(mac_addr)
        except bluetoothctl.BluetoothctlError as e:
            print(e)
        except pexpect.exceptions.TIMEOUT:
            pass
        self.bl.start_scan()
        time.sleep(1)
        
        npyscreen.notify('Scan and attempt to pair\nPress ps+share on your DualShock4 until it blinks quickly...', title='Connection in progress...')
        paired = False
        attempts = 0
        while not paired and attempts<5:
            try:
                attempts += 1
                paired = self.bl.pair(mac_addr)
            except bluetoothctl.BluetoothctlError as e:
                print(e)
            except pexpect.exceptions.TIMEOUT:
                pass
        if not paired:
            npyscreen.notify('Connection timed out... Please try again', title='Error')
            time.sleep(1)
            return

        
        npyscreen.notify('Negotiating the remaning stuffs, keep the DS4 in alive...', title='Connection in progress...')
        paired = False
        attempts = 0
        while not paired and attempts<5:
            try:
                paired = self.bl.connect(mac_addr)
            except bluetoothctl.BluetoothctlError as e:
                print(e)
            except pexpect.exceptions.TIMEOUT:
                pass
        if not paired:
            npyscreen.notify('Connection timed out... Please try again', title='Error')
            time.sleep(1)
            return

        npyscreen.notify('Done.', title='Connection Finished')
        time.sleep(1)
        return

class tmux_helper(object):
    server = None
    session = None

    def __init__(self, *args, **keywords):
        self.server = libtmux.Server()
    
    def find_session(self):
        self.session = self.server.find_where({ "session_name": "tr_2020" })
    
    def launch_tr(self):
        t = self.server
        s = t.sessions[0]
        w = s.windows[0]
        p = s.attached_pane
        npyscreen.notify(p.get('pane_current_path'))
        p.send_keys('roscore', enter=True)
        

# class ds4Box(npyscreen.BoxTitle):
#     def make_contained_widget(self, contained_widget_arguments=None):
#         self._my_widgets = []
#         _rely = self.rely+2
#         _relx = self.relx+2
#         self._my_widgets.append(connectButton(self.parent,
#             name="Connect Wizard",
#             rely=_rely, relx = _relx,
#             max_width=self.width-4,max_height=1,
#         ))
#         _rely+=2
#         self._my_widgets.append(npyscreen.TitleSelectOne(self.parent,
#             name='Joystick name', scroll_exit=True,
#             rely=_rely, relx = _relx,
#             max_width=self.width-4,max_height=5,
#             values = ['ds4red', 'ds4blue', 'ds4black', 'ds4white',
#                     'ds4orange', 'ds4navy', 'ds4berry', 'default(js0)']
#         ))
#         self.entry_widget = weakref.proxy(self._my_widgets[0])

#     def update(self, clear=True):
#         if self.hidden and clear:
#             self.clear()
#             return False
#         elif self.hidden:
#             return False
#         super(ds4Box, self).update(clear=clear)
#         for w in self._my_widgets:
#             w.update(clear=clear)
#             w.edit()

class MyTmuxApp(npyscreen.NPSAppManaged):
    def onStart(self):
        npyscreen.setTheme(npyscreen.Themes.BlackOnWhiteTheme)
        self.registerForm("MAIN", MainForm(name="Robot/Field Configuration Program v0.9"))

class MainForm(npyscreen.ActionFormV2):

    CANCEL_BUTTON_BR_OFFSET = (2, -5)
    OK_BUTTON_TEXT          = "Launch"
    CANCEL_BUTTON_TEXT      = "Quit"

    def create(self, *args, **keywords):
        super(MainForm, self).create(*args, **keywords)

    def create(self):
        # These lines create the form and populate it with widgets.
        # f = F.add(npyscreen.TitleFixedText, name = "Fixed Text:" , value="This is fixed text")

        _relx, _rely = int(0), int(0)
        _rely += 2
        self.auto_vel = self.add(npyscreen.TitleSlider, name="Auto MAX speed:",
            value=4.5, out_of=5.5, lowest=2.0, step=0.1, 
            max_width=70, rely=_rely,
        )
        _rely += 3
        self.manual_vel = self.add(npyscreen.TitleSlider, name="Manual MAX speed:",
            value=1.2, out_of=5.5, lowest=1.2, step=0.1,
            max_width=70, rely=_rely,
        )
        _rely += 4
        self.btn = self.add(connectButton, name="DualShock4 Connection Wizard",
            max_height=1, relx=0, rely=_rely,
        )
        _rely += 1
        self.ds4_name = self.add(npyscreen.TitleSelectOne,
            scroll_exit=True, name='Joystick name', 
            values = ['ds4red', 'ds4blue', 'ds4black', 'ds4white',
                    'ds4orange', 'ds4navy', 'ds4berry', 'input/js0'],
            max_width=40, max_height=9, rely=_rely,
        )
        _relx += 42
        _rely -= 1
        self.color = self.add(npyscreen.TitleSelectOne, 
            scroll_exit=True, name='Field Color', 
            values = ['red', 'blue'],
            max_width=40, max_height=3, relx=_relx, rely=_rely,
        )
        _rely += 3
        self.team = self.add(npyscreen.TitleSelectOne, 
            scroll_exit=True, name='Team Destinator', 
            values = ['rx', 'rtx'],
            max_width=40, max_height=3, relx=_relx, rely=_rely,
        )

        npyscreen.notify('Program initiating!\nPlease wait...', title='Robocon 2020 - M2 Robotics')
        self.th = tmux_helper()
        time.sleep(1)

    def on_cancel(self):
        confirmed = npyscreen.notify_ok_cancel("Quit will kill all the tmux/ROS session in the background!!!\nAre you sure?", editw=1)
        if confirmed:
            pass # Kill all tmux/ROS stuff and perform a clean shutdown

    def on_ok(self):
        confirmed = npyscreen.notify_ok_cancel("Are you sure about the settings!?", editw=1)
        if confirmed:
            self.th.launch_tr() # Spwan some threads to do tmux stuff

    def exit_application(self):
        # curses.beep()
        self.parentApp.setNextForm(None)
        self.editing = False
        self.parentApp.switchFormNow()

if __name__ == '__main__':
    app = MyTmuxApp()
    app.run()