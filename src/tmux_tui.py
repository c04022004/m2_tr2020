import npyscreen, time
import threading
#import bluetoothctl
import pexpect
import libtmux
import atexit


class connectButton(npyscreen.ButtonPress):
    def __init__(self, *args, **keywords):
        super(connectButton, self).__init__(*args, **keywords)
        self.ds4_name=keywords['ds4_name']
        #self.bl = bluetoothctl.Bluetoothctl()

    def whenPressed(self):
        # Example device ds4berry
        mac_addr = "8C:41:F2:8C:DD:A2"
        # npyscreen.notify('Target device: %s\nRemoving it from inventory and acquire it again. '%mac_addr, title='Connection in progress...')
        npyscreen.notify('Target device: %s\nRemoving it from inventory and acquire it again. '%self.ds4_name.get_selected_objects()[0], title='Connection in progress...')
        time.sleep(1)
        return
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
        return self.session
    
    def launch_tr(self, team, color, ds4_name):
        # t = self.server
        s = self.find_session()
        w = s.windows[0]
        p = s.attached_pane
        npyscreen.notify(p.get('pane_current_path'))
        w.split_window(vertical=True, attach=False, target="0")
        w.split_window(vertical=False, attach=False, target="0")
        w.split_window(vertical=False, attach=False, target="2")
        w.split_window(vertical=True, attach=False, target="2")
        panes = w.list_panes()
        panes[2].send_keys('killall -9 rosmaster', enter=True, suppress_history=False)
        panes[2].send_keys('roscore', enter=True, suppress_history=False)
        time.sleep(5)
        panes[0].send_keys('roslaunch m2_tr2020 localization_control.launch', suppress_history=False)
        panes[1].send_keys('roslaunch m2_tr2020 semi_auto.launch team:=%s color:=%s joy:=%s'%(team, color, ds4_name), suppress_history=False)
        panes[3].send_keys('rosbag record -a', enter=True, suppress_history=False)
        panes[4].send_keys('roslaunch m2_tr2020 base_hw_pneumatic.launch', suppress_history=False)
        # panes[0].send_keys(l[0])

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
        self.registerForm("FORM2", Form2(name="Page 2",lines=25, columns=100, draw_line_at=16))
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
        _rely += 3
        self.ds4_name = self.add(npyscreen.TitleSelectOne,
            scroll_exit=True, name='Joystick name', 
            values = ['ds4red', 'ds4blue', 'ds4black', 'ds4white',
                    'ds4orange', 'ds4navy', 'ds4berry', 'input/js0'],
            max_width=40, max_height=9, rely=_rely+1,
        )
        # _rely += 1
        self.btn = self.add(connectButton, name="DualShock4 Connection Wizard",
            max_height=1, relx=0, rely=_rely, ds4_name=self.ds4_name,
        )
        _relx += 42
        _rely += 1
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
        if (len(self.team.get_selected_objects()) == 0 or 
            len(self.color.get_selected_objects()) == 0 or 
            len(self.ds4_name.get_selected_objects()) == 0):
            npyscreen.notify_confirm("Invalid input: You didnt fill in all the fields!")
        else:
            confirmed = npyscreen.notify_ok_cancel("Are you sure about the settings!?", editw=1)
            if confirmed:
                self.th.launch_tr(team=self.team.get_selected_objects()[0],
                                color=self.color.get_selected_objects()[0],
                                ds4_name=self.ds4_name.get_selected_objects()[0]) # Spwan some threads to do tmux stuff
                self.parentApp.setNextForm("FORM2")

    def exit_application(self):
        # curses.beep()
        self.parentApp.setNextForm(None)
        self.editing = False
        self.parentApp.switchFormNow()

def exit_handler():
    server = libtmux.Server()
    session = server.find_where({ "session_name": "tr_2020" })
    window = session.attached_window
    panes=window.list_panes()
    for i in range(len(panes)-1):
        i=len(panes)-i-1
        panes[i].send_keys('^C', enter=False, suppress_history=False)
        panes[i].send_keys("exit")

class Form2(npyscreen.ActionFormV2):
    CANCEL_BUTTON_TEXT   = "Back"
    width=10
    height=10
    max_width=10
    max_height=10
    def create(self):
        self.option = self.add(npyscreen.TitleSelectOne,
        scroll_exit=True, name='Option', 
        values = ['Option1','Option2'],
        max_width=40, max_height=9)

        
    def exit(self):
        self.parentApp.switchForm(None) 

    def on_ok(self):
        npyscreen.notify_confirm("You selected "+str(self.option.get_selected_objects()), title="Notice", wrap=True, wide=True, editw=1)
        self.parentApp.setNextForm("FORM2")

    def on_cancel(self):
        self.parentApp.setNextForm("MAIN") 
        exit_handler()

if __name__ == '__main__':
    atexit.register(exit_handler)
    app = MyTmuxApp()
    app.run()
