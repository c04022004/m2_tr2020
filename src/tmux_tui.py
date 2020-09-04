#!/usr/bin/env python

import npyscreen, time
import re
import os
import threading
import subprocess
import bluetoothctl
import glob
import pexpect
import libtmux
import atexit

from ds4_names import DS4_NAME_TO_MAC, DS4_NAMES

TMUX_SESSION_NAME = "robocon_2020"

def npyscr_notify_terminal(cmd, args, timeout=10, backlog_size=10):
    backlog = []
    with pexpect.spawn(cmd, args=args) as proc:
        start = time.time()
        last_line = b""
        while time.time() < start + timeout:
            try:
                byte = proc.read_nonblocking(timeout=1)
                if byte == b"\n":
                    backlog.append(last_line.decode('utf-8'))
                    while len(backlog) > backlog_size:
                        backlog.pop(0)
                    last_line = b""
                else:
                    last_line += byte
            except pexpect.TIMEOUT:
                pass
            remaining_time = round(start + timeout - time.time())
            npyscreen.notify("\n".join(backlog), title=f"{cmd} {args} [{remaining_time}s]", wide=True)
        proc.terminate(force=True)

class connectButton(npyscreen.ButtonPress):
    def __init__(self, *args, **keywords):
        super(connectButton, self).__init__(*args, **keywords)
        self.ds4_name=keywords['ds4_name']
        self.logger_queue = []
        self.bl = bluetoothctl.Bluetoothctl(self.push_log)

    def set_status(self, title, status):
        self.logger_title = title
        self.logger_status = status
        self.flush_npyscreen_notify()

    def push_log(self, message):
        for line in message.split("\n"):
            self.logger_queue.append(message)
        LOG_SIZE = 5
        while len(self.logger_queue) > LOG_SIZE:
            self.logger_queue.pop(0)
        self.flush_npyscreen_notify()

    def flush_npyscreen_notify(self):
        msg = "{}\n===\n".format(self.logger_status)
        for log in self.logger_queue:
            msg += "\n"
            msg += log
        npyscreen.notify(msg, title=self.logger_title, wide=True)

    def whenPressed(self):
        # Example device ds4berry
        ds4_name = next(iter(self.ds4_name.get_selected_objects()), None)
        if ds4_name is None:
            self.set_status('Error', 'Choose a ds4 device first')
            time.sleep(0.5)
            return
        if ds4_name.startswith('input/js'):
            self.set_status('Error', 'input/js0 is already connected')
            time.sleep(0.5)
            return
        mac_addr = DS4_NAME_TO_MAC[ds4_name].upper()
        self.set_status('Connection in progress...', 'Target device: %s (%s)\nRemoving it from inventory and acquire it again. ' % (ds4_name, mac_addr))
        time.sleep(1)
        # Removing the target and reconnect
        try:
            self.bl.remove(mac_addr)
        except bluetoothctl.BluetoothctlError as e:
            print(e)
        except pexpect.exceptions.TIMEOUT:
            pass
        self.bl.start_scan()
        time.sleep(1)

        paired = False
        attempts = 0
        MAX_ATTEMPTS=5
        while not paired and attempts < MAX_ATTEMPTS:
            attempts += 1
            self.set_status('Connection in progress', 'Scan and attempt to pair\nAttempt {}/{}\nPress ps+share on your DualShock4 until it blinks quickly...'.format(attempts, MAX_ATTEMPTS))
            try:
                paired = self.bl.pair(mac_addr)
            except bluetoothctl.BluetoothctlError as e:
                print(e)
            except pexpect.exceptions.TIMEOUT:
                pass
        if not paired:
            self.set_status('Error', 'Connection timed out... Please try again')
            time.sleep(1)
            return

        self.set_status('Connection in progress', 'Negotiating the remaning stuffs, keep the DS4 in alive...')
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
            self.set_status('Error', 'Connection timed out... Please try again')
            time.sleep(1)
            return

        self.set_status('Connected', 'Setting as trusted')
        try:
            self.bl.trust(mac_addr)
        except bluetoothctl.BluetoothctlError as e:
            print(e)
        except pexpect.exceptions.TIMEOUT:
            pass

        self.set_status('Connected', 'Connection complete')
        time.sleep(1)
        return

class ListDs4Button(npyscreen.ButtonPress):
    def __init__(self, *args, **keywords):
        super(ListDs4Button, self).__init__(*args, **keywords)

    def whenPressed(self):
        files = list(map(lambda x: x[5:], iter(glob.glob('/dev/ds4*'))))

        for ctr in range(0, 4):
            path = '/dev/input/js{}'.format(ctr)
            if os.path.exists(path):
                mac = detect_mac(path)
                files.append('{} ({})'.format(path[5:], mac))

        npyscreen.notify_wait("Connected devices: {}".format(files if len(files) > 0 else "None"))

class PingButton(npyscreen.ButtonPress):
    def __init__(self, *args, **keywords):
        super(PingButton, self).__init__(*args, **keywords)

    def whenPressed(self):
        npyscr_notify_terminal("ping", ["10.42.0.1"], timeout=10)

class ProperSlider(npyscreen.Slider):
    def __init__(self, *args, **keywords):
        super(ProperSlider, self).__init__(*args, **keywords)
    
    def translate_value(self):
        stri = "%.1f / %.1f" %(self.value, self.out_of)
        if isinstance(stri, bytes):
            stri = stri.decode(self.encoding, 'replace')
        l = len("%.1f"%1)*2+4
        stri = stri.rjust(l)
        return stri

class ProperTitleSider(npyscreen.TitleText):
    _entry_type = ProperSlider

DEVNULL = open(os.devnull, 'wb')

def detect_mac(file):
    cmd = subprocess.check_output(["udevadm", "info", "--attribute-walk", "-n", file], stderr=DEVNULL, universal_newlines=True)
    regex = re.compile('^[ \\t]*ATTRS\\{uniq\\}=="(([0-9a-f]{2}:){5}[0-9a-f]{2})"$')
    for line in cmd.split("\n"):
        match = regex.match(line)
        if match is not None:
            return match.group(1)
    return None

class tmux_helper(object):
    server = None
    session = None

    def __init__(self, *args, **keywords):
        self.server = libtmux.Server()
    
    def find_session(self):
        if self.server.has_session(TMUX_SESSION_NAME):
            self.session = self.server.find_where({ "session_name": TMUX_SESSION_NAME})
        else:
            self.session = self.server.new_session(TMUX_SESSION_NAME)
        return self.session
    
    def launch_tr(self, team, color, manual_vel, auto_vel, ds4_name):
        # t = self.server
        s = self.find_session()
        w = s.windows[0]
        p = s.attached_pane
        npyscreen.notify("Starting...")
        w.split_window(vertical=True, attach=False, target="0")
        w.split_window(vertical=False, attach=False, target="0")
        w.split_window(vertical=False, attach=False, target="2")
        w.split_window(vertical=True,MAX_SPEED attach=False, target="2")
        panes = w.list_panes()
        panes[2].send_keys('killall -9 rosmaster', enter=True, suppress_history=False)
        panes[2].send_keys('roscore', enter=True, suppress_history=False)

        for i in range(25):
            time.sleep(0.2)
            text = panes[2].capture_pane()
            if "started core service [/rosout]" in text:
                break
        time.sleep(0.25)

        if team == "rx":
            panes[4].send_keys('roslaunch m2_tr2020 base_hw_djimotor.launch', suppress_history=False)
        elif team == "rtx":
            panes[4].send_keys('roslaunch m2_tr2020 base_hw_pneumatic.launch', suppress_history=False)
        time.sleep(1.0)

        panes[0].send_keys('roslaunch m2_tr2020 localization_control.launch color:=%s'%color, suppress_history=False)
        time.sleep(1.0)

        panes[1].send_keys('roslaunch m2_tr2020 semi_auto.launch team:=%s color:=%s joy:=/dev/%s manual_vel:=%f auto_vel:=%f'%(team, color, ds4_name, manual_vel, auto_vel), suppress_history=False)
        time.sleep(1.0)

        panes[3].send_keys('rosbag record -a -o /home/m2/bags/', enter=True, suppress_history=False)
    
    def relaunch(self, pane_id):
        s = self.find_session()
        w = s.windows[0]
        panes = w.list_panes()
        for i in range(10):
            panes[pane_id].send_keys('^C', enter=False, suppress_history=False)
            time.sleep(0.05)
        for i in range(10):
            panes[pane_id].send_keys('^Z', enter=False, suppress_history=False)
            time.sleep(0.05)
        time.sleep(0.25)
        for i in range(10):
            panes[pane_id].send_keys('kill -9 %1 %2 %3', enter=True, suppress_history=True)
        time.sleep(0.5)
        panes[pane_id].cmd('send-keys', 'Up')
        panes[pane_id].enter()
        # panes[pane_id].send_keys(self.commands[pane_id], enter=True, suppress_history=False)

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
        self.auto_vel = self.add(ProperTitleSider, name="Auto MAX speed:",
            value=4.5, out_of=5.5, lowest=2.0, step=0.1, 
            max_width=70, rely=_rely,MAX_SPEED
        )
        _rely += 3
        self.manual_vel = self.add(ProperTitleSider, name="Manual MAX speed:",
            value=2.5, out_of=5.5, lowest=1.2, step=0.1,
            max_width=70, rely=_rely,
        )
        _rely += 3
        def ds4_names_and_js():
            for (k, _) in DS4_NAMES:
                yield k
            for ctr in range(0, 4):
                yield "input/js{}".format(ctr)
        self.ds4_name = self.add(npyscreen.TitleSelectOne,
            scroll_exit=True, name='Joystick name',
            values = list(ds4_names_and_js()),
            max_width=40, max_height=15, rely=_rely+2,
        )
        # _rely += 1
        self.list_ds4 = self.add(ListDs4Button, name="List DS4 devices", max_height=1, relx=0, rely=_rely)
        self.btn = self.add(connectButton, name="DualShock4 Connection Wizard",
            max_height=1, relx=0, rely=_rely + 1, ds4_name=self.ds4_name,
        )

        _relx += 42
        _rely += 2
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
        _rely += 3
        self.ping = self.add(PingButton, name="Ping AP",
                max_height=1, relx=_relx, rely=_rely)
        _rely += 1

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
                                manual_vel=self.manual_vel.value,
                                auto_vel=self.auto_vel.value,
                                ds4_name=self.ds4_name.get_selected_objects()[0]) # Spwan some threads to do tmux stuff
                self.parentApp.setNextForm("FORM2")

    def exit_application(self):
        # curses.beep()
        self.parentApp.setNextForm(None)
        self.editing = False
        self.parentApp.switchFormNow()

def exit_handler():
    server = libtmux.Server()
    try:
        session = server.find_where({ "session_name": TMUX_SESSION_NAME})
        window = session.attached_window
        panes=window.list_panes()
        for pane_id in range(len(panes))[::-1]:
            # i=len(panes)-i-1
            for i in range(4):
                panes[pane_id].send_keys('^C', enter=False, suppress_history=False)
                time.sleep(0.1)
            for i in range(4):
                panes[pane_id].send_keys('^Z', enter=False, suppress_history=False)
                time.sleep(0.1)
            time.sleep(0.1)
            for i in range(4):
                panes[pane_id].send_keys('kill -9 %1 %2 %3', enter=True, suppress_history=True)
                time.sleep(0.1)
            time.sleep(0.1)
            if pane_id > 0:
                panes[pane_id].send_keys("exit")
    except libtmux.exc.LibTmuxException as e:
        print("Error during tmux cleanup: {}".format(e))

class Form2(npyscreen.ActionFormV2):
    CANCEL_BUTTON_TEXT   = "Back"
    width=50
    height=10
    max_width=50
    max_height=10
    def create(self):
        self.option = self.add(npyscreen.TitleSelectOne,
        scroll_exit=True, name='Option', 
        values = ['Relaunch hardware','Relaunch localization', 'Relaunch semi_auto'],
        max_width=50, max_height=9)
        self.th = tmux_helper()

        
    def exit(self):
        self.parentApp.switchForm(None) 

    def on_ok(self):
        if len(self.option.get_selected_objects()) == 0:
            npyscreen.notify_confirm("You selected nothing!", title="Notice", wrap=True, wide=True, editw=1)
        elif self.option.get_selected_objects()[0] == 'Relaunch hardware':
            npyscreen.notify("Relaunching hardware...")
            self.th.relaunch(4)
        elif self.option.get_selected_objects()[0] == 'Relaunch localization':
            npyscreen.notify("Relaunching localization...")
            self.th.relaunch(0)
        elif self.option.get_selected_objects()[0] == 'Relaunch semi_auto':
            npyscreen.notify("Relaunching semi_auto...")
            self.th.relaunch(1)

    def on_cancel(self):
        self.parentApp.setNextForm("MAIN")
        npyscreen.notify("Exiting...")
        exit_handler()

if __name__ == '__main__':
    atexit.register(exit_handler)
    app = MyTmuxApp()
    app.run()
