#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

class ControllerSwitcher:
    def __init__(self):
        rospy.init_node('gamepad_controller_switcher')
        
        # Hangi tuşların hangi kontrolcüye geçeceğini belirleyelim
        # GamePad_node.cpp'deki SDL2 haritalamanıza göre buton indekslerini ayarlayın.
        # Örneğin: 0 (A/Çarpı), 1 (B/Yuvarlak), vb.
        self.BTN_A = 0 # A (Çarpı) tuşuna basınca Squat'a geçsin
        self.BTN_B = 1  # B (Yuvarlak) tuşuna basınca WBIC'e geçsin
        self.BTN_LB = 4 # X (Kare) tuşuna basınca Yürüyüşe geçsin
        self.BTN_PS = 10; # PS button on playstation controller
        
        self.prev_buttons = []
        self.prev_SQUAT_CMD = False
        self.prev_WBIC_CMD = False
        self.prev_HELP_CMD = False
        
        # Mevcut çalışan kontrolcüyü takip edelim
        self.current_controller = None
        
        # Servisin hazır olmasını bekle
        rospy.loginfo("Waiting for /controller_manager/switch_controller service...")
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        rospy.loginfo("Service found!")

        # Başlangıçta varsayılan kontrolcüyü (Squat) başlatalım
        # rospy.loginfo(f"Starting default controller: {self.current_controller}")
        # self.call_switch_service(start=[self.current_controller], stop=[])

        # Joy mesajlarını dinle
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.loginfo("Ready to switch controllers via GamePad!")

    def joy_callback(self, msg):
        self.SQUAT_CMD = msg.buttons[self.BTN_A] and msg.buttons[self.BTN_LB]
        self.WBIC_CMD = msg.buttons[self.BTN_B] and msg.buttons[self.BTN_LB]
        self.HELP_CMD = msg.buttons[self.BTN_PS]
        if not self.prev_buttons:
            self.prev_buttons = msg.buttons
            self.prev_SQUAT_CMD = self.SQUAT_CMD
            self.prev_WBIC_CMD = self.WBIC_CMD
            self.prev_HELP_CMD = self.HELP_CMD
            return

        # HELP FONKSIYONU
        if self.HELP_CMD == 1 and self.prev_HELP_CMD == 0:
            self.print_help_info()
        
        # SQUAT KONTROLCÜSÜNE GEÇİŞ (Buton 0 - A)
        if self.SQUAT_CMD == 1 and self.prev_SQUAT_CMD == 0:
            if self.current_controller != 'controllers/go2_squat_controller':
                stop_list = [self.current_controller] if self.current_controller is not None else []
                self.call_switch_service(
                    start=['controllers/go2_squat_controller'], 
                    stop=stop_list
                )
                self.current_controller = 'controllers/go2_squat_controller'
            else:
                rospy.loginfo("Already running go2_squat_controller!")

        # WBIC KONTROLCÜSÜNE GEÇİŞ (Buton 1 - B)
        elif self.WBIC_CMD == 1 and self.prev_WBIC_CMD == 0:
            if self.current_controller != 'controllers/go2_wbic':
                stop_list = [self.current_controller] if self.current_controller is not None else []
                self.call_switch_service(
                    start=['controllers/go2_wbic'], 
                    stop=stop_list
                )
                self.current_controller = 'controllers/go2_wbic'
            else:
                rospy.loginfo("Already running go2_wbic!")

        # Önceki buton durumunu güncelle
        self.prev_buttons = msg.buttons
        self.prev_SQUAT_CMD = self.SQUAT_CMD
        self.prev_WBIC_CMD = self.WBIC_CMD
        self.prev_HELP_CMD = self.HELP_CMD

    def call_switch_service(self, start, stop):
        rospy.loginfo(f"Switching controllers... Starting: {start}, Stopping: {stop}")
        
        req = SwitchControllerRequest()
        req.start_controllers = start
        req.stop_controllers = stop
        req.strictness = SwitchControllerRequest.STRICT # Biri hata verirse işlemi iptal et
        req.start_asap = False
        req.timeout = 0.0 
        
        try:
            resp = self.switch_srv(req)
            if resp.ok:
                rospy.loginfo("Controller switch successful!")
            else:
                rospy.logwarn("Controller switch failed! Check if the controller is loaded.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def print_help_info(self):
        print("\n\n\n")
        print("**************** BUTTON CONFIGURATIONS ****************")
        print("*                                                     *")
        print("*  L1 + X ------------------> Start Squat Controller  *")
        print("*       Square -------------> Stand robot up          *")
        print("*       Circle -------------> Sit robot down          *")
        print("*                                                     *")
        print("*  L1 + Circle -------------> Start WBIC Controller   *")
        print("*       Options ------------> Walking mode on/off     *")
        print("*       Dpad Left/Right ----> Change gait             *")
        print("*                                                     *")
        print("*  PS Button ---------------> Show this help          *")
        print("*******************************************************")
        print("\n\n\n")
        return

if __name__ == '__main__':
    try:
        ControllerSwitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass