from gpiozero import LED, PWMOutputDevice
import time
import threading

# Ratio vert/rouge pour obtenir un orange plutot qu'un jaune (R a fond, G attenue)
ORANGE_GREEN_LEVEL = 0.45


class FrontLEDs:
    """Contrôleur des LEDs avant du robot."""

    LED1 = 9
    LED2 = 25
    LED3 = 11
    Left_R = 13
    Left_G = 19
    Left_B = 0
    Right_R = 1
    Right_G = 5
    Right_B = 6
    
    def __init__(self):
        self.leds = {}
        self.blink_state = None
        self.blink_lock = threading.Lock()
        self._blink_thread = None
        self._running = False
        self.setup()
    
    def setup(self):
        """Initialise toutes les LEDs."""
        self.leds[1] = LED(self.LED1)
        self.leds[2] = LED(self.LED2)
        self.leds[3] = LED(self.LED3)
        self.leds[4] = PWMOutputDevice(self.Left_R, active_high=False)
        self.leds[5] = PWMOutputDevice(self.Left_G, active_high=False)
        self.leds[6] = PWMOutputDevice(self.Left_B, active_high=False)
        self.leds[7] = PWMOutputDevice(self.Right_R, active_high=False)
        self.leds[8] = PWMOutputDevice(self.Right_G, active_high=False)
        self.leds[9] = PWMOutputDevice(self.Right_B, active_high=False)
    
    def start(self):
        """Lance le thread de gestion des clignotants."""
        if self._running:
            return
        self._running = True
        self._blink_thread = threading.Thread(target=self._blink_worker, daemon=True)
        self._blink_thread.start()
    
    def stop(self):
        """Arrête le thread et éteint toutes les LEDs."""
        self._running = False
        self.set_all_switch_off()
        if self._blink_thread and self._blink_thread.is_alive():
            self._blink_thread.join(timeout=1.0)
    
    def switch(self, num, status):
        """Allume/éteint une LED spécifique."""
        if num not in self.leds:
            print('Wrong Command: num must be between 1 and 9')
            return
        if status == 1:
            self.leds[num].on()
        elif status == 0:
            self.leds[num].off()
        else:
            print('Wrong Command: status must be 0 or 1')
    
    def set_all_switch_off(self):
        """Éteint toutes les LEDs."""
        for num in self.leds:
            self.switch(num, 0)
    
    def stop_blinkers(self):
        """Éteint uniquement les LEDs de clignotants."""
        for num in (4, 5, 7, 8):
            self.switch(num, 0)
    
    def appel_phares(self):
        """Flash toutes les LEDs RGB 3 fois."""
        previous_state = self.blink_state
        with self.blink_lock:
            self.blink_state = None
        self.stop_blinkers()
        
        flash_duration = 0.08
        print("Appel de phares")
        for _ in range(3):
            for num in (4, 5, 6, 7, 8, 9):
                self.leds[num].value = 1.0
            time.sleep(flash_duration)
            for num in (4, 5, 6, 7, 8, 9):
                self.leds[num].value = 0.0
            time.sleep(flash_duration)
        
        with self.blink_lock:
            self.blink_state = previous_state
    
    def _set_orange(self, r_num, g_num, on):
        """Active/désactive une paire LED rouge-verte (pour l'orange)."""
        if on:
            self.leds[r_num].value = 1.0
            self.leds[g_num].value = ORANGE_GREEN_LEVEL
        else:
            self.leds[r_num].value = 0.0
            self.leds[g_num].value = 0.0
    
    def cancel_blink(self):
        """Arrête les clignotants."""
        with self.blink_lock:
            if self.blink_state is None:
                return
            self.blink_state = None
        self.stop_blinkers()
        print("Clignotant coupe (commande manuelle recue)")
    
    def set_blink(self, mode):
        """Active un mode de clignotant (left, right, warning, ou None)."""
        with self.blink_lock:
            self.blink_state = None if self.blink_state == mode else mode
            new_state = self.blink_state
        
        if new_state is None:
            self.stop_blinkers()
            print("Clignotants eteints")
        elif new_state == 'left':
            print("Clignotant gauche active")
        elif new_state == 'right':
            print("Clignotant droit active")
        elif new_state == 'warning':
            print("Warnings actives")
    
    def _blink_worker(self):
        """Thread worker pour la gestion des clignotants."""
        blink_period = 0.5
        while self._running:
            with self.blink_lock:
                state = self.blink_state
            
            if state == 'left':
                self._set_orange(4, 5, True)
                time.sleep(blink_period)
                self._set_orange(4, 5, False)
                time.sleep(blink_period)
            elif state == 'right':
                self._set_orange(7, 8, True)
                time.sleep(blink_period)
                self._set_orange(7, 8, False)
                time.sleep(blink_period)
            elif state == 'warning':
                self._set_orange(4, 5, True)
                self._set_orange(7, 8, True)
                time.sleep(blink_period)
                self._set_orange(4, 5, False)
                self._set_orange(7, 8, False)
                time.sleep(blink_period)
            else:
                time.sleep(0.1)


if __name__ == "__main__":
    leds = FrontLEDs()
    leds.start()  
    try:
        while True:
            code = input()
            if code == 'q':
                break
            elif code == 'cl':
                leds.set_blink('left')
                continue
            elif code == 'cr':
                leds.set_blink('right')
                continue
            elif code == 'war':
                leds.set_blink('warning')
                continue
            elif code == 'ap':
                leds.appel_phares()
                continue
            
            try:
                code = int(code)
            except ValueError:
                print('Wrong Command: Example--11 pour allumer LED1, cl/cr/war pour les clignotants')
                continue
            
            leds.cancel_blink()
            
            action = code // 10
            num = code % 10
            
            if action == 1:
                leds.switch(num, 1)
            elif action == 2:
                leds.switch(num, 0)
            else:
                print('Wrong Command: Example--11 pour allumer LED1')
    except KeyboardInterrupt:
        pass
    finally:
        leds.stop()

