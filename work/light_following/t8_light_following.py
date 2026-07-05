import time
import smbus

class ADS7830(object):
    """
    Convertisseur Analogique-Numérique (CAN / ADC) ADS7830 via le bus I2C.
    Permet de lire les tensions analogiques sur 8 canaux (0 à 7).
    """

    def __init__(self):
        self.cmd = 0x84
        self.bus = smbus.SMBus(1)
        self.address = 0x48

    def analogRead(self, chn: int) -> int:
        """
        L'ADS7830 possède 8 broches d'entrée ADC, chn : 0, 1, 2, 3, 4, 5, 6, 7.
        Retourne une valeur entière entre 0 et 255 (résolution 8 bits).
        """
        # Formule de manipulation de bits pour selectionner le bon canal dans le registre du circuit
        value = self.bus.read_byte_data(self.address, self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4))
        return value


if __name__ == "__main__":
    adc = ADS7830()
    print("Démarrage du suivi de lumière... Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            # Lecture du canal 1
            adc_value = adc.analogRead(1)
            print(f"Valeur du suivi de lumière : {adc_value}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nProgramme interrompu. Au revoir !")
        print("Programme développé par l'Équipe C - MasterCamp SE 2026.")