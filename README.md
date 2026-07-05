# 🤖 VigiBot — Robot Autonome de Maintenance Industrielle

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.x](https://img.shields.io/badge/Python-3.x-blue.svg)](https://www.python.org/)
[![Flask](https://img.shields.io/badge/Flask-Interface%20Web-000000.svg?logo=flask)](https://flask.palletsprojects.com/)
[![Status: Beta](https://img.shields.io/badge/Status-Beta-orange.svg)]()
[![Platform: Raspberry Pi 4](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-c51a4a.svg?logo=raspberry-pi)](https://www.raspberrypi.com/)
[![Hardware: Adeept PiCar-B2](https://img.shields.io/badge/Kit-Adeept%20PiCar--B2-lightgrey.svg)](https://github.com/adeept/adeept_picar-b2)
[![Fork](https://img.shields.io/badge/Fork%20de-adeept%2Fadeept__picar--b2-6e5494.svg?logo=github)](https://github.com/adeept/adeept_picar-b2)
[![École: EFREI Paris](https://img.shields.io/badge/École-EFREI%20Paris-0033A0.svg)]()

> VigiBot est une plateforme robotique autonome dédiée à l'inspection industrielle, développée sur base du kit **Adeept PiCar-B2**. Le projet est mené dans le cadre d'un cursus **EFREI Paris**, avec pour ambition de démontrer une solution de navigation autonome modulaire, robuste et entièrement pilotable depuis une interface web embarquée.

---

## 📑 Sommaire

- [Contexte du projet](#-contexte-du-projet)
- [Fonctionnalités](#-fonctionnalités)
- [Architecture matérielle](#-architecture-matérielle)
- [Architecture logicielle](#-architecture-logicielle)
- [Interface web & pilotage](#-interface-web--pilotage)
- [Structure du dépôt](#-structure-du-dépôt)
- [Installation](#-installation)
- [Utilisation](#-utilisation)
- [Feuille de route](#-feuille-de-route)
- [Contribuer](#-contribuer)
- [Équipe & Contributeurs](#-équipe--contributeurs)
- [Licence](#-licence)
- [Remerciements](#-remerciements)

---

## 🎯 Contexte du projet

VigiBot répond à une problématique industrielle concrète : comment automatiser l'inspection d'une zone de production (allées, périmètres, rangées d'équipements) sans intervention humaine directe, tout en garantissant une navigation sûre et prévisible du robot ?

Le cahier des charges du projet impose au robot de savoir évoluer dans **4 zones** distinctes :

1. **Zone ligne caméra** : suivre un tracé rouge au sol par vision (`camera_line`).
2. **Zone ligne infrarouge** : suivre un tracé noir au sol par capteurs IR (`line_following`).
3. **Zone obstacles** : détecter et contourner des obstacles grâce aux capteurs à ultrasons (`obstacles`).
4. **Zone labyrinthe** : naviguer en environnement contraint en reconnaissant des flèches par caméra pour déterminer la direction à suivre (`labyrinthe`).

Le projet part du dépôt open source **[adeept/adeept_picar-b2](https://github.com/adeept/adeept_picar-b2)**, fourni avec le kit robotique, que nous avons forké et largement réécrit pour y implémenter notre propre logique de navigation autonome. 

---

## 🚀 Fonctionnalités

Le robot évolue au sein de **4 zones fonctionnelles principales**, chacune associée à son propre bloc de code et package, et orchestrées par un module de **transitions** qui gère le passage de l'une à l'autre selon le contexte détecté.

### Les 4 zones

| Zone                                       | Capteurs utilisés | Description |
|--------------------------------------------|---|---|
| 🔴 **`camera_line`** — Ligne caméra        | Caméra | Traitement d'image en temps réel (segmentation colorimétrique) pour suivre un tracé rouge au sol. Plusieurs itérations (`camera_line.py`, `camera_line2.py`, `camera_line3.py`) tracent l'évolution de l'algorithme. |
| ⚫️ **`line_following`** — Ligne infrarouge | Infrarouges | Suivi de ligne par réflectométrie IR (`t11_line_following.py`), avec gestion multi-thread (`t11_threads.py`), signalisation sonore (`t11_buzzer_Sirene.py`) et configuration en ligne de commande (`t11_argument_parser.py`). |
| 🚧 **`obstacles`** — Obstacles             | Ultrasons | Mesure de distance en continu (`avoid_objects.py`) et déclenchement d'une manœuvre de contournement, avec une version multi-thread pour un évitement plus réactif (`avoid_objects_threads.py`). |
| ↗️ **`labyrinthe`** — Labyrinthe           | Caméra | Navigation en environnement contraint par reconnaissance de flèches (`CameraDetection.py`, `IdentificationCamera.py`) déterminant la direction à suivre à chaque intersection (`labyrinthe.py`, version multi-thread : `labyrinthe_threads.py`). |

### Modules complémentaires

| Module | Rôle |
|---|---|
| 🔀 **`transitions`** | Machine à états assurant le passage cohérent d'une zone à l'autre selon l'environnement détecté et le mode sélectionné (`transitions.py`, `TransitionLineFollowing.py`). |
| 💡 **`light_following`** | Comportement de suivi d'une source lumineuse, avec une version « réelle » (`t10_real_light_following.py`) affinée par rapport au prototype initial (`t8_light_following.py`). |
| 🎮 **`controls`** | Mode de contrôle manuel du robot au clavier (`t9_keyboard_control.py`), utile pour les tests et la calibration. |
| 📝 **`logging`** | Journalisation centralisée des événements et du comportement du robot (`logger.py`), utile au débogage et au suivi des sessions. |

Chaque **composant matériel de base** est par ailleurs développé et validé **indépendamment** dans `work/components/`, en suivant la numérotation des tutoriels vus en cours (`t1` à `t7`) : LED avant/arrière, servomoteurs, moteur DC, capteur à ultrasons, suivi de ligne basique, puis intégration combinée des fonctions (`t7_functions_integration.py`). Chaque script embarque son propre `main` de test, exécutable directement pour valider le matériel de façon isolée.

---

## 🛠 Architecture matérielle

| Composant | Rôle | Détail                                                                   |
|---|---|--------------------------------------------------------------------------|
| **Raspberry Pi 4 (4 Go RAM)** | Unité de calcul principale | Exécute la vision, la navigation, le serveur Flask et le contrôle moteur |
| **Kit Adeept PiCar-B2** | Châssis, moteurs, carte de contrôle | Base mobile motorisée, servomoteurs, carte pilote                        |
| **Caméra (Pi Camera)** | Vision | Suivi de ligne rouge, reconnaissance visuelle, flux vidéo live           |
| **Capteurs infrarouges** | Suivi de ligne au sol | Détection de ligne noire, détection des limites de zone sécurisée        |
| **Capteurs à ultrasons** | Mesure de distance | Détection d'obstacles, détection de panneaux                             |
| **Module Wi-Fi (hotspot embarqué)** | Connectivité | Point d'accès autonome permettant de se connecter directement au robot.  |
| **Batterie embarquée** | Autonomie énergétique | Alimentation du Raspberry Pi et des moteurs                              |

```
┌──────────────────────────────┐
│         Raspberry Pi 4       │
│  (vision, décision, Flask)   │
└───────────┬───────────────────┘
            │
   ┌────────┼─────────┬────────────────┐
   │        │         │                │
 Caméra   Capteurs  Capteurs      Contrôleur
         Infrarouges Ultrasons    moteurs/servos
                                 (HAT du kit Adeept PiCar-B2)
```

---

## 💻 Architecture logicielle

Le code est développé en **Python 3**, en fork du dépôt Adeept original, réorganisé autour de deux grands principes : **isolation des composants** et **modularité des comportements**.

- **`work/components/`** : chaque composant matériel de base (LED, servomoteurs, moteur DC, ultrasons, suivi de ligne basique) possède son propre module Python, testable de manière autonome, avec un `main` de test intégré directement dans son fichier, permettant de valider chaque brique séparément avant intégration.
- **`work/camera_line/`**, **`work/line_following/`**, **`work/obstacles/`**, **`work/labyrinthe/`** : les **4 zones** de comportement principales de l'application, chacune encapsulant sa propre logique de perception + décision + actionnement.
- **`work/controls/`** : mode de pilotage manuel au clavier, utilisé pour les tests et la calibration.
- **`work/light_following/`** : comportement complémentaire de suivi de source lumineuse.
- **`work/transitions/`** : la logique de commutation entre zones, qui décide quand passer de l'une à l'autre (ex. passage du suivi de ligne à l'évitement d'obstacle en cas de détection).
- **`work/logging/`** : journalisation centralisée du comportement du robot, partagée par l'ensemble des blocs.
- **`work/main.py`** *(application principale)* : point d'entrée qui initialise les capteurs, lance la boucle de contrôle et embarque **le serveur Flask**.

---

## 🌐 Interface web & pilotage

L'application principale embarque un serveur Flask exposant une interface web permettant de :

- 📷 **Visualiser en direct** le flux de la caméra du robot ;
- ▶️⏹ **Démarrer / arrêter** le robot à distance ;
- 🎛 **Choisir le mode / la zone de départ** (ligne caméra, ligne infrarouge, obstacles, labyrinthe) avant de lancer une session.

---

## 📁 Structure du dépôt

```text
Vigibot/
├── work/                            # 🧠 Tout le travail de l'équipe
│   ├── camera_line/                 # Bloc : suivi de ligne rouge (caméra)
│   │   ├── __init__.py
│   │   ├── camera_line.py
│   │   ├── camera_line2.py
│   │   └── camera_line3.py
│   ├── components/                  # Composants matériels testés individuellement (tutoriels Adeept)
│   │   ├── __init__.py
│   │   ├── clignotants.py
│   │   ├── t1_front_led.py
│   │   ├── t2_back_led.py
│   │   ├── t3_servomotors.py
│   │   ├── t4_dc_motor.py
│   │   ├── t5_ultrasonic_sensor.py
│   │   ├── t6_line_tracking.py
│   │   └── t7_functions_integration.py
│   ├── controls/                    # Pilotage manuel
│   │   ├── __init__.py
│   │   └── t9_keyboard_control.py
│   ├── labyrinthe/                  # Bloc : navigation en labyrinthe (caméra)
│   │   ├── CameraDetection.py
│   │   ├── IdentificationCamera.py
│   │   ├── __init__.py
│   │   ├── labyrinthe.py
│   │   └── labyrinthe_threads.py
│   ├── light_following/             # Bloc : poursuite de source lumineuse
│   │   ├── __init__.py
│   │   ├── t10_real_light_following.py
│   │   ├── t8_light_following.py
│   │   └── t8_light_following2.py
│   ├── line_following/              # Bloc : suivi de ligne noire (infrarouges)
│   │   ├── __init__.py
│   │   ├── t11_argument_parser.py
│   │   ├── t11_buzzer_Sirene.py
│   │   ├── t11_line_following.py
│   │   ├── t11_robot.py
│   │   └── t11_threads.py
│   ├── logging/                     # Journalisation
│   │   ├── __init__.py
│   │   └── logger.py
│   ├── obstacles/                   # Bloc : évitement d'obstacles (ultrasons)
│   │   ├── __init__.py
│   │   ├── avoid_objects.py
│   │   └── avoid_objects_threads.py
│   ├── pannels/                     # Détection visuelle complémentaire (labyrinthe)
│   │   ├── __init__.py
│   │   ├── panneaux_detect.py
│   │   ├── pannel_test.py
│   │   └── test_cam_pannel.py
│   ├── transitions/                 # Machine à états : passage entre les blocs
│   │   ├── TransitionLineFollowing.py
│   │   ├── __init__.py
│   │   └── transitions.py
│   └── main.py                      # Point d'entrée + serveur Flask (pilotage web)
├── web/                             
├── flask-video-streaming/           
├── examples/                        # Exemples fournis par Adeept (dépôt d'origine)
├── DocEfrei/                        
├── initPosServos.py                 
├── setup.py                         
├── wifi_hotspot_manager.sh          
├── README_ADEEPT.md                 
└── README.md                        
```

> 📌 Tout le travail spécifique à VigiBot (au-delà du fork Adeept) se trouve dans **`work/`**.

---

## ⚙️ Installation

```bash
# Cloner le dépôt
git clone https://github.com/Sowker/Vigibot.git
cd Vigibot

# Installer les dépendances
python setup.py install
# ou, selon l'environnement :
pip install -r requirements.txt

# Initialiser la position des servomoteurs (une fois le matériel branché)
python initPosServos.py
```

---

## ▶️ Utilisation

```bash
cd work
python main.py
```

Une fois lancé, `main.py` démarre le serveur Flask embarqué. Il suffit alors d'ouvrir l'interface web (adresse IP du robot, port par défaut Flask) pour :

1. Visualiser le flux caméra en direct ;
2. Sélectionner la zone de départ (`camera_line`, `line_following`, `obstacles`, `labyrinthe`) ;
3. Démarrer ou arrêter le robot à tout moment.

Les transitions entre zones sont ensuite gérées automatiquement par le module `transitions/` en fonction de l'environnement détecté.

### Tester un composant isolément

Chaque composant de `work/components/` peut être exécuté indépendamment pour validation matérielle :

```bash
python work/components/t5_ultrasonic_sensor.py
python work/components/t3_servomotors.py
python work/components/t7_functions_integration.py
```

---

## 🗺 Feuille de route

- [x] Suivi de ligne rouge par caméra
- [x] Suivi de ligne noire par infrarouge
- [x] Détection et évitement d'obstacles
- [x] Interface web Flask (flux vidéo + pilotage + choix de zone)
- [x] Navigation autonome complète en labyrinthe par reconnaissance de flèches (`labyrinthe`)
- [x] Fiabilisation des transitions entre zones en conditions réelles
- [x] Amélioration de la robustesse en conditions de faible luminosité
- [x] Historique et journalisation des sessions d'inspection

---

## 👥 Équipe & Contributeurs

Projet réalisé dans le cadre d'un cursus à **EFREI Paris**, sur base du kit et du dépôt open source **Adeept PiCar-B2**.

| Nom | Rôle                                                                                                              | GitHub                                         |
|---|-------------------------------------------------------------------------------------------------------------------|------------------------------------------------|
| **Rémi** | Navigation en labyrinthe & reconnaissance de flèches (`labyrinthe`), suivi de ligne infrarouge (`line_following`) | [@Sowker](https://github.com/Sowker)           |
| **Pierre** | Évitement d'obstacles & ultrasons (`obstacles`)                                                                   | [@PierreMo](https://github.com/PierreMo)       |
| **Eliott** | Évitement d'obstacles & ultrasons (`obstacles`)                                                                   | [@Eliott](https://github.com/Eliott)           |
| **Antoine** | Machine à états (`transitions`), interface web (`main`), ligne caméra (`camera_line`)                             | [@ant-one-dev](https://github.com/ant-one-dev) |
| **Tristan** | Maintenance du code et actions                                                                                    | [@Tritrinut](https://github.com/Tritrinut)     |

---

## 📄 Licence

Ce projet est distribué sous licence **MIT**.

---

## 🙏 Remerciements

- **EFREI Paris**, pour l'encadrement pédagogique de ce projet.
- **[Adeept](https://github.com/adeept/adeept_picar-b2)**, pour le kit PiCar-B2 et le dépôt original ayant servi de base au fork.
- La communauté open source Python / OpenCV / Flask / Raspberry Pi.
