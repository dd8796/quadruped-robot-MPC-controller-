# cheetah_locomotion — origine du code

Locomotion de **MIT Cheetah-Software** (https://github.com/mit-biomimetics/Cheetah-Software,
commit `c71c5a138d3e418cc833e94e25357ceea8955daa`, licence MIT : `LICENSE.Cheetah-Software`),
adaptée à notre quadrupède. qpOASES 3.2 (LGPL 2.1) est repris tel que fourni dans
`third-party/qpOASES` du même dépôt, compilé en bibliothèque partagée.

Toute modification par rapport à l'original est marquée `QUADRU:` dans le code.

| Fichier | Origine | Modifications |
|---|---|---|
| `cppTypes.h`, `cTypes.h`, `Math/*.h`, `Gait.{h,cpp}`, `common_types.h`, `SolverMPC.h` | copie | aucune |
| `FootSwingTrajectory.{h,cpp}` | `common/.../Controllers` | chemin d'include ; sommet du vol = max(départ, arrivée) + hauteur (montée de marche) |
| `convexMPC_interface.{h,cpp}` | `convexMPC/` | `K_MAX_GAIT_SEGMENTS` 36 → 80 (débordement du tableau `gait` dans l'original) ; `get_solve_status()` |
| `SolverMPC.cpp` | `convexMPC/` | chemin qpOASES seul (`use_jcqp = 0`, défaut MIT) ; statut du QP |
| `RobotState.{h,cpp}` | `convexMPC/` | masse et inertie du robot en paramètre (codées en dur pour le Mini Cheetah) |
| `ConvexMPCLocomotion.{h,cpp}` | `convexMPC/` | entrées/sorties par `robot_interface.h` (pas de LCM/ControlFSMData) ; géométrie et allure du robot en paramètre ; régularisation `alpha` à l'échelle de la masse ; saut, MPC creux et radiocommande retirés ; point d'entrée d'adaptation d'appui (perception), hauteur du sol et tangage du terrain ; tenue de cap sans consigne de rotation ; accélération de consigne du pied en vol transmise (`LegCommand::aDes`) |
| `StateEstimator.{h,cpp}` | `OrientationEstimator.cpp`, `PositionVelocityEstimator.cpp`, `ContactEstimator` | regroupés ; lacet initial conservé (repère odométrie) ; initialisation à la pose de départ ; hauteur du sol sous les pieds ; localisation externe (`overridePose`, comme `FSM_State_Vision::_updateStateEstimator`) |
| `robot_interface.h` | nouveau | remplace `ControlFSMData`, `LegController`, `StateEstimatorContainer`, `Quadruped` |
