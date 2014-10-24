PRE-ISTRUZIONI:

Per evitare di dover eseguire il comando "source devel/setup.sh" manualmente
ogni volta che si apre un nuovo terminale, effettua i seguenti passi:

 (1) Apri un terminale

 (2) Esegui: gedit ~/.bashrc

 (3) Nel file appena aperto aggiungi alla fine le seguenti 4 righe
     (se avete chiamato il workspace catkin diversamente da "catkin_ws"
     fare le opportune modifiche):

# To add the env paths for catkin workspace automatically.
if [ -f ~/catkin_ws/devel/setup.sh ]; then
    . ~/catkin_ws/devel/setup.sh
fi

 (4) Salva e chiudi

I passi appena effettuati tornano utili quando si utilizza il comando "roslaunch",
che altrimenti darebbe errore (non riuscendo a trovare gli eseguibili/nodi ROS).

----------------------------------------------------------------------------------

ISTRUZIONI PER L'USO:

 (1) Copia la cartella "quadcopter_ctrl" all'interno di:
     "catkin_ws/src"
     e copia la cartella "Input" in:
     "catkin_ws/devel/lib/quadcopter_ctrl"
     
 (2) Compila con "catkin_make"
 
 (3) Apri un terminale ed esegui il comando: roscore
 
 (4) In un altro terminale apri V-REP (l'apertura dopo "roscore" è fondamentale). Dal
     menù di V-REP apri la scena "3rob_simulationEnv.ttt" situata in:
     "~/catkin_ws/src/quadcopter_ctrl"
 
 (5) In un terzo terminale esegui il comando: 
     "roslaunch quadcopter_ctrl swarmNodeCount_3.launch input:=access_mat_subs"
     e i quadricotteri in V-REP partiranno.
 
 ---------------------------------------------------------------------------------
 
 NOTE A MARGINE:
 
 - Chiudere il terminale "roscore" prima di V-REP può causare crash del PC.
 - Per segnale un malfunzionamento o un problema mandami una mail a:
   fwanderlingh@gmail.com
 - Guarda dietro di te! Una scimmia a tre teste! 
 
 
