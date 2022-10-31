# *"Angular Lane Following"*
La guida ha l'obiettivo di descrivere come utilizzare il codice della presente repository per avviare una versione alternativa dell'algoritmo originale di *lane following* del progetto *Duckietown* su un *duckiebot*. 

L'algoritmo di *lane following* è implementato dal codice che si trova in `pkgs`, nella presente repository. All'interno sono presenti due package:

 - ***line_detector***, che effettua le operazioni di *lane detection* dell'algoritmo;
 - ***lane_control***, che genera l'azione di controllo sul duckiebot.

Il Dockerfile è invece utilizzato per generare l'immagine Docker necessaria ad avviare l'algoritmo, come descritto nella sezione successiva.


## Avviare l'algoritmo
Per poter avviare l'algoritmo è necessario disporre, all'interno del proprio duckiebot, dell'immagine Docker `duckietown/dt-core:daffy-arm32v7`. Se il duckiebot è stato inizializzato correttamente, come illustrato nella [documentazione ufficiale](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_duckiebot.html), l'immagine è già presente. 

Per poter avviare l'algoritmo, è necessario inserire il codice all'interno dell'immagine `duckietown/dt-core:daffy-arm32v7`. Per farlo è necessario non solo l'interfaccia a riga di comando del proprio PC, ma anche di quella del proprio duckiebot. Quest'ultima può essere acceduta attraverso SSH con il comando

    ssh duckie@<DUCKIEBOT_NAME>.local

eseguito dal proprio PC; è normalmente richiesta una password - di default *quackquack*.

È possibile inserire il codice nello stack software ufficiale attraverso i passaggi seguenti:

 1. All'interno del duckiebot, ri-taggare l'immagine `duckietown/dt-core:daffy-arm32v7` con il comando `docker tag duckietown/dt-core:daffy-arm32v7 duckietown/dt-core:backup` ed eliminare il vecchio tag con il comando `docker rmi duckietown/dt-core:daffy-arm32v7`.
 2. All'interno del proprio PC, clonare la presente repository da GitHub.
 3. Copiare nel duckiebot la cartella `pkgs` della repository scaricata utilizzando il comando `scp -r pkgs/ duckie@<DUCKIEBOT_NAME>.local:/home/duckie`. Si suppone di essere nella directory in cui si trova `pkgs`, altrimenti sarà necessario specificare il percorso (relativo o assoluto).
 6. Copiare nel duckiebot il Dockerfile della repository utilizzando il comando `scp Dockerfile duckie@<DUCKIEBOT_NAME>.local:/home/duckie`. Si suppone di essere nella directory in cui si trova tale file, altrimenti sarà necessario specificare il percorso (relativo o assoluto).
 8. Nel duckiebot, spostarsi nella directory in cui sono memorizzati `pkgs` e il Dockerfile; come risultato dei comandi precedenti la directory è `/home/duckie`.
 9. Eseguire il comando `docker build -t duckietown/dt-core:daffy-arm32v7 --build-arg PACKAGES=pkgs .` nel duckiebot.

Una volta terminati questi passi, si potrà avviare l'algoritmo avviando la demo del *lane following* con il comando

    dts duckiebot demo --demo_name lane_following --duckiebot_name <DUCKIEBOT_NAME> --package_name duckietown_demos

Prima di eseguire il comando assicurarsi, ad esempio attraverso Portainer, che i container `dt-duckiebot-interface` e `dt-car-interface` siano in esecuzione.

## Funzionamento dell'algoritmo
Una volta eseguito il comando precedente è possibile avviare l'algoritmo attraverso il joystick virtuale di Duckietown. Eseguire dal proprio PC il comando

    dts duckiebot keyboard_control <DUCKIEBOT_NAME>


per avviare il joystick virtuale. Come è possibile leggere sulle istruzioni che compariranno nel terminale, l'algoritmo può essere avviato premendo 'A' e fermato premendo 'S'. 

È possibile approfondire il funzionamento dei nodi ROS presenti in *line_detector* e *lane_control* grazie ai commenti presenti nel codice.

### Configurazione
È possibile configurare i parametri dell'algoritmo grazie alla funzionalità di *riconfigurazione dinamica*. 
Accedere alla rete ROS del duckiebot da PC eseguendo il comando 

    dts start_gui_tools <DUCKIEBOT_NAME>

La riconfigurazione dinamica può essere acceduta eseguendo successivamente il comando

    rosrun rqt_gui rqt_gui -s reconfigure 

Poiché sarà sicuramente necessario disporre *anche* di una finestra di *rqt_image_view*, è consigliato di accedere a `no-vnc` eseguendo da PC il comando 

    dts start_gui_tools --vnc <DUCKIEBOT_NAME>

e navigando poi, attraverso un qualsiasi browser web, verso `http://localhost:8087/`.
